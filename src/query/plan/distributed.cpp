#include "query/plan/distributed.hpp"

#include <memory>

#include "query/plan/operator.hpp"
#include "query/plan/preprocess.hpp"
#include "utils/exceptions.hpp"

namespace query::plan {

namespace {

int64_t AddWorkerPlan(DistributedPlan &distributed_plan,
                      std::atomic<int64_t> &next_plan_id,
                      const std::shared_ptr<LogicalOperator> &worker_plan) {
  int64_t plan_id = next_plan_id++;
  distributed_plan.worker_plans.emplace_back(plan_id, worker_plan);
  return plan_id;
}

// Find the subtree parent, below which no operator uses symbols found in the
// `forbidden_symbols` set.
class IndependentSubtreeFinder : public HierarchicalLogicalOperatorVisitor {
 public:
  IndependentSubtreeFinder(
      const std::vector<std::vector<Symbol>> &forbidden_symbols,
      const SymbolTable *symbol_table)
      : forbidden_symbols_(forbidden_symbols), symbol_table_(symbol_table) {}

  using HierarchicalLogicalOperatorVisitor::PostVisit;
  using HierarchicalLogicalOperatorVisitor::PreVisit;
  using HierarchicalLogicalOperatorVisitor::Visit;

  // These don't use any symbols
  bool Visit(Once &) override { return true; }
  bool Visit(CreateIndex &) override { return true; }

  bool PostVisit(ScanAll &scan) override { return true; }
  bool PostVisit(ScanAllByLabel &scan) override { return true; }

  bool PostVisit(ScanAllByLabelPropertyRange &scan) override {
    if (subtree_) return true;
    if (scan.lower_bound()) {
      UsedSymbolsCollector collector(*symbol_table_);
      scan.lower_bound()->value()->Accept(collector);
      if (ContainsForbidden(collector.symbols_)) {
        throw utils::NotYetImplemented("distributed Cartesian planning");
      }
    }
    if (scan.upper_bound()) {
      UsedSymbolsCollector collector(*symbol_table_);
      scan.upper_bound()->value()->Accept(collector);
      if (ContainsForbidden(collector.symbols_)) {
        throw utils::NotYetImplemented("distributed Cartesian planning");
      }
    }
    return true;
  }

  bool PostVisit(ScanAllByLabelPropertyValue &scan) override {
    if (subtree_) return true;
    UsedSymbolsCollector collector(*symbol_table_);
    scan.expression()->Accept(collector);
    if (ContainsForbidden(collector.symbols_)) {
      throw utils::NotYetImplemented("distributed Cartesian planning");
    }
    return true;
  }

  bool PostVisit(Expand &exp) override {
    if (subtree_) return true;
    if (auto found = FindForbidden(exp.input_symbol())) {
      subtree_ = exp.input();
      parent_ = &exp;
      depends_on_ = found;
    }
    if (exp.existing_node()) {
      if (auto found = FindForbidden(exp.node_symbol())) {
        subtree_ = exp.input();
        parent_ = &exp;
        if (depends_on_) {
          depends_on_ = std::min(*found, *depends_on_);
        } else {
          depends_on_ = found;
        }
      }
    }
    CHECK(!FindForbidden(exp.edge_symbol()))
        << "Expand uses an already used edge symbol.";
    return true;
  }

  bool PostVisit(ExpandVariable &exp) override {
    if (subtree_) return true;
    if (auto found = FindForbidden(exp.input_symbol())) {
      subtree_ = exp.input();
      parent_ = &exp;
      depends_on_ = found;
    }
    if (exp.existing_node()) {
      if (auto found = FindForbidden(exp.node_symbol())) {
        subtree_ = exp.input();
        parent_ = &exp;
        if (depends_on_) {
          depends_on_ = std::min(*found, *depends_on_);
        } else {
          depends_on_ = found;
        }
      }
    }
    CHECK(!FindForbidden(exp.edge_symbol()))
        << "Expand uses an already used edge symbol.";
    // Check for bounding expressions.
    if (exp.lower_bound()) {
      UsedSymbolsCollector collector(*symbol_table_);
      exp.lower_bound()->Accept(collector);
      // TODO: Figure out what do to here. Perhaps we shouldn't allow variables
      // in bound expressions.
      if (ContainsForbidden(collector.symbols_)) {
        throw utils::NotYetImplemented("distributed Cartesian planning");
      }
    }
    if (exp.upper_bound()) {
      UsedSymbolsCollector collector(*symbol_table_);
      exp.upper_bound()->Accept(collector);
      if (ContainsForbidden(collector.symbols_)) {
        throw utils::NotYetImplemented("distributed Cartesian planning");
      }
    }
    // Check for lambda expressions
    if (exp.filter_lambda().expression) {
      UsedSymbolsCollector collector(*symbol_table_);
      exp.filter_lambda().expression->Accept(collector);
      // TODO: Extract filters or have them be inlined later in the planning.
      if (ContainsForbidden(collector.symbols_)) {
        throw utils::NotYetImplemented("distributed Cartesian planning");
      }
    }
    if (exp.weight_lambda()) {
      CHECK(exp.weight_lambda()->expression)
          << "Unexpected nullptr expression in lambda";
      UsedSymbolsCollector collector(*symbol_table_);
      exp.weight_lambda()->expression->Accept(collector);
      // TODO: Figure out what to do here
      if (ContainsForbidden(collector.symbols_)) {
        throw utils::NotYetImplemented("distributed Cartesian planning");
      }
    }
    return true;
  }

  bool PostVisit(ExpandUniquenessFilter<EdgeAccessor> &op) override {
    if (subtree_) return true;
    if (auto found = FindForbidden(op.expand_symbol())) {
      subtree_ = op.input();
      parent_ = &op;
      depends_on_ = found;
    }
    if (auto found = ContainsForbidden(op.previous_symbols())) {
      subtree_ = op.input();
      parent_ = &op;
      if (depends_on_) {
        depends_on_ = std::min(*found, *depends_on_);
      } else {
        depends_on_ = found;
      }
    }
    return true;
  }

  bool PostVisit(Filter &op) override {
    if (subtree_) return true;
    UsedSymbolsCollector collector(*symbol_table_);
    op.expression()->Accept(collector);
    if (auto found = ContainsForbidden(collector.symbols_)) {
      subtree_ = op.input();
      parent_ = &op;
      depends_on_ = found;
    }
    return true;
  }

  bool PostVisit(Produce &produce) override {
    if (subtree_) return true;
    for (auto *named_expr : produce.named_expressions()) {
      UsedSymbolsCollector collector(*symbol_table_);
      named_expr->expression_->Accept(collector);
      if (auto found = ContainsForbidden(collector.symbols_)) {
        subtree_ = produce.input();
        parent_ = &produce;
        depends_on_ = found;
        break;
      }
    }
    return true;
  }

  bool PostVisit(Optional &optional) override {
    throw utils::NotYetImplemented("distributed Cartesian planning");
  }

  bool PostVisit(Unwind &unwind) override {
    if (subtree_) return true;
    UsedSymbolsCollector collector(*symbol_table_);
    unwind.input_expression()->Accept(collector);
    if (auto found = ContainsForbidden(collector.symbols_)) {
      subtree_ = unwind.input();
      parent_ = &unwind;
      depends_on_ = found;
    }
    return true;
  }

  // Remaining operators can only appear if they don't contain any forbidden
  // symbols. This is the case when we are planning *another* Cartesian after
  // Produce.

  bool PostVisit(CreateNode &op) override {
    CHECK(!FindForbidden(symbol_table_->at(*(op.node_atom()->identifier_))));
    for (auto &kv : op.node_atom()->properties_) {
      UsedSymbolsCollector collector(*symbol_table_);
      kv.second->Accept(collector);
      CHECK(!ContainsForbidden(collector.symbols_));
    }
    return true;
  }

  bool PostVisit(CreateExpand &op) override {
    CHECK(!FindForbidden(op.input_symbol()));
    CHECK(!FindForbidden(symbol_table_->at(*(op.node_atom()->identifier_))));
    CHECK(!FindForbidden(symbol_table_->at(*(op.edge_atom()->identifier_))));
    for (auto &kv : op.node_atom()->properties_) {
      UsedSymbolsCollector collector(*symbol_table_);
      kv.second->Accept(collector);
      CHECK(!ContainsForbidden(collector.symbols_));
    }
    for (auto &kv : op.edge_atom()->properties_) {
      UsedSymbolsCollector collector(*symbol_table_);
      kv.second->Accept(collector);
      CHECK(!ContainsForbidden(collector.symbols_));
    }
    return true;
  }

  bool PostVisit(Delete &op) override {
    for (auto *expr : op.expressions()) {
      UsedSymbolsCollector collector(*symbol_table_);
      expr->Accept(collector);
      CHECK(!ContainsForbidden(collector.symbols_));
    }
    return true;
  }

  bool PostVisit(SetProperty &op) override {
    UsedSymbolsCollector collector(*symbol_table_);
    op.lhs()->Accept(collector);
    op.rhs()->Accept(collector);
    CHECK(!ContainsForbidden(collector.symbols_));
    return true;
  }

  bool PostVisit(SetProperties &op) override {
    CHECK(!FindForbidden(op.input_symbol()));
    UsedSymbolsCollector collector(*symbol_table_);
    op.rhs()->Accept(collector);
    CHECK(!ContainsForbidden(collector.symbols_));
    return true;
  }

  bool PostVisit(SetLabels &op) override {
    CHECK(!FindForbidden(op.input_symbol()));
    return true;
  }

  bool PostVisit(RemoveProperty &op) override {
    UsedSymbolsCollector collector(*symbol_table_);
    op.lhs()->Accept(collector);
    CHECK(!ContainsForbidden(collector.symbols_));
    return true;
  }

  bool PostVisit(RemoveLabels &op) override {
    CHECK(!FindForbidden(op.input_symbol()));
    return true;
  }

  bool PostVisit(Aggregate &aggr) override {
    CHECK(!ContainsForbidden(aggr.remember()));
    for (auto &elem : aggr.aggregations()) {
      UsedSymbolsCollector collector(*symbol_table_);
      if (elem.value) elem.value->Accept(collector);
      if (elem.key) elem.key->Accept(collector);
      CHECK(!ContainsForbidden(collector.symbols_));
    }
    for (auto *expr : aggr.group_by()) {
      UsedSymbolsCollector collector(*symbol_table_);
      expr->Accept(collector);
      CHECK(!ContainsForbidden(collector.symbols_));
    }
    return true;
  }

  bool PostVisit(Skip &skip) override {
    UsedSymbolsCollector collector(*symbol_table_);
    skip.expression()->Accept(collector);
    CHECK(!ContainsForbidden(collector.symbols_));
    return true;
  }

  bool PostVisit(Limit &limit) override {
    UsedSymbolsCollector collector(*symbol_table_);
    limit.expression()->Accept(collector);
    CHECK(!ContainsForbidden(collector.symbols_));
    return true;
  }

  bool PostVisit(OrderBy &order_by) override {
    CHECK(!ContainsForbidden(order_by.output_symbols()));
    for (auto *expr : order_by.order_by()) {
      UsedSymbolsCollector collector(*symbol_table_);
      expr->Accept(collector);
      CHECK(!ContainsForbidden(collector.symbols_));
    }
    return true;
  }

  bool PostVisit(Distinct &distinct) override {
    CHECK(!ContainsForbidden(distinct.value_symbols()));
    return true;
  }

  bool PostVisit(Cartesian &cart) override {
    CHECK(!ContainsForbidden(cart.left_symbols()) &&
          !ContainsForbidden(cart.right_symbols()));
    return true;
  }

  // Independent subtree
  std::shared_ptr<LogicalOperator> subtree_;
  // Immediate parent of `subtree_`.
  LogicalOperator *parent_{nullptr};
  // Minimum index into forbidden_symbols_
  std::experimental::optional<int64_t> depends_on_;

 protected:
  bool DefaultPostVisit() override {
    throw utils::NotYetImplemented("distributed Cartesian planning");
  }

 private:
  std::vector<std::vector<Symbol>> forbidden_symbols_;
  const SymbolTable *symbol_table_;

  template <class TCollection>
  std::experimental::optional<int64_t> ContainsForbidden(
      const TCollection &symbols) {
    for (int64_t i = 0; i < forbidden_symbols_.size(); ++i) {
      for (const auto &symbol : symbols) {
        if (utils::Contains(forbidden_symbols_[i], symbol)) {
          return std::experimental::make_optional(i);
        }
      }
    }
    return std::experimental::nullopt;
  }

  std::experimental::optional<int64_t> FindForbidden(const Symbol &symbol) {
    for (int64_t i = 0; i < forbidden_symbols_.size(); ++i) {
      if (utils::Contains(forbidden_symbols_[i], symbol)) {
        return std::experimental::make_optional(i);
      }
    }
    return std::experimental::nullopt;
  }
};

// Find the subtree parent, below which no operator uses symbols found in the
// `forbidden_symbols` set.
//
// A tuple is returned (subtree, parent, depends_on), where the parent may be
// nullptr if the given `op` is already a subtree which doesn't use
// `forbidden_symbols`. `depends_on` is set to the minimum index of the
// `forbidden_symbols` that the operators above the `subtree` depend on. The
// returned `parent` is therefore the last operator which depends on
// `forbidden_symbols`.
auto FindIndependentSubtree(
    const std::shared_ptr<LogicalOperator> &op,
    const std::vector<std::vector<Symbol>> &forbidden_symbols,
    const SymbolTable *symbol_table) {
  IndependentSubtreeFinder finder(forbidden_symbols, symbol_table);
  op->Accept(finder);
  auto subtree = finder.subtree_ ? finder.subtree_ : op;
  return std::make_tuple(subtree, finder.parent_, finder.depends_on_);
}

class DistributedPlanner : public HierarchicalLogicalOperatorVisitor {
 public:
  DistributedPlanner(DistributedPlan &distributed_plan,
                     std::atomic<int64_t> &next_plan_id)
      : distributed_plan_(distributed_plan), next_plan_id_(next_plan_id) {}

  using HierarchicalLogicalOperatorVisitor::PostVisit;
  using HierarchicalLogicalOperatorVisitor::PreVisit;

  // ScanAll are all done on each machine locally.
  // We need special care when multiple ScanAll operators appear, this means we
  // need a Cartesian product. Both the left and the right side of Cartesian
  // should be sent as standalone plans to each worker. Then, the master
  // execution should use PullRemote to wire them into Cartesian.  In case of
  // multiple Cartesians, we send each ScanAll part to workers and chain them
  // into multiple Cartesians on master.
  //
  // For example, `ScanAll(n) > ScanAll(m) > ScanAll(l)` is transformed to:
  //
  //    workers       |   master
  //
  //  * ScanAll(n) - - - - - - - - - \
  //                                  + PullRemote(n) \
  //  * ScanAll(m) \                                   + Cartesian
  //                +- PullRemote(m) \                /
  //                                  + Cartesian - -
  //                +- PullRemote(l) /
  //  * ScanAll(l) /
  //
  // Things get more complicated if any branch of the Cartesian has a Filter
  // operator which depends on the result from another branch.
  //
  // For example, `ScanAll (n) > ScanAll (m) > Filter (m.prop = n.prop)`:
  //
  //              workers                         |    master
  //
  //  * ScanAll(n) - - - - - - - - - - - - - -  \
  //                                             + PullRemote(n) \
  //                                                              + Cartesian
  //                                             + PullRemote(m) /
  //  * ScanAll(m) - - Filter (m.prop = n.prop) /
  //
  // Since the Filter depends on the first ScanAll branch, we can either:
  //  * enforce the first branch is evaluated before and data sent back to
  //    workers to evaluate the second; or
  //  * move the Filter after the Cartesian (or maybe inline it inside).
  //
  // This implementation goes with the 2nd approach, by tracking dependent
  // branches and putting them after Cartesian. We should probably redesign the
  // planning so that cost estimation can estimate Cartesian products and
  // influence the distributed planner. Therefore, the above example would turn
  // into:
  //
  //    workers        |    master
  //
  //  * ScanAll(n) - - \
  //                    + PullRemote(n) \
  //                                     + Cartesian - Filter (m.prop = n.prop)
  //                    + PullRemote(m) /
  //  * ScanAll(m) - - /
  //
  // Inserting the Cartesian operator is done through PlanCartesian while
  // post-visiting Produce, Aggregate or write operators.
  //
  // TODO: Finish Cartesian planning:
  //  * splitting indexed ScanAll;
  //  * allowing Cartesian after Produce (i.e. after WITH clause);
  //  * ...
  auto PlanCartesian(const std::shared_ptr<LogicalOperator> &rhs_input) {
    std::shared_ptr<LogicalOperator> cartesian;
    auto right_branch = MakeCartesianBranch(rhs_input);
    auto right_op = right_branch.subtree;
    std::vector<CartesianBranch> dependent_branches;
    dependent_branches.reserve(cartesian_branches_.size() + 1);
    if (right_branch.parent_start) {
      dependent_branches.push_back(right_branch);
    }
    auto append_dependants = [&dependent_branches](auto op, int64_t branch_ix) {
      // Append dependent parents, iteration is in reverse because we want
      // LIFO behaviour.
      for (auto it = dependent_branches.rbegin();
           it != dependent_branches.rend(); ++it) {
        if (it->depends_on.value() != branch_ix) continue;
        it->parent_end->set_input(op);
        op = it->parent_start;
      }
      dependent_branches.erase(
          std::remove_if(dependent_branches.begin(), dependent_branches.end(),
                         [branch_ix](const auto &branch) {
                           return branch.depends_on.value() == branch_ix;
                         }),
          dependent_branches.end());
      return op;
    };
    // We use this ordering of operators, so that left hand side can be
    // accumulated without having whole product accumulations. This relies on
    // the fact that Cartesian accumulation strategy accumulates the left
    // operator input.
    while (!cartesian_branches_.empty()) {
      auto left_branch = cartesian_branches_.back();
      cartesian_branches_.pop_back();
      if (left_branch.parent_start) {
        dependent_branches.push_back(left_branch);
      }
      cartesian = std::make_shared<Cartesian>(
          left_branch.subtree,
          left_branch.subtree->ModifiedSymbols(distributed_plan_.symbol_table),
          right_op, right_op->ModifiedSymbols(distributed_plan_.symbol_table));
      cartesian = append_dependants(cartesian, cartesian_branches_.size());
      right_op = cartesian;
    }
    CHECK(dependent_branches.empty())
        << "Expected to fill all Cartesian branches.";
    cartesian_symbols_.clear();
    return cartesian;
  }

  void AddForCartesian(ScanAll *scan) {
    cartesian_branches_.emplace_back(MakeCartesianBranch(scan->input()));
    // Collect modified symbols of the whole branch (independent subtree +
    // parent subtree).
    cartesian_symbols_.emplace_back(
        scan->input()->ModifiedSymbols(distributed_plan_.symbol_table));
    // Rewire the scan to be cut from the branch.
    scan->set_input(std::make_shared<Once>());
  }

  bool PreVisit(ScanAll &scan) override {
    prev_ops_.push_back(&scan);
    return true;
  }
  bool PostVisit(ScanAll &scan) override {
    prev_ops_.pop_back();
    if (has_scan_all_) {
      AddForCartesian(&scan);
    }
    has_scan_all_ = true;
    return true;
  }

  bool PreVisit(ScanAllByLabel &scan) override {
    prev_ops_.push_back(&scan);
    return true;
  }
  bool PostVisit(ScanAllByLabel &scan) override {
    prev_ops_.pop_back();
    if (has_scan_all_) {
      AddForCartesian(&scan);
    }
    has_scan_all_ = true;
    return true;
  }
  bool PreVisit(ScanAllByLabelPropertyRange &scan) override {
    prev_ops_.push_back(&scan);
    return true;
  }
  bool PostVisit(ScanAllByLabelPropertyRange &scan) override {
    prev_ops_.pop_back();
    if (has_scan_all_) {
      AddForCartesian(&scan);
    }
    has_scan_all_ = true;
    return true;
  }
  bool PreVisit(ScanAllByLabelPropertyValue &scan) override {
    prev_ops_.push_back(&scan);
    return true;
  }
  bool PostVisit(ScanAllByLabelPropertyValue &scan) override {
    prev_ops_.pop_back();
    if (has_scan_all_) {
      AddForCartesian(&scan);
    }
    has_scan_all_ = true;
    return true;
  }

  // Expand is done locally on each machine with RPC calls for worker-boundary
  // crossing edges.
  bool PreVisit(Expand &exp) override {
    prev_ops_.push_back(&exp);
    return true;
  }

  bool PreVisit(ExpandVariable &exp) override {
    prev_ops_.push_back(&exp);
    return true;
  }

  // The following operators filter the frame or put something on it. They
  // should be worker local.
  bool PreVisit(ConstructNamedPath &op) override {
    prev_ops_.push_back(&op);
    return true;
  }
  bool PreVisit(Filter &op) override {
    prev_ops_.push_back(&op);
    return true;
  }
  bool PreVisit(ExpandUniquenessFilter<VertexAccessor> &op) override {
    prev_ops_.push_back(&op);
    return true;
  }
  bool PreVisit(ExpandUniquenessFilter<EdgeAccessor> &op) override {
    prev_ops_.push_back(&op);
    return true;
  }
  bool PreVisit(Optional &op) override {
    prev_ops_.push_back(&op);
    return true;
  }

  // Skip needs to skip only the first N results from *all* of the results.
  // Therefore, the earliest (deepest in the plan tree) encountered Skip will
  // break the plan in 2 parts.
  //  1) Master plan with Skip and everything above it.
  //  2) Worker plan with operators below Skip, but without Skip itself.
  bool PreVisit(Skip &skip) override {
    prev_ops_.push_back(&skip);
    return true;
  }
  bool PostVisit(Skip &skip) override {
    prev_ops_.pop_back();
    return true;
  }

  // Limit, like Skip, needs to see *all* of the results, so we split the plan.
  // Unlike Skip, we can also do the operator locally on each machine. This may
  // improve the execution speed of workers. So, the 2 parts of the plan are:
  //  1) Master plan with Limit and everything above.
  //  2) Worker plan with operators below Limit, but including Limit itself.
  bool PreVisit(Limit &limit) override {
    prev_ops_.push_back(&limit);
    return true;
  }
  bool PostVisit(Limit &limit) override {
    prev_ops_.pop_back();
    return true;
  }

  // OrderBy is an associative operator, this means we can do ordering
  // on workers and then merge the results on master.
  bool PreVisit(OrderBy &order_by) override {
    prev_ops_.push_back(&order_by);
    return true;
  }
  bool PostVisit(OrderBy &order_by) override {
    prev_ops_.pop_back();
    return true;
  }

  // Treat Distinct just like Limit.
  bool PreVisit(Distinct &distinct) override {
    prev_ops_.push_back(&distinct);
    return true;
  }
  bool PostVisit(Distinct &distinct) override {
    prev_ops_.pop_back();
    return true;
  }

  // TODO: Union

  // For purposes of distribution, aggregation comes in 2 flavors:
  //  * associative and
  //  * non-associative.
  //
  // Associative aggregation can be done locally on workers, and then the
  // results merged on master. Similarly to how OrderBy can be distributed. For
  // this type of aggregation, master will need to have an aggregation merging
  // operator. This need not be a new LogicalOperator, it can be a new
  // Aggregation with different Expressions.
  //
  // Non-associative aggregation needs to see all of the results and is
  // completely done on master.
  bool PreVisit(Aggregate &aggr_op) override {
    prev_ops_.push_back(&aggr_op);
    return true;
  }
  bool PostVisit(Aggregate &aggr_op) override {
    prev_ops_.pop_back();
    if (!cartesian_branches_.empty()) {
      Split(aggr_op, PlanCartesian(aggr_op.input()));
      return true;
    }
    return true;
  }

  bool PreVisit(Produce &produce) override {
    prev_ops_.push_back(&produce);
    return true;
  }
  bool PostVisit(Produce &produce) override {
    prev_ops_.pop_back();
    if (!cartesian_branches_.empty()) {
      // TODO: It might be better to plan Cartesians later if this Produce isn't
      // the last one and is not followed by an operator which requires a merge
      // point (Skip, OrderBy, etc.).
      Split(produce, PlanCartesian(produce.input()));
    }
    return true;
  }

  bool PreVisit(Unwind &op) override {
    prev_ops_.push_back(&op);
    return true;
  }

  bool Visit(Once &) override { return true; }

  bool Visit(CreateIndex &) override { return true; }

  // Accumulate is used only if the query performs any writes. In such a case,
  // we need to synchronize the work done on master and all workers.
  // Synchronization will force applying changes to distributed storage, and
  // then we can continue with the rest of the plan. Currently, the remainder of
  // the plan is executed on master. In the future, when we support Cartesian
  // products after the WITH clause, we will need to split the plan in more
  // subparts to be executed on workers.
  bool PreVisit(Accumulate &acc) override {
    prev_ops_.push_back(&acc);
    return true;
  }
  bool PostVisit(Accumulate &acc) override {
    prev_ops_.pop_back();
    on_master_ = true;
    return true;
  }

  // CRUD operators follow

  bool PreVisit(CreateNode &op) override {
    prev_ops_.push_back(&op);
    return true;
  }
  bool PostVisit(CreateNode &op) override {
    prev_ops_.pop_back();
    if (!cartesian_branches_.empty()) {
      Split(op, PlanCartesian(op.input()));
    }
    return true;
  }

  bool PreVisit(CreateExpand &op) override {
    prev_ops_.push_back(&op);
    return true;
  }
  bool PostVisit(CreateExpand &op) override {
    prev_ops_.pop_back();
    if (!cartesian_branches_.empty()) {
      Split(op, PlanCartesian(op.input()));
    }
    return true;
  }

  bool PreVisit(Delete &op) override {
    prev_ops_.push_back(&op);
    return true;
  }
  bool PostVisit(Delete &op) override {
    prev_ops_.pop_back();
    if (!cartesian_branches_.empty()) {
      Split(op, PlanCartesian(op.input()));
    }
    return true;
  }

  bool PreVisit(SetProperty &op) override {
    prev_ops_.push_back(&op);
    return true;
  }
  bool PostVisit(SetProperty &op) override {
    prev_ops_.pop_back();
    if (!cartesian_branches_.empty()) {
      Split(op, PlanCartesian(op.input()));
    }
    return true;
  }

  bool PreVisit(SetProperties &op) override {
    prev_ops_.push_back(&op);
    return true;
  }
  bool PostVisit(SetProperties &op) override {
    prev_ops_.pop_back();
    if (!cartesian_branches_.empty()) {
      Split(op, PlanCartesian(op.input()));
    }
    return true;
  }

  bool PreVisit(SetLabels &op) override {
    prev_ops_.push_back(&op);
    return true;
  }
  bool PostVisit(SetLabels &op) override {
    prev_ops_.pop_back();
    if (!cartesian_branches_.empty()) {
      Split(op, PlanCartesian(op.input()));
    }
    return true;
  }

  bool PreVisit(RemoveProperty &op) override {
    prev_ops_.push_back(&op);
    return true;
  }
  bool PostVisit(RemoveProperty &op) override {
    prev_ops_.pop_back();
    if (!cartesian_branches_.empty()) {
      Split(op, PlanCartesian(op.input()));
    }
    return true;
  }

  bool PreVisit(RemoveLabels &op) override {
    prev_ops_.push_back(&op);
    return true;
  }
  bool PostVisit(RemoveLabels &op) override {
    prev_ops_.pop_back();
    if (!cartesian_branches_.empty()) {
      Split(op, PlanCartesian(op.input()));
    }
    return true;
  }

 protected:
  bool DefaultPreVisit() override {
    throw utils::NotYetImplemented("distributed planning");
  }

  bool DefaultPostVisit() override {
    prev_ops_.pop_back();
    return true;
  }

 private:
  struct CartesianBranch {
    // Independent subtree that can become a Cartesian branch
    std::shared_ptr<LogicalOperator> subtree;
    // Optional parent of the subtree which needs to come after the Cartesian
    // operator, because it depends on some other Cartesian branch.
    // parent_start is shared_ptr, because the ownership will be transfered to
    // some other operator.
    std::shared_ptr<LogicalOperator> parent_start;
    // parent_end is pointer, because we may only change its input.
    LogicalOperator *parent_end{nullptr};
    // Minimum index of the branch this parent depends on.
    std::experimental::optional<int64_t> depends_on;
  };

  CartesianBranch MakeCartesianBranch(
      const std::shared_ptr<LogicalOperator> &input) {
    CartesianBranch branch;
    std::tie(branch.subtree, branch.parent_end, branch.depends_on) =
        FindIndependentSubtree(input, cartesian_symbols_,
                               &distributed_plan_.symbol_table);
    if (branch.parent_end) {
      // This branch depends on another, so we need store the parent subtree
      // which contains that dependency.
      branch.parent_start = input;
    }
    if (on_master_ && cartesian_branches_.empty()) {
      // Since we are planning a new Cartesian, the first CartesianBranch must
      // not be sent to workers if we are executing on master.
      return branch;
    }
    return branch;
  }

  DistributedPlan &distributed_plan_;
  std::atomic<int64_t> &next_plan_id_;
  std::vector<LogicalOperator *> prev_ops_;
  // Operators that still need to be wired into Cartesian.
  std::vector<CartesianBranch> cartesian_branches_;
  // Symbols modified by the currently stored Cartesian branches. Each vector
  // corresponds to the above CartesianBranch.
  std::vector<std::vector<Symbol>> cartesian_symbols_;
  bool has_scan_all_ = false;
  // True if we have added a worker merge point on master, i.e. the rest of the
  // plan is executing on master.
  bool on_master_ = false;

  // Sets the master_op input to be merge_op. Afterwards, on_master_ is true.
  template <class TOp>
  void Split(TOp &master_op, std::shared_ptr<LogicalOperator> merge_op) {
    on_master_ = true;
    master_op.set_input(merge_op);
  }

  void SplitOnPrevious(std::unique_ptr<LogicalOperator> merge_op) {
    if (on_master_) throw utils::NotYetImplemented("distributed planning");
    on_master_ = true;
    if (prev_ops_.empty()) {
      distributed_plan_.master_plan = std::move(merge_op);
      return;
    }
    SetOnPrevious(std::move(merge_op));
  }

  void SetOnPrevious(std::unique_ptr<LogicalOperator> input_op) {
    auto *prev_op = prev_ops_.back();
    DCHECK(prev_op)
        << "SetOnPrevious should only be called when there is a previously "
           "visited operation";
    if (!prev_op->HasSingleInput())
      throw utils::NotYetImplemented("distributed planning");
    prev_op->set_input(std::move(input_op));
  }

  int64_t AddWorkerPlan(const std::shared_ptr<LogicalOperator> &worker_plan) {
    return ::query::plan::AddWorkerPlan(distributed_plan_, next_plan_id_,
                                        worker_plan);
  }
};

}  // namespace

}  // namespace query::plan

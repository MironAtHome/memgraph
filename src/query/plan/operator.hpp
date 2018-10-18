/** @file */

#pragma once

#include <experimental/optional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "query/common.hpp"
#include "query/frontend/ast/ast.hpp"
#include "query/frontend/semantic/symbol.hpp"
#include "query/typed_value.hpp"
#include "storage/types.hpp"
#include "utils/bound.hpp"
#include "utils/future.hpp"
#include "utils/hashing/fnv.hpp"
#include "utils/visitor.hpp"

namespace database {
class GraphDbAccessor;
}

namespace query {

class Context;
class ExpressionEvaluator;
class Frame;
class SymbolTable;

namespace plan {

/** @brief Base class for iteration cursors of @c LogicalOperator classes.
 *
 *  Each @c LogicalOperator must produce a concrete @c Cursor, which provides
 *  the iteration mechanism.
 */
class Cursor {
 public:
  /** @brief Run an iteration of a @c LogicalOperator.
   *
   *  Since operators may be chained, the iteration may pull results from
   *  multiple operators.
   *
   *  @param Frame May be read from or written to while performing the
   *      iteration.
   *  @param Context Used to get the position of symbols in frame and other
   * information.
   */
  virtual bool Pull(Frame &, Context &) = 0;

  /**
   * Resets the Cursor to it's initial state.
   */
  virtual void Reset() = 0;

  virtual ~Cursor() {}
};

class Once;
class CreateNode;
class CreateExpand;
class ScanAll;
class ScanAllByLabel;
class ScanAllByLabelPropertyRange;
class ScanAllByLabelPropertyValue;
class Expand;
class ExpandVariable;
class ConstructNamedPath;
class Filter;
class Produce;
class Delete;
class SetProperty;
class SetProperties;
class SetLabels;
class RemoveProperty;
class RemoveLabels;
template <typename TAccessor>
class ExpandUniquenessFilter;
class Accumulate;
class Aggregate;
class Skip;
class Limit;
class OrderBy;
class Merge;
class Optional;
class Unwind;
class Distinct;
class CreateIndex;
class Union;
class Cartesian;

using LogicalOperatorCompositeVisitor = ::utils::CompositeVisitor<
    Once, CreateNode, CreateExpand, ScanAll, ScanAllByLabel,
    ScanAllByLabelPropertyRange, ScanAllByLabelPropertyValue, Expand,
    ExpandVariable, ConstructNamedPath, Filter, Produce, Delete, SetProperty,
    SetProperties, SetLabels, RemoveProperty, RemoveLabels,
    ExpandUniquenessFilter<VertexAccessor>,
    ExpandUniquenessFilter<EdgeAccessor>, Accumulate, Aggregate, Skip, Limit,
    OrderBy, Merge, Optional, Unwind, Distinct, Union, Cartesian>;

using LogicalOperatorLeafVisitor = ::utils::LeafVisitor<Once, CreateIndex>;

/**
 * @brief Base class for hierarhical visitors of @c LogicalOperator class
 * hierarchy.
 */
class HierarchicalLogicalOperatorVisitor
    : public LogicalOperatorCompositeVisitor,
      public LogicalOperatorLeafVisitor {
 public:
  using LogicalOperatorCompositeVisitor::PostVisit;
  using LogicalOperatorCompositeVisitor::PreVisit;
  using LogicalOperatorLeafVisitor::Visit;
  using typename LogicalOperatorLeafVisitor::ReturnType;
};

/** Base class for logical operators.
 *
 *  Each operator describes an operation, which is to be performed on the
 *  database. Operators are iterated over using a @c Cursor. Various operators
 *  can serve as inputs to others and thus a sequence of operations is formed.
 */
class LogicalOperator
    : public ::utils::Visitable<HierarchicalLogicalOperatorVisitor> {
 public:
  virtual ~LogicalOperator() {}

  /** Constructs a @c Cursor which is used to run this operator.
   *
   *  @param database::GraphDbAccessor Used to perform operations on the
   * database.
   */
  virtual std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &) const = 0;

  /** Return @c Symbol vector where the query results will be stored.
   *
   * Currently, outputs symbols are generated in @c Produce and @c Union
   * operators. @c Skip, @c Limit, @c OrderBy and @c Distinct propagate the
   * symbols from @c Produce (if it exists as input operator).
   *
   *  @param SymbolTable used to find symbols for expressions.
   *  @return std::vector<Symbol> used for results.
   */
  virtual std::vector<Symbol> OutputSymbols(const SymbolTable &) const {
    return std::vector<Symbol>();
  }

  /**
   * Symbol vector whose values are modified by this operator sub-tree.
   *
   * This is different than @c OutputSymbols, because it returns all of the
   * modified symbols, including those that may not be returned as the
   * result of the query. Note that the modified symbols will not contain
   * those that should not be read after the operator is processed.
   *
   * For example, `MATCH (n)-[e]-(m) RETURN n AS l` will generate `ScanAll (n) >
   * Expand (e, m) > Produce (l)`. The modified symbols on Produce sub-tree will
   * be `l`, the same as output symbols, because it isn't valid to read `n`, `e`
   * nor `m` after Produce. On the other hand, modified symbols from Expand
   * contain `e` and `m`, as well as `n`, while output symbols are empty.
   * Modified symbols from ScanAll contain only `n`, while output symbols are
   * also empty.
   */
  virtual std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const = 0;

  /**
   * Returns true if the operator takes only one input operator.
   * NOTE: When this method returns true, you may use `input` and `set_input`
   * methods.
   */
  virtual bool HasSingleInput() const = 0;

  /**
   * Returns the input operator if it has any.
   * NOTE: This should only be called if `HasSingleInput() == true`.
   */
  virtual std::shared_ptr<LogicalOperator> input() const = 0;
  /**
   * Set a different input on this operator.
   * NOTE: This should only be called if `HasSingleInput() == true`.
   */
  virtual void set_input(std::shared_ptr<LogicalOperator>) = 0;
};

template <class TArchive>
std::pair<std::unique_ptr<LogicalOperator>, AstTreeStorage> LoadPlan(
    TArchive &ar) {
  std::unique_ptr<LogicalOperator> root;
  ar >> root;
  return {std::move(root), std::move(ar.template get_helper<AstTreeStorage>(
                               AstTreeStorage::kHelperId))};
}

/**
 * A logical operator whose Cursor returns true on the first Pull
 * and false on every following Pull.
 */
class Once : public LogicalOperator {
 public:
  DEFVISITABLE(HierarchicalLogicalOperatorVisitor);
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override {
    return {};
  }

  bool HasSingleInput() const override;
  std::shared_ptr<LogicalOperator> input() const override;
  void set_input(std::shared_ptr<LogicalOperator>) override;

 private:
  class OnceCursor : public Cursor {
   public:
    OnceCursor() {}
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    bool did_pull_{false};
  };
};

/** @brief Operator for creating a node.
 *
 *  This op is used both for creating a single node (`CREATE` statement without
 *  a preceeding `MATCH`), or multiple nodes (`MATCH ... CREATE` or
 *  `CREATE (), () ...`).
 *
 *  @sa CreateExpand
 */
class CreateNode : public LogicalOperator {
 public:
  /**
   * @param input Optional. If @c nullptr, then a single node will be
   *    created (a single successful @c Cursor::Pull from this op's @c Cursor).
   *    If a valid input, then a node will be created for each
   *    successful pull from the given input.
   * @param node_atom @c NodeAtom with information on how to create a node.
   * @param on_random_worker If the node should be created locally or on random
   * worker.
   */
  CreateNode(const std::shared_ptr<LogicalOperator> &input, NodeAtom *node_atom,
             bool on_random_worker);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

  auto on_random_worker() const { return on_random_worker_; }
  void set_on_random_worker(bool v) { on_random_worker_ = v; }

 private:
  CreateNode() {}

  std::shared_ptr<LogicalOperator> input_;
  NodeAtom *node_atom_ = nullptr;
  bool on_random_worker_ = false;

  class CreateNodeCursor : public Cursor {
   public:
    CreateNodeCursor(const CreateNode &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const CreateNode &self_;
    const std::unique_ptr<Cursor> input_cursor_;
  };
};

/** @brief Operator for creating edges and destination nodes.
 *
 *  This operator extends already created nodes with an edge. If the other node
 *  on the edge does not exist, it will be created. For example, in `MATCH (n)
 *  CREATE (n) -[r:r]-> (n)` query, this operator will create just the edge `r`.
 *  In `MATCH (n) CREATE (n) -[r:r]-> (m)` query, the operator will create both
 *  the edge `r` and the node `m`. In case of `CREATE (n) -[r:r]-> (m)` the
 *  first node `n` is created by @c CreateNode operator, while @c CreateExpand
 *  will create the edge `r` and `m`. Similarly, multiple @c CreateExpand are
 *  chained in cases when longer paths need creating.
 *
 *  @sa CreateNode
 */
class CreateExpand : public LogicalOperator {
 public:
  /** @brief Construct @c CreateExpand.
   *
   * @param node_atom @c NodeAtom at the end of the edge. Used to create a node,
   *     unless it refers to an existing one.
   * @param edge_atom @c EdgeAtom with information for the edge to be created.
   * @param input Optional. Previous @c LogicalOperator which will be pulled.
   *     For each successful @c Cursor::Pull, this operator will create an
   *     expansion.
   * @param input_symbol @c Symbol for the node at the start of the edge.
   * @param existing_node @c bool indicating whether the @c node_atom refers to
   *     an existing node. If @c false, the operator will also create the node.
   */
  CreateExpand(NodeAtom *node_atom, EdgeAtom *edge_atom,
               const std::shared_ptr<LogicalOperator> &input,
               Symbol input_symbol, bool existing_node);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

 private:
  // info on what's getting expanded
  NodeAtom *node_atom_;
  EdgeAtom *edge_atom_;

  // the input op and the symbol under which the op's result
  // can be found in the frame
  std::shared_ptr<LogicalOperator> input_;
  Symbol input_symbol_;

  // if the given node atom refers to an existing node
  // (either matched or created)
  bool existing_node_;

  CreateExpand() {}

  class CreateExpandCursor : public Cursor {
   public:
    CreateExpandCursor(const CreateExpand &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const CreateExpand &self_;
    database::GraphDbAccessor &db_;
    const std::unique_ptr<Cursor> input_cursor_;

    /** Gets the existing node (if existing_node_ == true), or creates a new
     * node (on the given worker) and returns it. */
    VertexAccessor &OtherVertex(int worker_id, Frame &frame, Context &context);

    /**
     * Helper function for creating an edge and adding it
     * to the frame.
     *
     * @param from  Origin vertex of the edge.
     * @param to  Destination vertex of the edge.
     * @param evaluator Expression evaluator for property value eval.
     */
    void CreateEdge(VertexAccessor &from, VertexAccessor &to, Frame &frame,
                    const SymbolTable &symbol_table,
                    ExpressionEvaluator &evaluator);
  };
};

/**
 * @brief Operator which iterates over all the nodes currently in the database.
 * When given an input (optional), does a cartesian product.
 *
 * It accepts an optional input. If provided then this op scans all the nodes
 * currently in the database for each successful Pull from it's input, thereby
 * producing a cartesian product of input Pulls and database elements.
 *
 * ScanAll can either iterate over the previous graph state (state before
 * the current transacton+command) or over current state. This is controlled
 * with a constructor argument.
 *
 * @sa ScanAllByLabel
 * @sa ScanAllByLabelPropertyRange
 * @sa ScanAllByLabelPropertyValue
 */
class ScanAll : public LogicalOperator {
 public:
  ScanAll(const std::shared_ptr<LogicalOperator> &input, Symbol output_symbol,
          GraphView graph_view = GraphView::OLD);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

  auto output_symbol() const { return output_symbol_; }
  auto graph_view() const { return graph_view_; }

 protected:
  std::shared_ptr<LogicalOperator> input_;
  Symbol output_symbol_;
  /**
   * @brief Controls which graph state is used to produce vertices.
   *
   * If @c GraphView::OLD, @c ScanAll will produce vertices visible in the
   * previous graph state, before modifications done by current transaction &
   * command. With @c GraphView::NEW, all vertices will be produced the current
   * transaction sees along with their modifications.
   */
  GraphView graph_view_;

  ScanAll() {}
};

/**
 * @brief Behaves like @c ScanAll, but this operator produces only vertices with
 * given label.
 *
 * @sa ScanAll
 * @sa ScanAllByLabelPropertyRange
 * @sa ScanAllByLabelPropertyValue
 */
class ScanAllByLabel : public ScanAll {
 public:
  ScanAllByLabel(const std::shared_ptr<LogicalOperator> &input,
                 Symbol output_symbol, storage::Label label,
                 GraphView graph_view = GraphView::OLD);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;

  storage::Label label() const { return label_; }

 private:
  storage::Label label_;

  ScanAllByLabel() {}
};

/**
 * Behaves like @c ScanAll, but produces only vertices with given label and
 * property value which is inside a range (inclusive or exlusive).
 *
 * @sa ScanAll
 * @sa ScanAllByLabel
 * @sa ScanAllByLabelPropertyValue
 */
class ScanAllByLabelPropertyRange : public ScanAll {
 public:
  /** Bound with expression which when evaluated produces the bound value. */
  using Bound = utils::Bound<Expression *>;
  /**
   * Constructs the operator for given label and property value in range
   * (inclusive).
   *
   * Range bounds are optional, but only one bound can be left out.
   *
   * @param input Preceding operator which will serve as the input.
   * @param output_symbol Symbol where the vertices will be stored.
   * @param label Label which the vertex must have.
   * @param property Property from which the value will be looked up from.
   * @param lower_bound Optional lower @c Bound.
   * @param upper_bound Optional upper @c Bound.
   * @param graph_view GraphView used when obtaining vertices.
   */
  ScanAllByLabelPropertyRange(const std::shared_ptr<LogicalOperator> &input,
                              Symbol output_symbol, storage::Label label,
                              storage::Property property,
                              std::experimental::optional<Bound> lower_bound,
                              std::experimental::optional<Bound> upper_bound,
                              GraphView graph_view = GraphView::OLD);

  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;

  auto label() const { return label_; }
  auto property() const { return property_; }
  auto lower_bound() const { return lower_bound_; }
  auto upper_bound() const { return upper_bound_; }

 private:
  storage::Label label_;
  storage::Property property_;
  std::experimental::optional<Bound> lower_bound_;
  std::experimental::optional<Bound> upper_bound_;

  ScanAllByLabelPropertyRange() {}
};

/**
 * Behaves like @c ScanAll, but produces only vertices with given label and
 * property value.
 *
 * @sa ScanAll
 * @sa ScanAllByLabel
 * @sa ScanAllByLabelPropertyRange
 */
class ScanAllByLabelPropertyValue : public ScanAll {
 public:
  /**
   * Constructs the operator for given label and property value.
   *
   * @param input Preceding operator which will serve as the input.
   * @param output_symbol Symbol where the vertices will be stored.
   * @param label Label which the vertex must have.
   * @param property Property from which the value will be looked up from.
   * @param expression Expression producing the value of the vertex property.
   * @param graph_view GraphView used when obtaining vertices.
   */
  ScanAllByLabelPropertyValue(const std::shared_ptr<LogicalOperator> &input,
                              Symbol output_symbol, storage::Label label,
                              storage::Property property,
                              Expression *expression,
                              GraphView graph_view = GraphView::OLD);

  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;

  auto label() const { return label_; }
  auto property() const { return property_; }
  auto expression() const { return expression_; }

 private:
  storage::Label label_;
  storage::Property property_;
  Expression *expression_;

  ScanAllByLabelPropertyValue() {}
};

/**
 * Common functionality and data members of single-edge
 * and variable-length expansion
 */
class ExpandCommon {
 protected:
  // types that we'll use for members in both subclasses
  using InEdgeT = decltype(std::declval<VertexAccessor>().in());
  using InEdgeIteratorT = decltype(std::declval<VertexAccessor>().in().begin());
  using OutEdgeT = decltype(std::declval<VertexAccessor>().out());
  using OutEdgeIteratorT =
      decltype(std::declval<VertexAccessor>().out().begin());

 public:
  /**
   * Initializes common expansion parameters.
   *
   * Edge/Node existence are controlled with booleans. 'true'
   * denotes that this expansion references an already
   * Pulled node/edge, and should only be checked for equality
   * during expansion.
   *
   * Expansion can be done from old or new state of the vertex
   * the expansion originates from. This is controlled with a
   * constructor argument.
   *
   * @param node_symbol Symbol pointing to the node to be expanded. This is
   *    where the new node will be stored.
   * @param edge_symbol Symbol for the edges to be expanded. This is where
   *    a TypedValue containing a list of expanded edges will be stored.
   * @param direction EdgeAtom::Direction determining the direction of edge
   *    expansion. The direction is relative to the starting vertex for each
   *    expansion.
   * @param edge_types storage::EdgeType specifying which edges we
   *    want to expand. If empty, all edges are valid. If not empty, only edges
   * with one of the given types are valid.
   * @param input Optional LogicalOperator that preceeds this one.
   * @param input_symbol Symbol that points to a VertexAccessor in the Frame
   *    that expansion should emanate from.
   * @param existing_node If or not the node to be expanded is already present
   *    in the Frame and should just be checked for equality.
   */
  ExpandCommon(Symbol node_symbol, Symbol edge_symbol,
               EdgeAtom::Direction direction,
               const std::vector<storage::EdgeType> &edge_types,
               const std::shared_ptr<LogicalOperator> &input,
               Symbol input_symbol, bool existing_node,
               GraphView graph_view = GraphView::AS_IS);

  const auto &input_symbol() const { return input_symbol_; }
  const auto &node_symbol() const { return node_symbol_; }
  const auto &edge_symbol() const { return edge_symbol_; }
  const auto &direction() const { return direction_; }
  const auto &edge_types() const { return edge_types_; }

 protected:
  // info on what's getting expanded
  Symbol node_symbol_;
  Symbol edge_symbol_;
  EdgeAtom::Direction direction_;
  std::vector<storage::EdgeType> edge_types_;

  // the input op and the symbol under which the op's result
  // can be found in the frame
  std::shared_ptr<LogicalOperator> input_;
  Symbol input_symbol_;

  // If the given node atom refer to a symbol that has already been expanded and
  // should be just validated in the frame.
  bool existing_node_;

  // from which state the input node should get expanded
  GraphView graph_view_;

  /**
   * For a newly expanded node handles existence checking and
   * frame placement.
   *
   * @return If or not the given new_node is a valid expansion. It is not
   * valid only when matching and existing node and new_node does not match
   * the old.
   */
  bool HandleExistingNode(const VertexAccessor &new_node, Frame &frame) const;

  ExpandCommon() {}
};

/**
 * @brief Expansion operator. For a node existing in the frame it
 * expands one edge and one node and places them on the frame.
 *
 * This class does not handle node/edge filtering based on
 * properties, labels and edge types. However, it does handle
 * filtering on existing node / edge.
 *
 * Filtering on existing means that for a pattern that references
 * an already declared node or edge (for example in
 * MATCH (a) MATCH (a)--(b)),
 * only expansions that match defined equalities are successfully
 * pulled.
 */
class Expand : public LogicalOperator, public ExpandCommon {
 public:
  /**
   * Creates an expansion. All parameters are forwarded to @c ExpandCommon and
   * are documented there.
   */
  using ExpandCommon::ExpandCommon;

  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

  class ExpandCursor : public Cursor {
   public:
    ExpandCursor(const Expand &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    struct FutureExpand {
      utils::Future<std::pair<EdgeAccessor, VertexAccessor>> edge_to;
      std::vector<TypedValue> frame_elems;
    };

    const Expand &self_;
    const std::unique_ptr<Cursor> input_cursor_;
    database::GraphDbAccessor &db_;

    // The iterable over edges and the current edge iterator are referenced via
    // optional because they can not be initialized in the constructor of
    // this class. They are initialized once for each pull from the input.
    std::experimental::optional<InEdgeT> in_edges_;
    std::experimental::optional<InEdgeIteratorT> in_edges_it_;
    std::experimental::optional<OutEdgeT> out_edges_;
    std::experimental::optional<OutEdgeIteratorT> out_edges_it_;
    // Stores the last frame before we yield the frame for future edge. It needs
    // to be restored afterward.
    std::vector<TypedValue> last_frame_;
    // Edges which are being asynchronously fetched from a remote worker.
    // NOTE: This should be destructed first to ensure that no invalid
    // references or pointers exists to other objects of this class.
    std::vector<FutureExpand> future_expands_;

    bool InitEdges(Frame &, Context &);
  };
};

/**
 * Variable-length expansion operator. For a node existing in
 * the frame it expands a variable number of edges and places them
 * (in a list-type TypedValue), as well as the final destination node,
 * on the frame.
 *
 * This class does not handle node/edge filtering based on
 * properties, labels and edge types. However, it does handle
 * filtering on existing node / edge. Additionally it handles's
 * edge-uniquess (cyphermorphism) because it's not feasable to do
 * later.
 *
 * Filtering on existing means that for a pattern that references
 * an already declared node or edge (for example in
 * MATCH (a) MATCH (a)--(b)),
 * only expansions that match defined equalities are succesfully
 * pulled.
 */
class ExpandVariable : public LogicalOperator, public ExpandCommon {
  // the Cursors are not declared in the header because
  // it's edges_ and edges_it_ are decltyped using a helper function
  // that should be inaccessible (private class function won't compile)
  friend class ExpandVariableCursor;
  friend class ExpandBreadthFirstCursor;
  friend class ExpandWeightedShortestPathCursor;

 public:
  struct Lambda {
    // Symbols for a single node and edge that are currently getting expanded.
    Symbol inner_edge_symbol;
    Symbol inner_node_symbol;
    // Expression used in lambda during expansion.
    Expression *expression;
  };

  /**
   * Creates a variable-length expansion. Most params are forwarded
   * to the @c ExpandCommon constructor, and are documented there.
   *
   * Expansion length bounds are both inclusive (as in Neo's Cypher
   * implementation).
   *
   * @param type - Either Type::DEPTH_FIRST (default variable-length expansion),
   * or Type::BREADTH_FIRST.
   * @param is_reverse Set to `true` if the edges written on frame should expand
   *    from `node_symbol` to `input_symbol`. Opposed to the usual expanding
   *    from `input_symbol` to `node_symbol`.
   * @param lower_bound An optional indicator of the minimum number of edges
   *    that get expanded (inclusive).
   * @param upper_bound An optional indicator of the maximum number of edges
   *    that get expanded (inclusive).
   * @param inner_edge_symbol Like `inner_node_symbol`
   * @param inner_node_symbol For each expansion the node expanded into is
   *    assigned to this symbol so it can be evaulated by the 'where'
   * expression.
   * @param filter_ The filter that must be satisfied for an expansion to
   * succeed. Can use inner(node|edge) symbols. If nullptr, it is ignored.
   */
  ExpandVariable(Symbol node_symbol, Symbol edge_symbol, EdgeAtom::Type type,
                 EdgeAtom::Direction direction,
                 const std::vector<storage::EdgeType> &edge_types,
                 bool is_reverse, Expression *lower_bound,
                 Expression *upper_bound,
                 const std::shared_ptr<LogicalOperator> &input,
                 Symbol input_symbol, bool existing_node, Lambda filter_lambda,
                 std::experimental::optional<Lambda> weight_lambda,
                 std::experimental::optional<Symbol> total_weight,
                 GraphView graph_view = GraphView::AS_IS);

  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

  auto type() const { return type_; }

 private:
  EdgeAtom::Type type_;
  // True if the path should be written as expanding from node_symbol to
  // input_symbol.
  bool is_reverse_;
  // lower and upper bounds of the variable length expansion
  // both are optional, defaults are (1, inf)
  Expression *lower_bound_;
  Expression *upper_bound_;

  Lambda filter_lambda_;
  std::experimental::optional<Lambda> weight_lambda_;
  std::experimental::optional<Symbol> total_weight_;

  ExpandVariable() {}
};

/**
 * Constructs a named path from it's elements and places it on the frame.
 */
class ConstructNamedPath : public LogicalOperator {
 public:
  ConstructNamedPath(const std::shared_ptr<LogicalOperator> &input,
                     Symbol path_symbol,
                     const std::vector<Symbol> &path_elements)
      : input_(input),
        path_symbol_(path_symbol),
        path_elements_(path_elements) {}
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

  const auto &path_symbol() const { return path_symbol_; }
  const auto &path_elements() const { return path_elements_; }

 private:
  std::shared_ptr<LogicalOperator> input_;
  Symbol path_symbol_;
  std::vector<Symbol> path_elements_;

  ConstructNamedPath() {}
};

/**
 * @brief Filter whose Pull returns true only when the given expression
 * evaluates into true.
 *
 * The given expression is assumed to return either NULL (treated as false) or
 * a
 * boolean value.
 */
class Filter : public LogicalOperator {
 public:
  Filter(const std::shared_ptr<LogicalOperator> &input_,
         Expression *expression_);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

 private:
  std::shared_ptr<LogicalOperator> input_;
  Expression *expression_;

  Filter() {}

  class FilterCursor : public Cursor {
   public:
    FilterCursor(const Filter &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const Filter &self_;
    database::GraphDbAccessor &db_;
    const std::unique_ptr<Cursor> input_cursor_;
  };
};

/**
 * @brief A logical operator that places an arbitrary number
 * if named expressions on the frame (the logical operator
 * for the RETURN clause).
 *
 * Supports optional input. When the input is provided,
 * it is Pulled from and the Produce succeeds once for
 * every input Pull (typically a MATCH/RETURN query).
 * When the input is not provided (typically a standalone
 * RETURN clause) the Produce's pull succeeds exactly once.
 */
class Produce : public LogicalOperator {
 public:
  Produce(const std::shared_ptr<LogicalOperator> &input,
          const std::vector<NamedExpression *> &named_expressions);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> OutputSymbols(const SymbolTable &) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

  const std::vector<NamedExpression *> &named_expressions();

 private:
  std::shared_ptr<LogicalOperator> input_;
  std::vector<NamedExpression *> named_expressions_;

  class ProduceCursor : public Cursor {
   public:
    ProduceCursor(const Produce &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const Produce &self_;
    database::GraphDbAccessor &db_;
    const std::unique_ptr<Cursor> input_cursor_;
  };

  Produce() {}
};

/**
 * @brief Operator for deleting vertices and edges.
 *
 * Has a flag for using DETACH DELETE when deleting
 * vertices.
 */
class Delete : public LogicalOperator {
 public:
  Delete(const std::shared_ptr<LogicalOperator> &input_,
         const std::vector<Expression *> &expressions, bool detach_);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

 private:
  std::shared_ptr<LogicalOperator> input_;
  std::vector<Expression *> expressions_;
  // if the vertex should be detached before deletion
  // if not detached, and has connections, an error is raised
  // ignored when deleting edges
  bool detach_;

  Delete() {}

  class DeleteCursor : public Cursor {
   public:
    DeleteCursor(const Delete &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const Delete &self_;
    database::GraphDbAccessor &db_;
    const std::unique_ptr<Cursor> input_cursor_;
  };
};

/**
 * @brief Logical Op for setting a single property on a single vertex or edge.
 *
 * The property value is an expression that must evaluate to some type that
 * can be stored (a TypedValue that can be converted to PropertyValue).
 */
class SetProperty : public LogicalOperator {
 public:
  SetProperty(const std::shared_ptr<LogicalOperator> &input,
              PropertyLookup *lhs, Expression *rhs);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

 private:
  std::shared_ptr<LogicalOperator> input_;
  PropertyLookup *lhs_;
  Expression *rhs_;

  SetProperty() {}

  class SetPropertyCursor : public Cursor {
   public:
    SetPropertyCursor(const SetProperty &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const SetProperty &self_;
    database::GraphDbAccessor &db_;
    const std::unique_ptr<Cursor> input_cursor_;
  };
};

/**
 * @brief Logical op for setting the whole properties set on a vertex or an
 * edge.
 *
 * The value being set is an expression that must evaluate to a vertex, edge
 * or
 * map (literal or parameter).
 *
 * Supports setting (replacing the whole properties set with another) and
 * updating.
 */
class SetProperties : public LogicalOperator {
 public:
  /**
   * @brief Defines how setting the properties works.
   *
   * @c UPDATE means that the current property set is augmented with
   * additional
   * ones (existing props of the same name are replaced), while @c REPLACE
   * means
   * that the old props are discarded and replaced with new ones.
   */
  enum class Op { UPDATE, REPLACE };

  SetProperties(const std::shared_ptr<LogicalOperator> &input,
                Symbol input_symbol, Expression *rhs, Op op);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

 private:
  std::shared_ptr<LogicalOperator> input_;
  Symbol input_symbol_;
  Expression *rhs_;
  Op op_;

  SetProperties() {}

  class SetPropertiesCursor : public Cursor {
   public:
    SetPropertiesCursor(const SetProperties &self,
                        database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const SetProperties &self_;
    database::GraphDbAccessor &db_;
    const std::unique_ptr<Cursor> input_cursor_;

    /** Helper function that sets the given values on either
     * a VertexRecord or an EdgeRecord.
     * @tparam TRecordAccessor Either RecordAccessor<Vertex> or
     * RecordAccessor<Edge>
     */
    template <typename TRecordAccessor>
    void Set(TRecordAccessor &record, const TypedValue &rhs) const;
  };
};

/**
 * @brief Logical operator for setting an arbitrary number of labels on a
 * Vertex.
 *
 * It does NOT remove labels that are already set on that Vertex.
 */
class SetLabels : public LogicalOperator {
 public:
  SetLabels(const std::shared_ptr<LogicalOperator> &input, Symbol input_symbol,
            const std::vector<storage::Label> &labels);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

 private:
  std::shared_ptr<LogicalOperator> input_;
  Symbol input_symbol_;
  std::vector<storage::Label> labels_;

  SetLabels() {}

  class SetLabelsCursor : public Cursor {
   public:
    SetLabelsCursor(const SetLabels &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const SetLabels &self_;
    const std::unique_ptr<Cursor> input_cursor_;
  };
};

/**
 * @brief Logical op for removing a property from an
 * edge or a vertex.
 */
class RemoveProperty : public LogicalOperator {
 public:
  RemoveProperty(const std::shared_ptr<LogicalOperator> &input,
                 PropertyLookup *lhs);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

 private:
  std::shared_ptr<LogicalOperator> input_;
  PropertyLookup *lhs_;

  RemoveProperty() {}

  class RemovePropertyCursor : public Cursor {
   public:
    RemovePropertyCursor(const RemoveProperty &self,
                         database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const RemoveProperty &self_;
    database::GraphDbAccessor &db_;
    const std::unique_ptr<Cursor> input_cursor_;
  };
};

/**
 * @brief Logical operator for removing an arbitrary number of
 * labels on a Vertex.
 *
 * If a label does not exist on a Vertex, nothing happens.
 */
class RemoveLabels : public LogicalOperator {
 public:
  RemoveLabels(const std::shared_ptr<LogicalOperator> &input,
               Symbol input_symbol, const std::vector<storage::Label> &labels);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

 private:
  std::shared_ptr<LogicalOperator> input_;
  Symbol input_symbol_;
  std::vector<storage::Label> labels_;

  RemoveLabels() {}

  class RemoveLabelsCursor : public Cursor {
   public:
    RemoveLabelsCursor(const RemoveLabels &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const RemoveLabels &self_;
    const std::unique_ptr<Cursor> input_cursor_;
  };
};

/**
 * Filter whose Pull returns true only when the given
 * expand_symbol frame value (the latest expansion) is not
 * equal to any of the previous_symbols frame values.
 *
 * Used for implementing [iso|cypher]morphism.
 * Isomorphism is vertex-uniqueness. It means that
 * two different vertices in a pattern can not map to the
 * same data vertex. For example, if the database
 * contains one vertex with a recursive relationship,
 * then the query
 * MATCH ()-[]->() combined with vertex uniqueness
 * yields no results (no uniqueness yields one).
 * Cyphermorphism is edge-uniqueness (the above
 * explanation applies). By default Neo4j uses
 * Cyphermorphism (that's where the name stems from,
 * it is not a valid graph-theory term).
 *
 * Works for both Edge and Vertex uniqueness checks
 * (provide the accessor type as a template argument).
 * Supports variable-length-edges (uniqueness comparisons
 * between edges and an edge lists).
 */
template <typename TAccessor>
class ExpandUniquenessFilter : public LogicalOperator {
 public:
  ExpandUniquenessFilter(const std::shared_ptr<LogicalOperator> &input,
                         Symbol expand_symbol,
                         const std::vector<Symbol> &previous_symbols);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

 private:
  std::shared_ptr<LogicalOperator> input_;
  Symbol expand_symbol_;
  std::vector<Symbol> previous_symbols_;

  ExpandUniquenessFilter() {}

  class ExpandUniquenessFilterCursor : public Cursor {
   public:
    ExpandUniquenessFilterCursor(const ExpandUniquenessFilter &self,
                                 database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const ExpandUniquenessFilter &self_;
    const std::unique_ptr<Cursor> input_cursor_;
  };
};

/** @brief Pulls everything from the input before passing it through.
 * Optionally advances the command after accumulation and before emitting.
 *
 * On the first Pull from this Op's Cursor the input Cursor will be
 * Pulled until it is empty. The results will be accumulated in the
 * temporary cache. Once the input Cursor is empty, this Op's Cursor
 * will start returning cached stuff from it's Pull.
 *
 * This technique is used for ensuring all the operations from the
 * previous LogicalOp have been performed before exposing data
 * to the next. A typical use-case is the `MATCH - SET - RETURN`
 * query in which every SET iteration must be performed before
 * RETURN starts iterating (see Memgraph Wiki for detailed reasoning).
 *
 * IMPORTANT: This Op does not cache all the results but only those
 * elements from the frame whose symbols (frame positions) it was given.
 * All other frame positions will contain undefined junk after this
 * op has executed, and should not be used.
 *
 * This op can also advance the command after the accumulation and
 * before emitting. If the command gets advanced, every value that
 * has been cached will be reconstructed before Pull returns.
 *
 * @param input Input @c LogicalOperator.
 * @param symbols A vector of Symbols that need to be accumulated
 *  and exposed to the next op.
 */
class Accumulate : public LogicalOperator {
 public:
  Accumulate(const std::shared_ptr<LogicalOperator> &input,
             const std::vector<Symbol> &symbols, bool advance_command = false);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

  const auto &symbols() const { return symbols_; };
  auto advance_command() const { return advance_command_; }

 private:
  std::shared_ptr<LogicalOperator> input_;
  std::vector<Symbol> symbols_;
  bool advance_command_;

  Accumulate() {}

  class AccumulateCursor : public Cursor {
   public:
    AccumulateCursor(const Accumulate &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const Accumulate &self_;
    database::GraphDbAccessor &db_;
    const std::unique_ptr<Cursor> input_cursor_;
    std::vector<std::vector<TypedValue>> cache_;
    decltype(cache_.begin()) cache_it_ = cache_.begin();
    bool pulled_all_input_{false};
  };
};

/**
 * Custom equality function for a vector of typed values.
 * Used in unordered_maps in Aggregate and Distinct operators.
 */
struct TypedValueVectorEqual {
  bool operator()(const std::vector<TypedValue> &left,
                  const std::vector<TypedValue> &right) const;
};

/** @brief Performs an arbitrary number of aggregations of data
 * from the given input grouped by the given criteria.
 *
 * Aggregations are defined by triples that define
 * (input data expression, type of aggregation, output symbol).
 * Input data is grouped based on the given set of named
 * expressions. Grouping is done on unique values.
 *
 * IMPORTANT:
 * Ops taking their input from an aggregation are only
 * allowed to use frame values that are either aggregation
 * outputs or group-by named-expressions. All other frame
 * elements are in an undefined state after aggregation.
 */
class Aggregate : public LogicalOperator {
 public:
  /** @brief An aggregation element, contains:
   * (input data expression, key expression - only used in COLLECT_MAP, type of
   * aggregation, output symbol).
   */
  struct Element {
    Expression *value;
    Expression *key;
    Aggregation::Op op;
    Symbol output_sym;
  };

  Aggregate(const std::shared_ptr<LogicalOperator> &input,
            const std::vector<Element> &aggregations,
            const std::vector<Expression *> &group_by,
            const std::vector<Symbol> &remember);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

  const auto &aggregations() const { return aggregations_; }
  const auto &group_by() const { return group_by_; }
  const auto &remember() const { return remember_; }

 private:
  std::shared_ptr<LogicalOperator> input_;
  std::vector<Element> aggregations_;
  std::vector<Expression *> group_by_;
  std::vector<Symbol> remember_;

  Aggregate() {}

  class AggregateCursor : public Cursor {
   public:
    AggregateCursor(const Aggregate &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    // Data structure for a single aggregation cache.
    // does NOT include the group-by values since those
    // are a key in the aggregation map.
    // The vectors in an AggregationValue contain one element
    // for each aggregation in this LogicalOp.
    struct AggregationValue {
      // how many input rows has been aggregated in respective
      // values_ element so far
      std::vector<int> counts_;
      // aggregated values. Initially Null (until at least one
      // input row with a valid value gets processed)
      std::vector<TypedValue> values_;
      // remember values.
      std::vector<TypedValue> remember_;
    };

    const Aggregate &self_;
    database::GraphDbAccessor &db_;
    const std::unique_ptr<Cursor> input_cursor_;
    // storage for aggregated data
    // map key is the vector of group-by values
    // map value is an AggregationValue struct
    std::unordered_map<
        std::vector<TypedValue>, AggregationValue,
        // use FNV collection hashing specialized for a vector of TypedValues
        utils::FnvCollection<std::vector<TypedValue>, TypedValue,
                             TypedValue::Hash>,
        // custom equality
        TypedValueVectorEqual>
        aggregation_;
    // iterator over the accumulated cache
    decltype(aggregation_.begin()) aggregation_it_ = aggregation_.begin();
    // this LogicalOp pulls all from the input on it's first pull
    // this switch tracks if this has been performed
    bool pulled_all_input_{false};

    /**
     * Pulls from the input operator until exhausted and aggregates the
     * results. If the input operator is not provided, a single call
     * to ProcessOne is issued.
     *
     * Accumulation automatically groups the results so that `aggregation_`
     * cache cardinality depends on number of
     * aggregation results, and not on the number of inputs.
     */
    void ProcessAll(Frame &, Context &);

    /**
     * Performs a single accumulation.
     */
    void ProcessOne(Frame &frame, const SymbolTable &symbolTable,
                    ExpressionEvaluator &evaluator);

    /** Ensures the new AggregationValue has been initialized. This means
     * that the value vectors are filled with an appropriate number of Nulls,
     * counts are set to 0 and remember values are remembered.
     */
    void EnsureInitialized(Frame &frame, AggregationValue &agg_value) const;

    /** Updates the given AggregationValue with new data. Assumes that
     * the AggregationValue has been initialized */
    void Update(Frame &frame, const SymbolTable &symbol_table,
                ExpressionEvaluator &evaluator, AggregationValue &agg_value);

    /** Checks if the given TypedValue is legal in MIN and MAX. If not
     * an appropriate exception is thrown. */
    void EnsureOkForMinMax(const TypedValue &value) const;

    /** Checks if the given TypedValue is legal in AVG and SUM. If not
     * an appropriate exception is thrown. */
    void EnsureOkForAvgSum(const TypedValue &value) const;
  };
};

/** @brief Skips a number of Pulls from the input op.
 *
 * The given expression determines how many Pulls from the input
 * should be skipped (ignored).
 * All other successful Pulls from the
 * input are simply passed through.
 *
 * The given expression is evaluated after the first Pull from
 * the input, and only once. Neo does not allow this expression
 * to contain identifiers, and neither does Memgraph, but this
 * operator's implementation does not expect this.
 */
class Skip : public LogicalOperator {
 public:
  Skip(const std::shared_ptr<LogicalOperator> &input, Expression *expression);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> OutputSymbols(const SymbolTable &) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

 private:
  std::shared_ptr<LogicalOperator> input_;
  Expression *expression_;

  Skip() {}

  class SkipCursor : public Cursor {
   public:
    SkipCursor(const Skip &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const Skip &self_;
    database::GraphDbAccessor &db_;
    const std::unique_ptr<Cursor> input_cursor_;
    // init to_skip_ to -1, indicating
    // that it's still unknown (input has not been Pulled yet)
    int to_skip_{-1};
    int skipped_{0};
  };
};

/** @brief Limits the number of Pulls from the input op.
 *
 * The given expression determines how many
 * input Pulls should be passed through. The input is not
 * Pulled once this limit is reached. Note that this has
 * implications: the out-of-bounds input Pulls are never
 * evaluated.
 *
 * The limit expression must NOT use anything from the
 * Frame. It is evaluated before the first Pull from the
 * input. This is consistent with Neo (they don't allow
 * identifiers in limit expressions), and it's necessary
 * when limit evaluates to 0 (because 0 Pulls from the
 * input should be performed).
 */
class Limit : public LogicalOperator {
 public:
  Limit(const std::shared_ptr<LogicalOperator> &input, Expression *expression);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> OutputSymbols(const SymbolTable &) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

 private:
  std::shared_ptr<LogicalOperator> input_;
  Expression *expression_;

  Limit() {}

  class LimitCursor : public Cursor {
   public:
    LimitCursor(const Limit &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const Limit &self_;
    database::GraphDbAccessor &db_;
    std::unique_ptr<Cursor> input_cursor_;
    // init limit_ to -1, indicating
    // that it's still unknown (Cursor has not been Pulled yet)
    int limit_{-1};
    int pulled_{0};
  };
};

/** @brief Logical operator for ordering (sorting) results.
 *
 * Sorts the input rows based on an arbitrary number of
 * Expressions. Ascending or descending ordering can be chosen
 * for each independently (not providing enough orderings
 * results in a runtime error).
 *
 * For each row an arbitrary number of Frame elements can be
 * remembered. Only these elements (defined by their Symbols)
 * are valid for usage after the OrderBy operator.
 */
class OrderBy : public LogicalOperator {
 public:
  OrderBy(const std::shared_ptr<LogicalOperator> &input,
          const std::vector<std::pair<Ordering, Expression *>> &order_by,
          const std::vector<Symbol> &output_symbols);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> OutputSymbols(const SymbolTable &) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

  const auto &order_by() const { return order_by_; }
  const auto &compare() const { return compare_; }
  const auto &output_symbols() const { return output_symbols_; }

 private:
  std::shared_ptr<LogicalOperator> input_;
  TypedValueVectorCompare compare_;
  std::vector<Expression *> order_by_;
  std::vector<Symbol> output_symbols_;

  OrderBy() {}

  class OrderByCursor : public Cursor {
   public:
    OrderByCursor(const OrderBy &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const OrderBy &self_;
    database::GraphDbAccessor &db_;
    const std::unique_ptr<Cursor> input_cursor_;
    bool did_pull_all_{false};
    // a cache of elements pulled from the input
    // first pair element is the order-by vector
    // second pair is the remember vector
    // the cache is filled and sorted (only on first pair elem) on first Pull
    std::vector<std::pair<std::vector<TypedValue>, std::vector<TypedValue>>>
        cache_;
    // iterator over the cache_, maintains state between Pulls
    decltype(cache_.begin()) cache_it_ = cache_.begin();
  };
};

/**
 * Merge operator. For every sucessful Pull from the
 * input operator a Pull from the merge_match is attempted. All
 * successfull Pulls from the merge_match are passed on as output.
 * If merge_match Pull does not yield any elements, a single Pull
 * from the merge_create op is performed.
 *
 * The input logical op is optional. If false (nullptr)
 * it will be replaced by a Once op.
 *
 * For an argumentation of this implementation see the wiki
 * documentation.
 */
class Merge : public LogicalOperator {
 public:
  Merge(const std::shared_ptr<LogicalOperator> &input,
        const std::shared_ptr<LogicalOperator> &merge_match,
        const std::shared_ptr<LogicalOperator> &merge_create);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  // TODO: Consider whether we want to treat Merge as having single input. It
  // makes sense that we do, because other branches are executed depending on
  // the input.
  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

  auto merge_match() const { return merge_match_; }
  auto merge_create() const { return merge_create_; }

 private:
  std::shared_ptr<LogicalOperator> input_;
  std::shared_ptr<LogicalOperator> merge_match_;
  std::shared_ptr<LogicalOperator> merge_create_;

  Merge() {}

  class MergeCursor : public Cursor {
   public:
    MergeCursor(const Merge &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const std::unique_ptr<Cursor> input_cursor_;
    const std::unique_ptr<Cursor> merge_match_cursor_;
    const std::unique_ptr<Cursor> merge_create_cursor_;

    // indicates if the next Pull from this cursor
    // should perform a pull from input_cursor_
    // this is true when:
    //  - first Pulling from this cursor
    //  - previous Pull from this cursor exhausted the merge_match_cursor
    bool pull_input_{true};
  };
};

/**
 * Optional operator. Used for optional match. For every
 * successful Pull from the input branch a Pull from the optional
 * branch is attempted (and Pulled from till exhausted). If zero
 * Pulls succeed from the optional branch, the Optional operator
 * sets the optional symbols to TypedValue::Null on the Frame
 * and returns true, once.
 */
class Optional : public LogicalOperator {
 public:
  Optional(const std::shared_ptr<LogicalOperator> &input,
           const std::shared_ptr<LogicalOperator> &optional,
           const std::vector<Symbol> &optional_symbols);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  // TODO: Consider whether we want to treat Optional as having single input.
  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

  auto optional() const { return optional_; }
  const auto &optional_symbols() const { return optional_symbols_; }

 private:
  std::shared_ptr<LogicalOperator> input_;
  std::shared_ptr<LogicalOperator> optional_;
  std::vector<Symbol> optional_symbols_;

  Optional() {}

  class OptionalCursor : public Cursor {
   public:
    OptionalCursor(const Optional &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const Optional &self_;
    const std::unique_ptr<Cursor> input_cursor_;
    const std::unique_ptr<Cursor> optional_cursor_;
    // indicates if the next Pull from this cursor should
    // perform a Pull from the input_cursor_
    // this is true when:
    //  - first pulling from this Cursor
    //  - previous Pull from this cursor exhausted the optional_cursor_
    bool pull_input_{true};
  };
};

/**
 * Takes a list TypedValue as it's input and yields each
 * element as it's output.
 *
 * Input is optional (unwind can be the first clause in a query).
 */
class Unwind : public LogicalOperator {
 public:
  Unwind(const std::shared_ptr<LogicalOperator> &input,
         Expression *input_expression_, Symbol output_symbol);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

  Expression *input_expression() const { return input_expression_; }

 private:
  std::shared_ptr<LogicalOperator> input_;
  Expression *input_expression_;
  Symbol output_symbol_;

  Unwind() {}

  class UnwindCursor : public Cursor {
   public:
    UnwindCursor(const Unwind &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const Unwind &self_;
    database::GraphDbAccessor &db_;
    const std::unique_ptr<Cursor> input_cursor_;
    // typed values we are unwinding and yielding
    std::vector<TypedValue> input_value_;
    // current position in input_value_
    std::vector<TypedValue>::iterator input_value_it_ = input_value_.end();
  };
};

/**
 * Ensures that only distinct rows are yielded.
 * This implementation accepts a vector of Symbols
 * which define a row. Only those Symbols are valid
 * for use in operators following Distinct.
 *
 * This implementation maintains input ordering.
 */
class Distinct : public LogicalOperator {
 public:
  Distinct(const std::shared_ptr<LogicalOperator> &input,
           const std::vector<Symbol> &value_symbols);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> OutputSymbols(const SymbolTable &) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override { return true; }
  std::shared_ptr<LogicalOperator> input() const override { return input_; }
  void set_input(std::shared_ptr<LogicalOperator> input) override {
    input_ = input;
  }

 private:
  std::shared_ptr<LogicalOperator> input_;
  std::vector<Symbol> value_symbols_;

  Distinct() {}

  class DistinctCursor : public Cursor {
   public:
    DistinctCursor(const Distinct &self, database::GraphDbAccessor &db);

    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const Distinct &self_;
    const std::unique_ptr<Cursor> input_cursor_;
    // a set of already seen rows
    std::unordered_set<
        std::vector<TypedValue>,
        // use FNV collection hashing specialized for a vector of TypedValues
        utils::FnvCollection<std::vector<TypedValue>, TypedValue,
                             TypedValue::Hash>,
        TypedValueVectorEqual>
        seen_rows_;
  };
};

/**
 * Creates an index for a combination of label and a property.
 *
 * This operator takes no input and it shouldn't serve as an input to any
 * operator. Pulling from the cursor of this operator will create an index in
 * the database for the vertices which have the given label and property. In
 * case the index already exists, nothing happens.
 */
class CreateIndex : public LogicalOperator {
 public:
  CreateIndex(storage::Label label, storage::Property property);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override {
    return {};
  }

  bool HasSingleInput() const override;
  std::shared_ptr<LogicalOperator> input() const override;
  void set_input(std::shared_ptr<LogicalOperator>) override;

  auto label() const { return label_; }
  auto property() const { return property_; }

 private:
  storage::Label label_;
  storage::Property property_;

  CreateIndex() {}
};

/**
 * A logical operator that applies UNION operator on inputs and places the
 * result on the frame.
 *
 * This operator takes two inputs, a vector of symbols for the result, and
 * vectors of symbols used  by each of the inputs.
 */
class Union : public LogicalOperator {
 public:
  Union(const std::shared_ptr<LogicalOperator> &left_op,
        const std::shared_ptr<LogicalOperator> &right_op,
        const std::vector<Symbol> &union_symbols,
        const std::vector<Symbol> &left_symbols,
        const std::vector<Symbol> &right_symbols);
  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> OutputSymbols(const SymbolTable &) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override;
  std::shared_ptr<LogicalOperator> input() const override;
  void set_input(std::shared_ptr<LogicalOperator>) override;

 private:
  std::shared_ptr<LogicalOperator> left_op_, right_op_;
  std::vector<Symbol> union_symbols_, left_symbols_, right_symbols_;

  Union() {}

  class UnionCursor : public Cursor {
   public:
    UnionCursor(const Union &self, database::GraphDbAccessor &db);
    bool Pull(Frame &, Context &) override;
    void Reset() override;

   private:
    const Union &self_;
    const std::unique_ptr<Cursor> left_cursor_, right_cursor_;
  };
};

/** Operator for producing a Cartesian product from 2 input branches */
class Cartesian : public LogicalOperator {
 public:
  /** Construct the operator with left input branch and right input branch. */
  Cartesian(const std::shared_ptr<LogicalOperator> &left_op,
            const std::vector<Symbol> &left_symbols,
            const std::shared_ptr<LogicalOperator> &right_op,
            const std::vector<Symbol> &right_symbols)
      : left_op_(left_op),
        left_symbols_(left_symbols),
        right_op_(right_op),
        right_symbols_(right_symbols) {}

  bool Accept(HierarchicalLogicalOperatorVisitor &visitor) override;
  std::unique_ptr<Cursor> MakeCursor(
      database::GraphDbAccessor &db) const override;
  std::vector<Symbol> ModifiedSymbols(const SymbolTable &) const override;

  bool HasSingleInput() const override;
  std::shared_ptr<LogicalOperator> input() const override;
  void set_input(std::shared_ptr<LogicalOperator>) override;

  auto left_op() const { return left_op_; }
  auto left_symbols() const { return left_symbols_; }
  auto right_op() const { return right_op_; }
  auto right_symbols() const { return right_symbols_; }

 private:
  std::shared_ptr<LogicalOperator> left_op_;
  std::vector<Symbol> left_symbols_;
  std::shared_ptr<LogicalOperator> right_op_;
  std::vector<Symbol> right_symbols_;

  Cartesian() {}
};

}  // namespace plan
}  // namespace query

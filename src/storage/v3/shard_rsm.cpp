// Copyright 2022 Memgraph Ltd.
//
// Use of this software is governed by the Business Source License
// included in the file licenses/BSL.txt; by using this file, you agree to be bound by the terms of the Business Source
// License, and you may not use this file except in compliance with the Business Source License.
//
// As of the Change Date specified in that file, in accordance with
// the Business Source License, use of this software will be governed
// by the Apache License, Version 2.0, included in the file
// licenses/APL.txt.

#include <algorithm>
#include <functional>
#include <iterator>
#include <optional>
#include <unordered_set>
#include <utility>

#include "parser/opencypher/parser.hpp"
#include "query/v2/requests.hpp"
#include "storage/v2/vertex.hpp"
#include "storage/v2/view.hpp"
#include "storage/v3/bindings/ast/ast.hpp"
#include "storage/v3/bindings/cypher_main_visitor.hpp"
#include "storage/v3/bindings/db_accessor.hpp"
#include "storage/v3/bindings/eval.hpp"
#include "storage/v3/bindings/frame.hpp"
#include "storage/v3/bindings/pretty_print_ast_to_original_expression.hpp"
#include "storage/v3/bindings/symbol_generator.hpp"
#include "storage/v3/bindings/symbol_table.hpp"
#include "storage/v3/bindings/typed_value.hpp"
#include "storage/v3/expr.hpp"
#include "storage/v3/id_types.hpp"
#include "storage/v3/key_store.hpp"
#include "storage/v3/property_value.hpp"
#include "storage/v3/request_helper.hpp"
#include "storage/v3/result.hpp"
#include "storage/v3/schemas.hpp"
#include "storage/v3/shard.hpp"
#include "storage/v3/shard_rsm.hpp"
#include "storage/v3/value_conversions.hpp"
#include "storage/v3/vertex_accessor.hpp"
#include "storage/v3/vertex_id.hpp"
#include "storage/v3/view.hpp"

namespace memgraph::storage::v3 {
using msgs::Label;
using msgs::PropertyId;
using msgs::Value;

using conversions::ConvertPropertyVector;
using conversions::ConvertValueVector;
using conversions::FromPropertyValueToValue;
using conversions::ToMsgsVertexId;
using conversions::ToPropertyValue;

namespace {
namespace msgs = msgs;

using AllEdgePropertyDataStructure = std::map<PropertyId, msgs::Value>;
using SpecificEdgePropertyDataStructure = std::vector<msgs::Value>;

using AllEdgeProperties = std::tuple<msgs::VertexId, msgs::Gid, AllEdgePropertyDataStructure>;
using SpecificEdgeProperties = std::tuple<msgs::VertexId, msgs::Gid, SpecificEdgePropertyDataStructure>;

using SpecificEdgePropertiesVector = std::vector<SpecificEdgeProperties>;
using AllEdgePropertiesVector = std::vector<AllEdgeProperties>;

using EdgeAccessors = std::vector<storage::v3::EdgeAccessor>;

using EdgeFiller =
    std::function<ShardResult<void>(const EdgeAccessor &edge, bool is_in_edge, msgs::ExpandOneResultRow &result_row)>;
using EdgeUniquenessFunction = std::function<EdgeAccessors(EdgeAccessors &&, msgs::EdgeDirection)>;

struct VertexIdCmpr {
  bool operator()(const storage::v3::VertexId *lhs, const storage::v3::VertexId *rhs) const { return *lhs < *rhs; }
};

std::vector<std::pair<PropertyId, PropertyValue>> ConvertPropertyMap(
    std::vector<std::pair<PropertyId, Value>> &&properties) {
  std::vector<std::pair<PropertyId, PropertyValue>> ret;
  ret.reserve(properties.size());

  std::transform(std::make_move_iterator(properties.begin()), std::make_move_iterator(properties.end()),
                 std::back_inserter(ret), [](std::pair<PropertyId, Value> &&property) {
                   return std::make_pair(property.first, ToPropertyValue(std::move(property.second)));
                 });

  return ret;
}

std::vector<std::pair<PropertyId, Value>> FromMap(const std::map<PropertyId, Value> &properties) {
  std::vector<std::pair<PropertyId, Value>> ret;
  ret.reserve(properties.size());

  std::transform(properties.begin(), properties.end(), std::back_inserter(ret),
                 [](const auto &property) { return std::make_pair(property.first, property.second); });

  return ret;
}

std::optional<std::map<PropertyId, Value>> CollectSpecificPropertiesFromAccessor(const VertexAccessor &acc,
                                                                                 const std::vector<PropertyId> &props,
                                                                                 View view) {
  std::map<PropertyId, Value> ret;

  for (const auto &prop : props) {
    auto result = acc.GetProperty(prop, view);
    if (result.HasError()) {
      spdlog::debug("Encountered an Error while trying to get a vertex property.");
      return std::nullopt;
    }
    auto &value = result.GetValue();
    ret.emplace(std::make_pair(prop, FromPropertyValueToValue(std::move(value))));
  }

  return ret;
}

std::optional<std::map<PropertyId, Value>> PrimaryKeysFromAccessor(const VertexAccessor &acc, View view,
                                                                   const Schemas::Schema *schema) {
  std::map<PropertyId, Value> ret;
  auto props = acc.Properties(view);
  auto maybe_pk = acc.PrimaryKey(view);
  if (maybe_pk.HasError()) {
    spdlog::debug("Encountered an error while trying to get vertex primary key.");
    return std::nullopt;
  }
  auto &pk = maybe_pk.GetValue();
  MG_ASSERT(schema->second.size() == pk.size(), "PrimaryKey size does not match schema!");
  for (size_t i{0}; i < schema->second.size(); ++i) {
    ret.emplace(schema->second[i].property_id, FromPropertyValueToValue(std::move(pk[i])));
  }

  return ret;
}

std::optional<std::map<PropertyId, Value>> CollectAllPropertiesFromAccessor(const VertexAccessor &acc, View view,
                                                                            const Schemas::Schema *schema) {
  std::map<PropertyId, Value> ret;
  auto props = acc.Properties(view);
  if (props.HasError()) {
    spdlog::debug("Encountered an error while trying to get vertex properties.");
    return std::nullopt;
  }

  auto &properties = props.GetValue();
  std::transform(properties.begin(), properties.end(), std::inserter(ret, ret.begin()),
                 [](std::pair<const PropertyId, PropertyValue> &pair) {
                   return std::make_pair(pair.first, FromPropertyValueToValue(std::move(pair.second)));
                 });
  properties.clear();

  auto pks = PrimaryKeysFromAccessor(acc, view, schema);
  if (pks) {
    ret.merge(*pks);
  }

  return ret;
}

bool FilterOnVertex(DbAccessor &dba, const storage::v3::VertexAccessor &v_acc, const std::vector<std::string> &filters,
                    const std::string_view node_name) {
  return std::ranges::all_of(filters, [&node_name, &dba, &v_acc](const auto &filter_expr) {
    auto res = ComputeExpression(dba, v_acc, std::nullopt, filter_expr, node_name, "");
    return res.IsBool() && res.ValueBool();
  });
}

std::vector<TypedValue> EvaluateVertexExpressions(DbAccessor &dba, const VertexAccessor &v_acc,
                                                  const std::vector<std::string> &expressions,
                                                  std::string_view node_name) {
  std::vector<TypedValue> evaluated_expressions;
  evaluated_expressions.reserve(expressions.size());

  std::transform(expressions.begin(), expressions.end(), std::back_inserter(evaluated_expressions),
                 [&dba, &v_acc, &node_name](const auto &expression) {
                   return ComputeExpression(dba, v_acc, std::nullopt, expression, node_name, "");
                 });

  return evaluated_expressions;
}

ShardResult<std::vector<msgs::Label>> FillUpSourceVertexSecondaryLabels(const std::optional<VertexAccessor> &v_acc,
                                                                        const msgs::ExpandOneRequest &req) {
  auto secondary_labels = v_acc->Labels(View::NEW);
  if (secondary_labels.HasError()) {
    spdlog::debug("Encountered an error while trying to get the secondary labels of a vertex. Transaction id: {}",
                  req.transaction_id.logical_id);
    return secondary_labels.GetError();
  }

  auto &sec_labels = secondary_labels.GetValue();
  std::vector<msgs::Label> msgs_secondary_labels;
  msgs_secondary_labels.reserve(sec_labels.size());

  std::transform(sec_labels.begin(), sec_labels.end(), std::back_inserter(msgs_secondary_labels),
                 [](auto label_id) { return msgs::Label{.id = label_id}; });

  return msgs_secondary_labels;
}

ShardResult<std::map<PropertyId, Value>> FillUpSourceVertexProperties(const std::optional<VertexAccessor> &v_acc,
                                                                      const msgs::ExpandOneRequest &req,
                                                                      storage::v3::View view,
                                                                      const Schemas::Schema *schema) {
  std::map<PropertyId, Value> src_vertex_properties;

  if (!req.src_vertex_properties) {
    auto props = v_acc->Properties(View::NEW);
    if (props.HasError()) {
      spdlog::debug("Encountered an error while trying to access vertex properties. Transaction id: {}",
                    req.transaction_id.logical_id);
      return props.GetError();
    }

    for (auto &[key, val] : props.GetValue()) {
      src_vertex_properties.insert(std::make_pair(key, FromPropertyValueToValue(std::move(val))));
    }
    auto pks = PrimaryKeysFromAccessor(*v_acc, view, schema);
    if (pks) {
      src_vertex_properties.merge(*pks);
    }

  } else if (req.src_vertex_properties.value().empty()) {
    // NOOP
  } else {
    for (const auto &prop : req.src_vertex_properties.value()) {
      auto prop_val = v_acc->GetProperty(prop, View::OLD);
      if (prop_val.HasError()) {
        spdlog::debug("Encountered an error while trying to access vertex properties. Transaction id: {}",
                      req.transaction_id.logical_id);
        return prop_val.GetError();
      }
      src_vertex_properties.insert(std::make_pair(prop, FromPropertyValueToValue(std::move(prop_val.GetValue()))));
    }
  }

  return src_vertex_properties;
}

ShardResult<std::array<std::vector<EdgeAccessor>, 2>> FillUpConnectingEdges(
    const std::optional<VertexAccessor> &v_acc, const msgs::ExpandOneRequest &req,
    const EdgeUniquenessFunction &maybe_filter_based_on_edge_uniquness) {
  std::vector<EdgeTypeId> edge_types{};
  edge_types.reserve(req.edge_types.size());
  std::transform(req.edge_types.begin(), req.edge_types.end(), std::back_inserter(edge_types),
                 [](const msgs::EdgeType &edge_type) { return edge_type.id; });

  std::vector<EdgeAccessor> in_edges;
  std::vector<EdgeAccessor> out_edges;

  switch (req.direction) {
    case msgs::EdgeDirection::OUT: {
      auto out_edges_result = v_acc->OutEdges(View::NEW, edge_types);
      if (out_edges_result.HasError()) {
        spdlog::debug("Encountered an error while trying to get out-going EdgeAccessors. Transaction id: {}",
                      req.transaction_id.logical_id);
        return out_edges_result.GetError();
      }
      out_edges =
          maybe_filter_based_on_edge_uniquness(std::move(out_edges_result.GetValue()), msgs::EdgeDirection::OUT);
      break;
    }
    case msgs::EdgeDirection::IN: {
      auto in_edges_result = v_acc->InEdges(View::NEW, edge_types);
      if (in_edges_result.HasError()) {
        spdlog::debug(
            "Encountered an error while trying to get in-going EdgeAccessors. Transaction id: {}"[req.transaction_id
                                                                                                      .logical_id]);
        return in_edges_result.GetError();
      }
      in_edges = maybe_filter_based_on_edge_uniquness(std::move(in_edges_result.GetValue()), msgs::EdgeDirection::IN);
      break;
    }
    case msgs::EdgeDirection::BOTH: {
      auto in_edges_result = v_acc->InEdges(View::NEW, edge_types);
      if (in_edges_result.HasError()) {
        spdlog::debug("Encountered an error while trying to get in-going EdgeAccessors. Transaction id: {}",
                      req.transaction_id.logical_id);
        return in_edges_result.GetError();
      }
      in_edges = maybe_filter_based_on_edge_uniquness(std::move(in_edges_result.GetValue()), msgs::EdgeDirection::IN);
      auto out_edges_result = v_acc->OutEdges(View::NEW, edge_types);
      if (out_edges_result.HasError()) {
        spdlog::debug("Encountered an error while trying to get out-going EdgeAccessors. Transaction id: {}",
                      req.transaction_id.logical_id);
        return out_edges_result.GetError();
      }
      out_edges =
          maybe_filter_based_on_edge_uniquness(std::move(out_edges_result.GetValue()), msgs::EdgeDirection::OUT);
      break;
    }
  }
  return std::array<std::vector<EdgeAccessor>, 2>{in_edges, out_edges};
}

using AllEdgePropertyDataStructure = std::map<PropertyId, msgs::Value>;
using SpecificEdgePropertyDataStructure = std::vector<msgs::Value>;

using AllEdgeProperties = std::tuple<msgs::VertexId, msgs::Gid, AllEdgePropertyDataStructure>;
using SpecificEdgeProperties = std::tuple<msgs::VertexId, msgs::Gid, SpecificEdgePropertyDataStructure>;

using SpecificEdgePropertiesVector = std::vector<SpecificEdgeProperties>;
using AllEdgePropertiesVector = std::vector<AllEdgeProperties>;

template <bool are_in_edges>
ShardResult<void> FillEdges(const std::vector<EdgeAccessor> &edges, msgs::ExpandOneResultRow &row,
                            const EdgeFiller &edge_filler) {
  for (const auto &edge : edges) {
    if (const auto res = edge_filler(edge, are_in_edges, row); res.HasError()) {
      return res.GetError();
    }
  }
  return {};
}

ShardResult<msgs::ExpandOneResultRow> GetExpandOneResult(
    Shard::Accessor &acc, msgs::VertexId src_vertex, const msgs::ExpandOneRequest &req,
    const EdgeUniquenessFunction &maybe_filter_based_on_edge_uniquness, const EdgeFiller &edge_filler,
    const Schemas::Schema *schema) {
  /// Fill up source vertex
  const auto primary_key = ConvertPropertyVector(src_vertex.second);
  auto v_acc = acc.FindVertex(primary_key, View::NEW);

  msgs::Vertex source_vertex = {.id = src_vertex};
  auto maybe_secondary_labels = FillUpSourceVertexSecondaryLabels(v_acc, req);
  if (maybe_secondary_labels.HasError()) {
    return maybe_secondary_labels.GetError();
  }
  source_vertex.labels = std::move(*maybe_secondary_labels);

  auto src_vertex_properties = FillUpSourceVertexProperties(v_acc, req, storage::v3::View::NEW, schema);

  if (src_vertex_properties.HasError()) {
    return src_vertex_properties.GetError();
  }

  /// Fill up connecting edges
  auto fill_up_connecting_edges = FillUpConnectingEdges(v_acc, req, maybe_filter_based_on_edge_uniquness);
  if (fill_up_connecting_edges.HasError()) {
    return fill_up_connecting_edges.GetError();
  }

  auto [in_edges, out_edges] = fill_up_connecting_edges.GetValue();

  msgs::ExpandOneResultRow result_row;
  result_row.src_vertex = std::move(source_vertex);
  result_row.src_vertex_properties = std::move(*src_vertex_properties);
  static constexpr bool kInEdges = true;
  static constexpr bool kOutEdges = false;
  if (const auto fill_edges_res = FillEdges<kInEdges>(in_edges, result_row, edge_filler); fill_edges_res.HasError()) {
    return fill_edges_res.GetError();
  }
  if (const auto fill_edges_res = FillEdges<kOutEdges>(out_edges, result_row, edge_filler); fill_edges_res.HasError()) {
    return fill_edges_res.GetError();
  }

  return result_row;
}

EdgeUniquenessFunction InitializeEdgeUniquenessFunction(bool only_unique_neighbor_rows) {
  // Functions to select connecting edges based on uniquness
  EdgeUniquenessFunction maybe_filter_based_on_edge_uniquness;

  if (only_unique_neighbor_rows) {
    maybe_filter_based_on_edge_uniquness = [](EdgeAccessors &&edges,
                                              msgs::EdgeDirection edge_direction) -> EdgeAccessors {
      std::function<bool(std::set<const storage::v3::VertexId *, VertexIdCmpr> &, const storage::v3::EdgeAccessor &)>
          is_edge_unique;
      switch (edge_direction) {
        case msgs::EdgeDirection::OUT: {
          is_edge_unique = [](std::set<const storage::v3::VertexId *, VertexIdCmpr> &other_vertex_set,
                              const storage::v3::EdgeAccessor &edge_acc) {
            auto [it, insertion_happened] = other_vertex_set.insert(&edge_acc.ToVertex());
            return insertion_happened;
          };
          break;
        }
        case msgs::EdgeDirection::IN: {
          is_edge_unique = [](std::set<const storage::v3::VertexId *, VertexIdCmpr> &other_vertex_set,
                              const storage::v3::EdgeAccessor &edge_acc) {
            auto [it, insertion_happened] = other_vertex_set.insert(&edge_acc.FromVertex());
            return insertion_happened;
          };
          break;
        }
        case msgs::EdgeDirection::BOTH:
          MG_ASSERT(false, "This is should never happen, msgs::EdgeDirection::BOTH should not be passed here.");
      }

      EdgeAccessors ret;
      std::set<const storage::v3::VertexId *, VertexIdCmpr> other_vertex_set;

      for (const auto &edge : edges) {
        if (is_edge_unique(other_vertex_set, edge)) {
          ret.emplace_back(edge);
        }
      }

      return ret;
    };
  } else {
    maybe_filter_based_on_edge_uniquness =
        [](EdgeAccessors &&edges, msgs::EdgeDirection /*edge_direction*/) -> EdgeAccessors { return std::move(edges); };
  }

  return maybe_filter_based_on_edge_uniquness;
}

EdgeFiller InitializeEdgeFillerFunction(const msgs::ExpandOneRequest &req) {
  EdgeFiller edge_filler;

  if (!req.edge_properties) {
    edge_filler = [transaction_id = req.transaction_id.logical_id](
                      const EdgeAccessor &edge, const bool is_in_edge,
                      msgs::ExpandOneResultRow &result_row) -> ShardResult<void> {
      auto properties_results = edge.Properties(View::NEW);
      if (properties_results.HasError()) {
        spdlog::debug("Encountered an error while trying to get edge properties. Transaction id: {}", transaction_id);
        return properties_results.GetError();
      }

      std::map<PropertyId, msgs::Value> value_properties;
      for (auto &[prop_key, prop_val] : properties_results.GetValue()) {
        value_properties.insert(std::make_pair(prop_key, FromPropertyValueToValue(std::move(prop_val))));
      }
      using EdgeWithAllProperties = msgs::ExpandOneResultRow::EdgeWithAllProperties;
      EdgeWithAllProperties edges{ToMsgsVertexId(edge.FromVertex()), msgs::EdgeType{edge.EdgeType()},
                                  edge.Gid().AsUint(), std::move(value_properties)};
      if (is_in_edge) {
        result_row.in_edges_with_all_properties.push_back(std::move(edges));
      } else {
        result_row.out_edges_with_all_properties.push_back(std::move(edges));
      }
      return {};
    };
  } else {
    edge_filler = [&req](const EdgeAccessor &edge, const bool is_in_edge,
                         msgs::ExpandOneResultRow &result_row) -> ShardResult<void> {
      std::vector<msgs::Value> value_properties;
      value_properties.reserve(req.edge_properties.value().size());
      for (const auto &edge_prop : req.edge_properties.value()) {
        auto property_result = edge.GetProperty(edge_prop, View::NEW);
        if (property_result.HasError()) {
          spdlog::debug("Encountered an error while trying to get edge properties. Transaction id: {}",
                        req.transaction_id.logical_id);
          return property_result.GetError();
        }
        value_properties.emplace_back(FromPropertyValueToValue(std::move(property_result.GetValue())));
      }
      using EdgeWithSpecificProperties = msgs::ExpandOneResultRow::EdgeWithSpecificProperties;
      EdgeWithSpecificProperties edges{ToMsgsVertexId(edge.FromVertex()), msgs::EdgeType{edge.EdgeType()},
                                       edge.Gid().AsUint(), std::move(value_properties)};
      if (is_in_edge) {
        result_row.in_edges_with_specific_properties.push_back(std::move(edges));
      } else {
        result_row.out_edges_with_specific_properties.push_back(std::move(edges));
      }
      return {};
    };
  }

  return edge_filler;
}

auto CreateErrorResponse(const ShardError &shard_error, const auto transaction_id, const std::string_view action) {
  msgs::ShardError message_shard_error{shard_error.code, shard_error.message};
  spdlog::debug("{} In transaction {} {} failed: {}: {}", shard_error.source, transaction_id.logical_id, action,
                ErrorCodeToString(shard_error.code), shard_error.message);
  return message_shard_error;
}

};  // namespace
msgs::WriteResponses ShardRsm::ApplyWrite(msgs::CreateVerticesRequest &&req) {
  auto acc = shard_->Access(req.transaction_id);

  std::optional<msgs::ShardError> shard_error;

  for (auto &new_vertex : req.new_vertices) {
    /// TODO(gvolfing) Consider other methods than converting. Change either
    /// the way that the property map is stored in the messages, or the
    /// signature of CreateVertexAndValidate.
    auto converted_property_map = ConvertPropertyMap(std::move(new_vertex.properties));

    // TODO(gvolfing) make sure if this conversion is actually needed.
    std::vector<LabelId> converted_label_ids;
    converted_label_ids.reserve(new_vertex.label_ids.size());

    std::transform(new_vertex.label_ids.begin(), new_vertex.label_ids.end(), std::back_inserter(converted_label_ids),
                   [](const auto &label_id) { return label_id.id; });

    PrimaryKey transformed_pk;
    std::transform(new_vertex.primary_key.begin(), new_vertex.primary_key.end(), std::back_inserter(transformed_pk),
                   [](msgs::Value &val) { return ToPropertyValue(std::move(val)); });
    auto result_schema = acc.CreateVertexAndValidate(converted_label_ids, transformed_pk, converted_property_map);

    if (result_schema.HasError()) {
      shard_error.emplace(CreateErrorResponse(result_schema.GetError(), req.transaction_id, "creating vertices"));
      break;
    }
  }

  return msgs::CreateVerticesResponse{std::move(shard_error)};
}

msgs::WriteResponses ShardRsm::ApplyWrite(msgs::UpdateVerticesRequest &&req) {
  auto acc = shard_->Access(req.transaction_id);

  std::optional<msgs::ShardError> shard_error;
  for (auto &vertex : req.update_vertices) {
    auto vertex_to_update = acc.FindVertex(ConvertPropertyVector(std::move(vertex.primary_key)), View::OLD);
    if (!vertex_to_update) {
      shard_error.emplace(msgs::ShardError{common::ErrorCode::OBJECT_NOT_FOUND});
      spdlog::debug("In transaction {} vertex could not be found while trying to update its properties.",
                    req.transaction_id.logical_id);
      break;
    }

    for (const auto label : vertex.add_labels) {
      if (const auto maybe_error = vertex_to_update->AddLabelAndValidate(label); maybe_error.HasError()) {
        shard_error.emplace(CreateErrorResponse(maybe_error.GetError(), req.transaction_id, "adding label"));
        break;
      }
    }
    for (const auto label : vertex.remove_labels) {
      if (const auto maybe_error = vertex_to_update->RemoveLabelAndValidate(label); maybe_error.HasError()) {
        shard_error.emplace(CreateErrorResponse(maybe_error.GetError(), req.transaction_id, "adding label"));
        break;
      }
    }

    for (auto &update_prop : vertex.property_updates) {
      if (const auto result_schema = vertex_to_update->SetPropertyAndValidate(
              update_prop.first, ToPropertyValue(std::move(update_prop.second)));
          result_schema.HasError()) {
        shard_error.emplace(CreateErrorResponse(result_schema.GetError(), req.transaction_id, "adding label"));
        break;
      }
    }
  }

  return msgs::UpdateVerticesResponse{std::move(shard_error)};
}

msgs::WriteResponses ShardRsm::ApplyWrite(msgs::DeleteVerticesRequest &&req) {
  std::optional<msgs::ShardError> shard_error;
  auto acc = shard_->Access(req.transaction_id);

  for (auto &propval : req.primary_keys) {
    auto vertex_acc = acc.FindVertex(ConvertPropertyVector(std::move(propval)), View::OLD);

    if (!vertex_acc) {
      shard_error.emplace(msgs::ShardError{common::ErrorCode::OBJECT_NOT_FOUND});
      spdlog::debug("In transaction {} vertex could not be found while trying to delete it.",
                    req.transaction_id.logical_id);
      break;
    }
    // TODO(gvolfing)
    // Since we will not have different kinds of deletion types in one transaction,
    // we dont have to enter the switch statement on every iteration. Optimize this.
    switch (req.deletion_type) {
      case msgs::DeleteVerticesRequest::DeletionType::DELETE: {
        auto result = acc.DeleteVertex(&vertex_acc.value());
        if (result.HasError() || !(result.GetValue().has_value())) {
          shard_error.emplace(CreateErrorResponse(result.GetError(), req.transaction_id, "deleting vertices"));
        }
        break;
      }
      case msgs::DeleteVerticesRequest::DeletionType::DETACH_DELETE: {
        auto result = acc.DetachDeleteVertex(&vertex_acc.value());
        if (result.HasError() || !(result.GetValue().has_value())) {
          shard_error.emplace(CreateErrorResponse(result.GetError(), req.transaction_id, "deleting vertices"));
        }
        break;
      }
    }
    if (shard_error) {
      break;
    }
  }

  return msgs::DeleteVerticesResponse{std::move(shard_error)};
}

msgs::WriteResponses ShardRsm::ApplyWrite(msgs::CreateExpandRequest &&req) {
  auto acc = shard_->Access(req.transaction_id);
  std::optional<msgs::ShardError> shard_error;

  for (auto &new_expand : req.new_expands) {
    const auto from_vertex_id =
        v3::VertexId{new_expand.src_vertex.first.id, ConvertPropertyVector(std::move(new_expand.src_vertex.second))};

    const auto to_vertex_id =
        VertexId{new_expand.dest_vertex.first.id, ConvertPropertyVector(std::move(new_expand.dest_vertex.second))};

    if (!(shard_->IsVertexBelongToShard(from_vertex_id) || shard_->IsVertexBelongToShard(to_vertex_id))) {
      shard_error = msgs::ShardError{common::ErrorCode::OBJECT_NOT_FOUND,
                                     "Error while trying to insert edge, none of the vertices belong to this shard"};
      spdlog::debug("Error while trying to insert edge, none of the vertices belong to this shard. Transaction id: {}",
                    req.transaction_id.logical_id);
      break;
    }

    auto edge_acc = acc.CreateEdge(from_vertex_id, to_vertex_id, new_expand.type.id, Gid::FromUint(new_expand.id.gid));
    if (edge_acc.HasValue()) {
      auto edge = edge_acc.GetValue();
      if (!new_expand.properties.empty()) {
        for (const auto &[property, value] : new_expand.properties) {
          if (const auto maybe_error = edge.SetProperty(property, ToPropertyValue(value)); maybe_error.HasError()) {
            shard_error.emplace(
                CreateErrorResponse(maybe_error.GetError(), req.transaction_id, "setting edge property"));
            break;
          }
        }
        if (shard_error) {
          break;
        }
      }
    } else {
      // TODO Code for this
      shard_error = msgs::ShardError{common::ErrorCode::OBJECT_NOT_FOUND};
      spdlog::debug("Creating edge was not successful. Transaction id: {}", req.transaction_id.logical_id);
      break;
    }

    // Add properties to the edge if there is any
    if (!new_expand.properties.empty()) {
      for (auto &[edge_prop_key, edge_prop_val] : new_expand.properties) {
        auto set_result = edge_acc->SetProperty(edge_prop_key, ToPropertyValue(std::move(edge_prop_val)));
        if (set_result.HasError()) {
          shard_error.emplace(CreateErrorResponse(set_result.GetError(), req.transaction_id, "adding edge property"));
          break;
        }
      }
    }
  }

  return msgs::CreateExpandResponse{std::move(shard_error)};
}

msgs::WriteResponses ShardRsm::ApplyWrite(msgs::DeleteEdgesRequest &&req) {
  std::optional<msgs::ShardError> shard_error;
  auto acc = shard_->Access(req.transaction_id);

  for (auto &edge : req.edges) {
    if (shard_error) {
      break;
    }

    auto edge_acc = acc.DeleteEdge(VertexId(edge.src.first.id, ConvertPropertyVector(std::move(edge.src.second))),
                                   VertexId(edge.dst.first.id, ConvertPropertyVector(std::move(edge.dst.second))),
                                   Gid::FromUint(edge.id.gid));
    if (edge_acc.HasError() || !edge_acc.HasValue()) {
      shard_error.emplace(CreateErrorResponse(edge_acc.GetError(), req.transaction_id, "delete edge"));
      continue;
    }
  }

  return msgs::DeleteEdgesResponse{std::move(shard_error)};
}

msgs::WriteResponses ShardRsm::ApplyWrite(msgs::UpdateEdgesRequest &&req) {
  // TODO(antaljanosbenjamin): handle when the vertex is the destination vertex
  auto acc = shard_->Access(req.transaction_id);

  std::optional<msgs::ShardError> shard_error;

  for (auto &edge : req.new_properties) {
    if (shard_error) {
      break;
    }

    auto vertex_acc = acc.FindVertex(ConvertPropertyVector(std::move(edge.src.second)), View::OLD);
    if (!vertex_acc) {
      shard_error = msgs::ShardError{common::ErrorCode::OBJECT_NOT_FOUND, "Source vertex was not found"};
      spdlog::debug("Encountered an error while trying to acquire VertexAccessor with transaction id: {}",
                    req.transaction_id.logical_id);
      continue;
    }

    // Since we are using the source vertex of the edge we are only interested
    // in the vertex's out-going edges
    auto edges_res = vertex_acc->OutEdges(View::OLD);
    if (edges_res.HasError()) {
      shard_error.emplace(CreateErrorResponse(edges_res.GetError(), req.transaction_id, "update edge"));
      continue;
    }

    auto &edge_accessors = edges_res.GetValue();

    // Look for the appropriate edge accessor
    bool edge_accessor_did_match = false;
    for (auto &edge_accessor : edge_accessors) {
      if (edge_accessor.Gid().AsUint() == edge.edge_id.gid) {  // Found the appropriate accessor
        edge_accessor_did_match = true;
        for (auto &[key, value] : edge.property_updates) {
          // TODO(gvolfing)
          // Check if the property was set if SetProperty does not do that itself.
          auto res = edge_accessor.SetProperty(key, ToPropertyValue(std::move(value)));
          if (res.HasError()) {
            // TODO(jbajic) why not set action unsuccessful here?
            shard_error.emplace(CreateErrorResponse(edges_res.GetError(), req.transaction_id, "update edge"));
          }
        }
      }
    }

    if (!edge_accessor_did_match) {
      // TODO(jbajic) Do we need this
      shard_error = msgs::ShardError{common::ErrorCode::OBJECT_NOT_FOUND, "Edge was not found"};
      spdlog::debug("Could not find the Edge with the specified Gid. Transaction id: {}",
                    req.transaction_id.logical_id);
      continue;
    }
  }

  return msgs::UpdateEdgesResponse{std::move(shard_error)};
}

msgs::ReadResponses ShardRsm::HandleRead(msgs::ScanVerticesRequest &&req) {
  auto acc = shard_->Access(req.transaction_id);
  std::optional<msgs::ShardError> shard_error;

  std::vector<msgs::ScanResultRow> results;
  if (req.batch_limit) {
    results.reserve(*req.batch_limit);
  }
  std::optional<msgs::VertexId> next_start_id;

  const auto view = View(req.storage_view);
  auto dba = DbAccessor{&acc};
  const auto emplace_scan_result = [&](const VertexAccessor &vertex) {
    std::vector<Value> expression_results;
    if (!req.filter_expressions.empty()) {
      // NOTE - DbAccessor might get removed in the future.
      const bool eval = FilterOnVertex(dba, vertex, req.filter_expressions, expr::identifier_node_symbol);
      if (!eval) {
        return;
      }
    }
    if (!req.vertex_expressions.empty()) {
      // NOTE - DbAccessor might get removed in the future.
      expression_results = ConvertToValueVectorFromTypedValueVector(
          EvaluateVertexExpressions(dba, vertex, req.vertex_expressions, expr::identifier_node_symbol));
    }

    std::optional<std::map<PropertyId, Value>> found_props;

    if (req.props_to_return) {
      found_props = CollectSpecificPropertiesFromAccessor(vertex, req.props_to_return.value(), view);
    } else {
      const auto *schema = shard_->GetSchema(shard_->PrimaryLabel());
      found_props = CollectAllPropertiesFromAccessor(vertex, view, schema);
    }

    // TODO(gvolfing) -VERIFY-
    // Vertex is separated from the properties in the response.
    // Is it useful to return just a vertex without the properties?
    if (!found_props) {
      shard_error = msgs::ShardError{common::ErrorCode::OBJECT_NOT_FOUND, "Requested properties were not found!"};
    }

    results.emplace_back(msgs::ScanResultRow{.vertex = ConstructValueVertex(vertex, view).vertex_v,
                                             .props = FromMap(found_props.value()),
                                             .evaluated_vertex_expressions = std::move(expression_results)});
  };

  const auto start_id = ConvertPropertyVector(std::move(req.start_id.second));
  uint64_t sample_counter{0};
  auto vertex_iterable = acc.Vertices(view);
  if (!req.order_bys.empty()) {
    const auto ordered = OrderByElements(acc, dba, vertex_iterable, req.order_bys);
    // we are traversing Elements
    auto it = GetStartOrderedElementsIterator(ordered, start_id, View(req.storage_view));
    for (; it != ordered.end(); ++it) {
      emplace_scan_result(it->vertex_acc);
      ++sample_counter;
      if (req.batch_limit && sample_counter == req.batch_limit) {
        // Reached the maximum specified batch size.
        // Get the next element before exiting.
        ++it;
        if (it != ordered.end()) {
          const auto &next_vertex = it->vertex_acc;
          next_start_id = ConstructValueVertex(next_vertex, view).vertex_v.id;
        }

        break;
      }
    }
  } else {
    // We are going through VerticesIterable::Iterator
    auto it = GetStartVertexIterator(vertex_iterable, start_id, View(req.storage_view));
    for (; it != vertex_iterable.end(); ++it) {
      emplace_scan_result(*it);

      ++sample_counter;
      if (req.batch_limit && sample_counter == req.batch_limit) {
        // Reached the maximum specified batch size.
        // Get the next element before exiting.
        const auto &next_vertex = *(++it);
        next_start_id = ConstructValueVertex(next_vertex, view).vertex_v.id;

        break;
      }
    }
  }

  msgs::ScanVerticesResponse resp{.error = std::move(shard_error)};
  if (!resp.error) {
    resp.next_start_id = next_start_id;
    resp.results = std::move(results);
  }

  return resp;
}

msgs::ReadResponses ShardRsm::HandleRead(msgs::ExpandOneRequest &&req) {
  auto acc = shard_->Access(req.transaction_id);
  std::optional<msgs::ShardError> shard_error;

  std::vector<msgs::ExpandOneResultRow> results;

  auto maybe_filter_based_on_edge_uniquness = InitializeEdgeUniquenessFunction(req.only_unique_neighbor_rows);
  auto edge_filler = InitializeEdgeFillerFunction(req);

  for (auto &src_vertex : req.src_vertices) {
    // Get Vertex acc
    auto src_vertex_acc_opt = acc.FindVertex(ConvertPropertyVector((src_vertex.second)), View::NEW);
    if (!src_vertex_acc_opt) {
      shard_error = msgs::ShardError{common::ErrorCode::OBJECT_NOT_FOUND, "Source vertex was not found."};
      spdlog::debug("Encountered an error while trying to obtain VertexAccessor. Transaction id: {}",
                    req.transaction_id.logical_id);
      break;
    }

    if (!req.filters.empty()) {
      // NOTE - DbAccessor might get removed in the future.
      auto dba = DbAccessor{&acc};
      const bool eval = FilterOnVertex(dba, src_vertex_acc_opt.value(), req.filters, expr::identifier_node_symbol);
      if (!eval) {
        continue;
      }
    }
    const auto result = GetExpandOneResult(acc, src_vertex, req, maybe_filter_based_on_edge_uniquness, edge_filler,
                                           shard_->GetSchema(shard_->PrimaryLabel()));

    if (result.HasError()) {
      // Code Error
      shard_error = msgs::ShardError{common::ErrorCode::OBJECT_NOT_FOUND, "Source vertex was not found."};
      break;
    }

    results.emplace_back(result.GetValue());
  }

  msgs::ExpandOneResponse resp{.error = std::move(shard_error)};
  if (!resp.error) {
    resp.result = std::move(results);
  }

  return resp;
}

msgs::WriteResponses ShardRsm::ApplyWrite(msgs::CommitRequest &&req) {
  shard_->Access(req.transaction_id).Commit(req.commit_timestamp);
  return msgs::CommitResponse{};
};

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
msgs::ReadResponses ShardRsm::HandleRead(msgs::GetPropertiesRequest && /*req*/) {
  return msgs::GetPropertiesResponse{};
}

}  // namespace memgraph::storage::v3

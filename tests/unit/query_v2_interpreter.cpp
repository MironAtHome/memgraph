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
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <unordered_set>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "communication/bolt/v1/value.hpp"
#include "glue/v2/communication.hpp"
#include "query/v2/auth_checker.hpp"
#include "query/v2/bindings/typed_value.hpp"
#include "query/v2/config.hpp"
#include "query/v2/exceptions.hpp"
#include "query/v2/interpreter.hpp"
#include "query/v2/stream.hpp"
#include "query_v2_query_common.hpp"
#include "result_stream_faker.hpp"
#include "storage/v3/isolation_level.hpp"
#include "storage/v3/key_store.hpp"
#include "storage/v3/name_id_mapper.hpp"
#include "storage/v3/property_value.hpp"
#include "utils/csv_parsing.hpp"
#include "utils/logging.hpp"

namespace memgraph::query::v2::tests {

auto ToEdgeList(const memgraph::communication::bolt::Value &v) {
  std::vector<memgraph::communication::bolt::Edge> list;
  for (auto x : v.ValueList()) {
    list.push_back(x.ValueEdge());
  }
  return list;
}

auto StringToUnorderedSet(const std::string &element) {
  const auto element_split = memgraph::utils::Split(element, ", ");
  return std::unordered_set<std::string>(element_split.begin(), element_split.end());
};

struct InterpreterFaker {
  InterpreterFaker(memgraph::storage::v3::Shard *db, const memgraph::query::v2::InterpreterConfig config,
                   const std::filesystem::path &data_directory)
      : interpreter_context(db, config, data_directory), interpreter(&interpreter_context) {
    interpreter_context.auth_checker = &auth_checker;
  }

  auto Prepare(const std::string &query,
               const std::map<std::string, memgraph::storage::v3::PropertyValue> &params = {}) {
    ResultStreamFaker stream(interpreter_context.db);

    const auto [header, _, qid] = interpreter.Prepare(query, params, nullptr);
    stream.Header(header);
    return std::make_pair(std::move(stream), qid);
  }

  void Pull(ResultStreamFaker *stream, std::optional<int> n = {}, std::optional<int> qid = {}) {
    const auto summary = interpreter.Pull(stream, n, qid);
    stream->Summary(summary);
  }

  /**
   * Execute the given query and commit the transaction.
   *
   * Return the query stream.
   */
  auto Interpret(const std::string &query,
                 const std::map<std::string, memgraph::storage::v3::PropertyValue> &params = {}) {
    auto prepare_result = Prepare(query, params);

    auto &stream = prepare_result.first;
    auto summary = interpreter.Pull(&stream, {}, prepare_result.second);
    stream.Summary(summary);

    return std::move(stream);
  }

  memgraph::query::v2::AllowEverythingAuthChecker auth_checker;
  memgraph::query::v2::InterpreterContext interpreter_context;
  memgraph::query::v2::Interpreter interpreter;
};

// TODO: This is not a unit test, but tests/integration dir is chaotic at the
// moment. After tests refactoring is done, move/rename this.

class InterpreterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(db_.CreateSchema(label, {storage::v3::SchemaProperty{property, common::SchemaType::INT}}));
  }

  auto Prepare(const std::string &query,
               const std::map<std::string, memgraph::storage::v3::PropertyValue> &params = {}) {
    return default_interpreter.Prepare(query, params);
  }

  void Pull(ResultStreamFaker *stream, std::optional<int> n = {}, std::optional<int> qid = {}) {
    default_interpreter.Pull(stream, n, qid);
  }

  auto Interpret(const std::string &query,
                 const std::map<std::string, memgraph::storage::v3::PropertyValue> &params = {}) {
    return default_interpreter.Interpret(query, params);
  }

  storage::v3::LabelId NameToLabelId(std::string_view label_name) {
    return storage::v3::LabelId::FromUint(id_mapper.NameToId(label_name));
  }

  storage::v3::PropertyId NameToPropertyId(std::string_view property_name) {
    return storage::v3::PropertyId::FromUint(id_mapper.NameToId(property_name));
  }

  storage::v3::PrimaryKey pk{storage::v3::PropertyValue(0)};
  memgraph::storage::v3::NameIdMapper id_mapper;
  const storage::v3::LabelId label{NameToLabelId("label")};
  storage::v3::Shard db_{label, pk, std::nullopt};
  std::filesystem::path data_directory{std::filesystem::temp_directory_path() / "MG_tests_unit_query_v2_interpreter"};
  const storage::v3::PropertyId property{NameToPropertyId("property")};
  InterpreterFaker default_interpreter{&db_, {}, data_directory};
};

TEST_F(InterpreterTest, DummyTestToForceQueryV2Compilation) {
  ASSERT_TRUE(true) << "This test is only here to make sure mg-query-v2 gets compiled when building unit tests so "
                       "clang-tidy can analyze it properly.";
}

// TEST_F(InterpreterTest, MultiplePulls) {
//   {
//     auto [stream, qid] = Prepare("UNWIND [1,2,3,4,5] as n RETURN n");
//     ASSERT_EQ(stream.GetHeader().size(), 1U);
//     EXPECT_EQ(stream.GetHeader()[0], "n");
//     Pull(&stream, 1);
//     ASSERT_EQ(stream.GetSummary().count("has_more"), 1);
//     ASSERT_TRUE(stream.GetSummary().at("has_more").ValueBool());
//     ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0][0].ValueInt(), 1);
//     Pull(&stream, 2);
//     ASSERT_EQ(stream.GetSummary().count("has_more"), 1);
//     ASSERT_TRUE(stream.GetSummary().at("has_more").ValueBool());
//     ASSERT_EQ(stream.GetResults().size(), 3U);
//     ASSERT_EQ(stream.GetResults()[1][0].ValueInt(), 2);
//     ASSERT_EQ(stream.GetResults()[2][0].ValueInt(), 3);
//     Pull(&stream);
//     ASSERT_EQ(stream.GetSummary().count("has_more"), 1);
//     ASSERT_FALSE(stream.GetSummary().at("has_more").ValueBool());
//     ASSERT_EQ(stream.GetResults().size(), 5U);
//     ASSERT_EQ(stream.GetResults()[3][0].ValueInt(), 4);
//     ASSERT_EQ(stream.GetResults()[4][0].ValueInt(), 5);
//   }
// }

// Run query with different ast twice to see if query executes correctly when
// ast is read from cache.
// TEST_F(InterpreterTest, AstCache) {
//   {
//     auto stream = Interpret("RETURN 2 + 3");
//     ASSERT_EQ(stream.GetHeader().size(), 1U);
//     EXPECT_EQ(stream.GetHeader()[0], "2 + 3");
//     ASSERT_EQ(stream.GetResults().size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0][0].ValueInt(), 5);
//   }
//   {
//     // Cached ast, different literals.
//     auto stream = Interpret("RETURN 5 + 4");
//     ASSERT_EQ(stream.GetResults().size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0][0].ValueInt(), 9);
//   }
//   {
//     // Different ast (because of different types).
//     auto stream = Interpret("RETURN 5.5 + 4");
//     ASSERT_EQ(stream.GetResults().size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0][0].ValueDouble(), 9.5);
//   }
//   {
//     // Cached ast, same literals.
//     auto stream = Interpret("RETURN 2 + 3");
//     ASSERT_EQ(stream.GetResults().size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0][0].ValueInt(), 5);
//   }
//   {
//     // Cached ast, different literals.
//     auto stream = Interpret("RETURN 10.5 + 1");
//     ASSERT_EQ(stream.GetResults().size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0][0].ValueDouble(), 11.5);
//   }
//   {
//     // Cached ast, same literals, different whitespaces.
//     auto stream = Interpret("RETURN  10.5 + 1");
//     ASSERT_EQ(stream.GetResults().size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0][0].ValueDouble(), 11.5);
//   }
//   {
//     // Cached ast, same literals, different named header.
//     auto stream = Interpret("RETURN  10.5+1");
//     ASSERT_EQ(stream.GetHeader().size(), 1U);
//     EXPECT_EQ(stream.GetHeader()[0], "10.5+1");
//     ASSERT_EQ(stream.GetResults().size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0][0].ValueDouble(), 11.5);
//   }
// }

// // Run query with same ast multiple times with different parameters.
// TEST_F(InterpreterTest, Parameters) {
//   {
//     auto stream = Interpret("RETURN $2 + $`a b`", {{"2", memgraph::storage::v3::PropertyValue(10)},
//                                                    {"a b", memgraph::storage::v3::PropertyValue(15)}});
//     ASSERT_EQ(stream.GetHeader().size(), 1U);
//     EXPECT_EQ(stream.GetHeader()[0], "$2 + $`a b`");
//     ASSERT_EQ(stream.GetResults().size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0][0].ValueInt(), 25);
//   }
//   {
//     // Not needed parameter.
//     auto stream = Interpret("RETURN $2 + $`a b`", {{"2", memgraph::storage::v3::PropertyValue(10)},
//                                                    {"a b", memgraph::storage::v3::PropertyValue(15)},
//                                                    {"c", memgraph::storage::v3::PropertyValue(10)}});
//     ASSERT_EQ(stream.GetHeader().size(), 1U);
//     EXPECT_EQ(stream.GetHeader()[0], "$2 + $`a b`");
//     ASSERT_EQ(stream.GetResults().size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0][0].ValueInt(), 25);
//   }
//   {
//     // Cached ast, different parameters.
//     auto stream = Interpret("RETURN $2 + $`a b`", {{"2", memgraph::storage::v3::PropertyValue("da")},
//                                                    {"a b", memgraph::storage::v3::PropertyValue("ne")}});
//     ASSERT_EQ(stream.GetResults().size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0][0].ValueString(), "dane");
//   }
//   {
//     // Non-primitive literal.
//     auto stream = Interpret(
//         "RETURN $2", {{"2", memgraph::storage::v3::PropertyValue(std::vector<memgraph::storage::v3::PropertyValue>{
//                                 memgraph::storage::v3::PropertyValue(5), memgraph::storage::v3::PropertyValue(2),
//                                 memgraph::storage::v3::PropertyValue(3)})}});
//     ASSERT_EQ(stream.GetResults().size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//     auto result =
//         memgraph::query::v2::test_common::ToIntList(memgraph::glue::v2::ToTypedValue(stream.GetResults()[0][0]));
//     ASSERT_THAT(result, testing::ElementsAre(5, 2, 3));
//   }
//   {
//     // Cached ast, unprovided parameter.
//     ASSERT_THROW(Interpret("RETURN $2 + $`a b`", {{"2", memgraph::storage::v3::PropertyValue("da")},
//                                                   {"ab", memgraph::storage::v3::PropertyValue("ne")}}),
//                  memgraph::query::v2::UnprovidedParameterError);
//   }
// }

// // Run CREATE/MATCH/MERGE queries with property map
// TEST_F(InterpreterTest, ParametersAsPropertyMap) {
//   {
//     std::map<std::string, memgraph::storage::v3::PropertyValue> property_map{};
//     property_map["name"] = memgraph::storage::v3::PropertyValue("name1");
//     property_map["age"] = memgraph::storage::v3::PropertyValue(25);
//     auto stream =
//         Interpret("CREATE (n:label $prop) RETURN n", {
//                                                          {"prop",
//                                                          memgraph::storage::v3::PropertyValue(property_map)},
//                                                      });
//     ASSERT_EQ(stream.GetHeader().size(), 1U);
//     ASSERT_EQ(stream.GetHeader()[0], "n");
//     ASSERT_EQ(stream.GetResults().size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//     auto result = stream.GetResults()[0][0].ValueVertex();
//     EXPECT_EQ(result.properties["name"].ValueString(), "name1");
//     EXPECT_EQ(result.properties["age"].ValueInt(), 25);
//   }
//   {
//     EXPECT_NO_THROW(Interpret("CREATE SCHEMA ON :Person(name STRING, age INTEGER)"));
//     std::map<std::string, memgraph::storage::v3::PropertyValue> property_map{};
//     property_map["name"] = memgraph::storage::v3::PropertyValue("name1");
//     property_map["age"] = memgraph::storage::v3::PropertyValue(25);
//     EXPECT_NO_THROW(Interpret("CREATE (:Person {name: 'test', age: 30})"));
//     auto stream = Interpret("MATCH (m:Person) CREATE (n:Person $prop) RETURN n",
//                             {
//                                 {"prop", memgraph::storage::v3::PropertyValue(property_map)},
//                             });
//     ASSERT_EQ(stream.GetHeader().size(), 1U);
//     ASSERT_EQ(stream.GetHeader()[0], "n");
//     ASSERT_EQ(stream.GetResults().size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//     auto result = stream.GetResults()[0][0].ValueVertex();
//     EXPECT_EQ(result.properties["name"].ValueString(), "name1");
//     EXPECT_EQ(result.properties["age"].ValueInt(), 25);
//   }
//   {
//     EXPECT_NO_THROW(Interpret("CREATE SCHEMA ON :L1(name STRING)"));
//     std::map<std::string, memgraph::storage::v3::PropertyValue> property_map{};
//     property_map["name"] = memgraph::storage::v3::PropertyValue("name1");
//     property_map["weight"] = memgraph::storage::v3::PropertyValue(121);
//     auto stream = Interpret("CREATE (:L1 {name: 'name1'})-[r:TO $prop]->(:L1 {name: 'name2'}) RETURN r",
//                             {
//                                 {"prop", memgraph::storage::v3::PropertyValue(property_map)},
//                             });
//     ASSERT_EQ(stream.GetHeader().size(), 1U);
//     ASSERT_EQ(stream.GetHeader()[0], "r");
//     ASSERT_EQ(stream.GetResults().size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//     auto result = stream.GetResults()[0][0].ValueEdge();
//     EXPECT_EQ(result.properties["name"].ValueString(), "name1");
//     EXPECT_EQ(result.properties["weight"].ValueInt(), 121);
//   }
//   {
//     std::map<std::string, memgraph::storage::v3::PropertyValue> property_map{};
//     property_map["name"] = memgraph::storage::v3::PropertyValue("name1");
//     property_map["age"] = memgraph::storage::v3::PropertyValue(15);
//     ASSERT_THROW(Interpret("MATCH (n $prop) RETURN n",
//                            {
//                                {"prop", memgraph::storage::v3::PropertyValue(property_map)},
//                            }),
//                  memgraph::query::v2::SemanticException);
//   }
//   {
//     EXPECT_NO_THROW(Interpret("CREATE SCHEMA ON :L2(name STRING, age INTEGER)"));
//     std::map<std::string, memgraph::storage::v3::PropertyValue> property_map{};
//     property_map["name"] = memgraph::storage::v3::PropertyValue("name1");
//     property_map["age"] = memgraph::storage::v3::PropertyValue(15);
//     ASSERT_THROW(Interpret("MERGE (n:L2 $prop) RETURN n",
//                            {
//                                {"prop", memgraph::storage::v3::PropertyValue(property_map)},
//                            }),
//                  memgraph::query::v2::SemanticException);
//   }
// }

// // Test bfs end to end.
// TEST_F(InterpreterTest, Bfs) {
//   srand(0);
//   const auto kNumLevels = 10;
//   const auto kNumNodesPerLevel = 100;
//   const auto kNumEdgesPerNode = 100;
//   const auto kNumUnreachableNodes = 1000;
//   const auto kNumUnreachableEdges = 100000;
//   const auto kReachable = "reachable";
//   const auto kId = "id";

//   std::vector<std::vector<memgraph::query::v2::VertexAccessor>> levels(kNumLevels);
//   int id = 0;

//   // Set up.
//   {
//     auto storage_dba = db_.Access();
//     memgraph::query::v2::DbAccessor dba(&storage_dba);
//     auto add_node = [&](int level, bool reachable) {
//       auto maybe_node = dba.InsertVertexAndValidate(label, {}, {{property, storage::v3::PropertyValue(id)}});
//       MG_ASSERT(maybe_node.HasValue());
//       auto node = maybe_node.GetValue();
//       MG_ASSERT(
//           node.SetPropertyAndValidate(dba.NameToProperty(kId),
//           memgraph::storage::v3::PropertyValue(id++)).HasValue());
//       MG_ASSERT(
//           node.SetPropertyAndValidate(dba.NameToProperty(kReachable),
//           memgraph::storage::v3::PropertyValue(reachable))
//               .HasValue());
//       levels[level].push_back(node);
//       return node;
//     };

//     auto add_edge = [&](auto &v1, auto &v2, bool reachable) {
//       auto edge = dba.InsertEdge(&v1, &v2, dba.NameToEdgeType("edge"));
//       MG_ASSERT(edge->SetProperty(dba.NameToProperty(kReachable), memgraph::storage::v3::PropertyValue(reachable))
//                     .HasValue());
//     };

//     // Add source node.
//     add_node(0, true);

//     // Add reachable nodes.
//     for (int i = 1; i < kNumLevels; ++i) {
//       for (int j = 0; j < kNumNodesPerLevel; ++j) {
//         auto node = add_node(i, true);
//         for (int k = 0; k < kNumEdgesPerNode; ++k) {
//           auto &node2 = levels[i - 1][rand() % levels[i - 1].size()];
//           add_edge(node2, node, true);
//         }
//       }
//     }

//     // Add unreachable nodes.
//     for (int i = 0; i < kNumUnreachableNodes; ++i) {
//       auto node = add_node(rand() % kNumLevels,  // Not really important.
//                            false);
//       for (int j = 0; j < kNumEdgesPerNode; ++j) {
//         auto &level = levels[rand() % kNumLevels];
//         auto &node2 = level[rand() % level.size()];
//         add_edge(node2, node, true);
//         add_edge(node, node2, true);
//       }
//     }

//     // Add unreachable edges.
//     for (int i = 0; i < kNumUnreachableEdges; ++i) {
//       auto &level1 = levels[rand() % kNumLevels];
//       auto &node1 = level1[rand() % level1.size()];
//       auto &level2 = levels[rand() % kNumLevels];
//       auto &node2 = level2[rand() % level2.size()];
//       add_edge(node1, node2, false);
//     }

//     ASSERT_FALSE(dba.Commit().HasError());
//   }

//   auto stream = Interpret(
//       "MATCH (n {id: 0})-[r *bfs..5 (e, n | n.reachable and "
//       "e.reachable)]->(m) RETURN n, r, m");

//   ASSERT_EQ(stream.GetHeader().size(), 3U);
//   EXPECT_EQ(stream.GetHeader()[0], "n");
//   EXPECT_EQ(stream.GetHeader()[1], "r");
//   EXPECT_EQ(stream.GetHeader()[2], "m");
//   ASSERT_EQ(stream.GetResults().size(), 5 * kNumNodesPerLevel);

//   auto dba = db_.Access();
//   int expected_level = 1;
//   int remaining_nodes_in_level = kNumNodesPerLevel;
//   std::unordered_set<int64_t> matched_ids;

//   for (const auto &result : stream.GetResults()) {
//     const auto &begin = result[0].ValueVertex();
//     const auto &edges = ToEdgeList(result[1]);
//     const auto &end = result[2].ValueVertex();

//     // Check that path is of expected length. Returned paths should be from
//     // shorter to longer ones.
//     EXPECT_EQ(edges.size(), expected_level);
//     // Check that starting node is correct.
//     EXPECT_EQ(edges.front().from, begin.id);
//     EXPECT_EQ(begin.properties.at(kId).ValueInt(), 0);
//     for (int i = 1; i < static_cast<int>(edges.size()); ++i) {
//       // Check that edges form a connected path.
//       EXPECT_EQ(edges[i - 1].to.AsInt(), edges[i].from.AsInt());
//     }
//     auto matched_id = end.properties.at(kId).ValueInt();
//     EXPECT_EQ(edges.back().to, end.id);
//     // Check that we didn't match that node already.
//     EXPECT_TRUE(matched_ids.insert(matched_id).second);
//     // Check that shortest path was found.
//     EXPECT_TRUE(matched_id > kNumNodesPerLevel * (expected_level - 1) &&
//                 matched_id <= kNumNodesPerLevel * expected_level);
//     if (!--remaining_nodes_in_level) {
//       remaining_nodes_in_level = kNumNodesPerLevel;
//       ++expected_level;
//     }
//   }
// }

// // Test shortest path end to end.
// TEST_F(InterpreterTest, ShortestPath) {
//   EXPECT_NO_THROW(Interpret("CREATE SCHEMA ON :A(x INTEGER)"));
//   EXPECT_NO_THROW(Interpret("CREATE SCHEMA ON :B(x INTEGER)"));
//   EXPECT_NO_THROW(Interpret("CREATE SCHEMA ON :C(x INTEGER)"));
//   const auto test_shortest_path = [this](const bool use_duration) {
//     const auto get_weight = [use_duration](const auto value) {
//       return fmt::format(fmt::runtime(use_duration ? "DURATION('PT{}S')" : "{}"), value);
//     };

//     Interpret(
//         fmt::format("CREATE (n:A {{x: 1}}), (m:B {{x: 2}}), (l:C {{x: 1}}), (n)-[:r1 {{w: {} "
//                     "}}]->(m)-[:r2 {{w: {}}}]->(l), (n)-[:r3 {{w: {}}}]->(l)",
//                     get_weight(1), get_weight(2), get_weight(4)));

//     auto stream = Interpret("MATCH (n)-[e *wshortest 5 (e, n | e.w) ]->(m) return e");

//     ASSERT_EQ(stream.GetHeader().size(), 1U);
//     EXPECT_EQ(stream.GetHeader()[0], "e");
//     ASSERT_EQ(stream.GetResults().size(), 3U);

//     auto dba = db_.Access();
//     std::vector<std::vector<std::string>> expected_results{{"r1"}, {"r2"}, {"r1", "r2"}};

//     for (const auto &result : stream.GetResults()) {
//       const auto &edges = ToEdgeList(result[0]);

//       std::vector<std::string> datum;
//       datum.reserve(edges.size());

//       for (const auto &edge : edges) {
//         datum.push_back(edge.type);
//       }

//       bool any_match = false;
//       for (const auto &expected : expected_results) {
//         if (expected == datum) {
//           any_match = true;
//           break;
//         }
//       }

//       EXPECT_TRUE(any_match);
//     }

//     Interpret("MATCH (n) DETACH DELETE n");
//   };

//   static constexpr bool kUseNumeric{false};
//   static constexpr bool kUseDuration{true};
//   {
//     SCOPED_TRACE("Test with numeric values");
//     test_shortest_path(kUseNumeric);
//   }
//   {
//     SCOPED_TRACE("Test with Duration values");
//     test_shortest_path(kUseDuration);
//   }
// }

// TEST_F(InterpreterTest, CreateLabelIndexInMulticommandTransaction) {
//   Interpret("BEGIN");
//   ASSERT_THROW(Interpret("CREATE INDEX ON :X"), memgraph::query::v2::IndexInMulticommandTxException);
//   Interpret("ROLLBACK");
// }

// TEST_F(InterpreterTest, CreateLabelPropertyIndexInMulticommandTransaction) {
//   Interpret("BEGIN");
//   ASSERT_THROW(Interpret("CREATE INDEX ON :X(y)"), memgraph::query::v2::IndexInMulticommandTxException);
//   Interpret("ROLLBACK");
// }

// TEST_F(InterpreterTest, CreateExistenceConstraintInMulticommandTransaction) {
//   Interpret("BEGIN");
//   ASSERT_THROW(Interpret("CREATE CONSTRAINT ON (n:A) ASSERT EXISTS (n.a)"),
//                memgraph::query::v2::ConstraintInMulticommandTxException);
//   Interpret("ROLLBACK");
// }

// TEST_F(InterpreterTest, CreateUniqueConstraintInMulticommandTransaction) {
//   Interpret("BEGIN");
//   ASSERT_THROW(Interpret("CREATE CONSTRAINT ON (n:A) ASSERT n.a, n.b IS UNIQUE"),
//                memgraph::query::v2::ConstraintInMulticommandTxException);
//   Interpret("ROLLBACK");
// }

// TEST_F(InterpreterTest, ShowIndexInfoInMulticommandTransaction) {
//   Interpret("BEGIN");
//   ASSERT_THROW(Interpret("SHOW INDEX INFO"), memgraph::query::v2::InfoInMulticommandTxException);
//   Interpret("ROLLBACK");
// }

// TEST_F(InterpreterTest, ShowConstraintInfoInMulticommandTransaction) {
//   Interpret("BEGIN");
//   ASSERT_THROW(Interpret("SHOW CONSTRAINT INFO"), memgraph::query::v2::InfoInMulticommandTxException);
//   Interpret("ROLLBACK");
// }

// TEST_F(InterpreterTest, ShowStorageInfoInMulticommandTransaction) {
//   Interpret("BEGIN");
//   ASSERT_THROW(Interpret("SHOW STORAGE INFO"), memgraph::query::v2::InfoInMulticommandTxException);
//   Interpret("ROLLBACK");
// }

// // // NOLINTNEXTLINE(hicpp-special-member-functions)
// TEST_F(InterpreterTest, ExistenceConstraintTest) {
//   ASSERT_NO_THROW(Interpret("CREATE SCHEMA ON :A(a INTEGER);"));

//   Interpret("CREATE CONSTRAINT ON (n:A) ASSERT EXISTS (n.b);");
//   Interpret("CREATE (:A{a: 3, b:1})");
//   Interpret("CREATE (:A{a: 3, b:2})");
//   ASSERT_THROW(Interpret("CREATE (:A {a: 12})"), memgraph::query::v2::QueryException);
//   Interpret("MATCH (n:A{a:3, b: 2}) SET n.b=5");
//   Interpret("CREATE (:A{a:2, b: 3})");
//   Interpret("MATCH (n:A{a:3, b: 1}) DETACH DELETE n");
//   Interpret("CREATE (n:A{a:2, b: 3})");
//   ASSERT_THROW(Interpret("CREATE CONSTRAINT ON (n:A) ASSERT EXISTS (n.c);"),
//                memgraph::query::v2::QueryRuntimeException);
// }

// TEST_F(InterpreterTest, UniqueConstraintTest) {
//   ASSERT_NO_THROW(Interpret("CREATE SCHEMA ON :A(a INTEGER);"));

//   // Empty property list should result with syntax exception.
//   ASSERT_THROW(Interpret("CREATE CONSTRAINT ON (n:A) ASSERT IS UNIQUE;"),
//  memgraph::frontend::opencypher::SyntaxException);
//   ASSERT_THROW(Interpret("DROP CONSTRAINT ON (n:A) ASSERT IS UNIQUE;"),
//  memgraph::frontend::opencypher::SyntaxException);

//   // Too large list of properties should also result with syntax exception.
//   {
//     std::stringstream stream;
//     stream << " ON (n:A) ASSERT ";
//     for (size_t i = 0; i < 33; ++i) {
//       if (i > 0) stream << ", ";
//       stream << "n.prop" << i;
//     }
//     stream << " IS UNIQUE;";
//     std::string create_query = "CREATE CONSTRAINT" + stream.str();
//     std::string drop_query = "DROP CONSTRAINT" + stream.str();
//     ASSERT_THROW(Interpret(create_query), memgraph::query::v2::SyntaxException);
//     ASSERT_THROW(Interpret(drop_query), memgraph::query::v2::SyntaxException);
//   }

//   // Providing property list with duplicates results with syntax exception.
//   ASSERT_THROW(Interpret("CREATE CONSTRAINT ON (n:A) ASSERT n.a, n.b, n.a IS UNIQUE;"),
//                memgraph::query::v2::SyntaxException);
//   ASSERT_THROW(Interpret("DROP CONSTRAINT ON (n:A) ASSERT n.a, n.b, n.a IS UNIQUE;"),
//                memgraph::query::v2::SyntaxException);

//   // Commit of vertex should fail if a constraint is violated.
//   Interpret("CREATE CONSTRAINT ON (n:A) ASSERT n.a, n.b IS UNIQUE;");
//   Interpret("CREATE (:A{a:1, b:2})");
//   Interpret("CREATE (:A{a:1, b:3})");
//   ASSERT_THROW(Interpret("CREATE (:A{a:1, b:2})"), memgraph::query::v2::QueryException);

//   // Attempt to create a constraint should fail if it's violated.
//   Interpret("CREATE (:A{a:1, c:2})");
//   Interpret("CREATE (:A{a:1, c:2})");
//   ASSERT_THROW(Interpret("CREATE CONSTRAINT ON (n:A) ASSERT n.a, n.c IS UNIQUE;"),
//                memgraph::query::v2::QueryRuntimeException);

//   Interpret("MATCH (n:A{a:2, b:2}) SET n.a=1");
//   Interpret("CREATE (:A{a:2})");
//   Interpret("MATCH (n:A{a:2}) DETACH DELETE n");
//   Interpret("CREATE (n:A{a:2})");

//   // Show constraint info.
//   {
//     auto stream = Interpret("SHOW CONSTRAINT INFO");
//     ASSERT_EQ(stream.GetHeader().size(), 3U);
//     const auto &header = stream.GetHeader();
//     ASSERT_EQ(header[0], "constraint type");
//     ASSERT_EQ(header[1], "label");
//     ASSERT_EQ(header[2], "properties");
//     ASSERT_EQ(stream.GetResults().size(), 1U);
//     const auto &result = stream.GetResults().front();
//     ASSERT_EQ(result.size(), 3U);
//     ASSERT_EQ(result[0].ValueString(), "unique");
//     ASSERT_EQ(result[1].ValueString(), "A");
//     const auto &properties = result[2].ValueList();
//     ASSERT_EQ(properties.size(), 2U);
//     ASSERT_EQ(properties[0].ValueString(), "a");
//     ASSERT_EQ(properties[1].ValueString(), "b");
//   }

//   // Drop constraint.
//   Interpret("DROP CONSTRAINT ON (n:A) ASSERT n.a, n.b IS UNIQUE;");
//   // Removing the same constraint twice should not throw any exception.
//   Interpret("DROP CONSTRAINT ON (n:A) ASSERT n.a, n.b IS UNIQUE;");
// }

// TEST_F(InterpreterTest, ExplainQuery) {
//   const auto &interpreter_context = default_interpreter.interpreter_context;

//   EXPECT_EQ(interpreter_context.plan_cache.size(), 0U);
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 0U);
//   auto stream = Interpret("EXPLAIN MATCH (n) RETURN *;");
//   ASSERT_EQ(stream.GetHeader().size(), 1U);
//   EXPECT_EQ(stream.GetHeader().front(), "QUERY PLAN");
//   std::vector<std::string> expected_rows{" * Produce {n}", " * ScanAll (n)", " * Once"};
//   ASSERT_EQ(stream.GetResults().size(), expected_rows.size());
//   auto expected_it = expected_rows.begin();
//   for (const auto &row : stream.GetResults()) {
//     ASSERT_EQ(row.size(), 1U);
//     EXPECT_EQ(row.front().ValueString(), *expected_it);
//     ++expected_it;
//   }
//   // We should have a plan cache for MATCH ...
//   EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   // We should have AST cache for EXPLAIN ... and for inner MATCH ...
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 2U);
//   Interpret("MATCH (n) RETURN *;");
//   EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 2U);
// }

// TEST_F(InterpreterTest, ExplainQueryMultiplePulls) {
//   const auto &interpreter_context = default_interpreter.interpreter_context;

//   EXPECT_EQ(interpreter_context.plan_cache.size(), 0U);
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 0U);
//   auto [stream, qid] = Prepare("EXPLAIN MATCH (n) RETURN *;");
//   ASSERT_EQ(stream.GetHeader().size(), 1U);
//   EXPECT_EQ(stream.GetHeader().front(), "QUERY PLAN");
//   std::vector<std::string> expected_rows{" * Produce {n}", " * ScanAll (n)", " * Once"};
//   Pull(&stream, 1);
//   ASSERT_EQ(stream.GetResults().size(), 1);
//   auto expected_it = expected_rows.begin();
//   ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//   EXPECT_EQ(stream.GetResults()[0].front().ValueString(), *expected_it);
//   ++expected_it;

//   Pull(&stream, 1);
//   ASSERT_EQ(stream.GetResults().size(), 2);
//   ASSERT_EQ(stream.GetResults()[1].size(), 1U);
//   EXPECT_EQ(stream.GetResults()[1].front().ValueString(), *expected_it);
//   ++expected_it;

//   Pull(&stream);
//   ASSERT_EQ(stream.GetResults().size(), 3);
//   ASSERT_EQ(stream.GetResults()[2].size(), 1U);
//   EXPECT_EQ(stream.GetResults()[2].front().ValueString(), *expected_it);
//   // We should have a plan cache for MATCH ...
//   EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   // We should have AST cache for EXPLAIN ... and for inner MATCH ...
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 2U);
//   Interpret("MATCH (n) RETURN *;");
//   EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 2U);
// }

// TEST_F(InterpreterTest, ExplainQueryInMulticommandTransaction) {
//   const auto &interpreter_context = default_interpreter.interpreter_context;

//   EXPECT_EQ(interpreter_context.plan_cache.size(), 0U);
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 0U);
//   Interpret("BEGIN");
//   auto stream = Interpret("EXPLAIN MATCH (n) RETURN *;");
//   Interpret("COMMIT");
//   ASSERT_EQ(stream.GetHeader().size(), 1U);
//   EXPECT_EQ(stream.GetHeader().front(), "QUERY PLAN");
//   std::vector<std::string> expected_rows{" * Produce {n}", " * ScanAll (n)", " * Once"};
//   ASSERT_EQ(stream.GetResults().size(), expected_rows.size());
//   auto expected_it = expected_rows.begin();
//   for (const auto &row : stream.GetResults()) {
//     ASSERT_EQ(row.size(), 1U);
//     EXPECT_EQ(row.front().ValueString(), *expected_it);
//     ++expected_it;
//   }
//   // We should have a plan cache for MATCH ...
//   EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   // We should have AST cache for EXPLAIN ... and for inner MATCH ...
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 2U);
//   Interpret("MATCH (n) RETURN *;");
//   EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 2U);
// }

// TEST_F(InterpreterTest, ExplainQueryWithParams) {
//   const auto &interpreter_context = default_interpreter.interpreter_context;

//   EXPECT_EQ(interpreter_context.plan_cache.size(), 0U);
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 0U);
//   auto stream =
//       Interpret("EXPLAIN MATCH (n) WHERE n.id = $id RETURN *;", {{"id",
//       memgraph::storage::v3::PropertyValue(42)}});
//   ASSERT_EQ(stream.GetHeader().size(), 1U);
//   EXPECT_EQ(stream.GetHeader().front(), "QUERY PLAN");
//   std::vector<std::string> expected_rows{" * Produce {n}", " * Filter", " * ScanAll (n)", " * Once"};
//   ASSERT_EQ(stream.GetResults().size(), expected_rows.size());
//   auto expected_it = expected_rows.begin();
//   for (const auto &row : stream.GetResults()) {
//     ASSERT_EQ(row.size(), 1U);
//     EXPECT_EQ(row.front().ValueString(), *expected_it);
//     ++expected_it;
//   }
//   // We should have a plan cache for MATCH ...
//   EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   // We should have AST cache for EXPLAIN ... and for inner MATCH ...
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 2U);
//   Interpret("MATCH (n) WHERE n.id = $id RETURN *;", {{"id",
//   memgraph::storage::v3::PropertyValue("something else")}});
//   EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 2U);
// }

// TEST_F(InterpreterTest, ProfileQuery) {
//   const auto &interpreter_context = default_interpreter.interpreter_context;

//   EXPECT_EQ(interpreter_context.plan_cache.size(), 0U);
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 0U);
//   auto stream = Interpret("PROFILE MATCH (n) RETURN *;");
//   std::vector<std::string> expected_header{"OPERATOR", "ACTUAL HITS", "RELATIVE TIME", "ABSOLUTE
//   TIME"}; EXPECT_EQ(stream.GetHeader(), expected_header); std::vector<std::string> expected_rows{"*
//   Produce", "* ScanAll", "* Once"}; ASSERT_EQ(stream.GetResults().size(), expected_rows.size()); auto
//   expected_it = expected_rows.begin(); for (const auto &row : stream.GetResults()) {
//     ASSERT_EQ(row.size(), 4U);
//     EXPECT_EQ(row.front().ValueString(), *expected_it);
//     ++expected_it;
//   }
//   // We should have a plan cache for MATCH ...
//   EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   // We should have AST cache for PROFILE ... and for inner MATCH ...
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 2U);
//   Interpret("MATCH (n) RETURN *;");
//   EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 2U);
// }

// TEST_F(InterpreterTest, ProfileQueryMultiplePulls) {
//   const auto &interpreter_context = default_interpreter.interpreter_context;

//   EXPECT_EQ(interpreter_context.plan_cache.size(), 0U);
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 0U);
//   auto [stream, qid] = Prepare("PROFILE MATCH (n) RETURN *;");
//   std::vector<std::string> expected_header{"OPERATOR", "ACTUAL HITS", "RELATIVE TIME", "ABSOLUTE
//   TIME"}; EXPECT_EQ(stream.GetHeader(), expected_header);

//   std::vector<std::string> expected_rows{"* Produce", "* ScanAll", "* Once"};
//   auto expected_it = expected_rows.begin();

//   Pull(&stream, 1);
//   ASSERT_EQ(stream.GetResults().size(), 1U);
//   ASSERT_EQ(stream.GetResults()[0].size(), 4U);
//   ASSERT_EQ(stream.GetResults()[0][0].ValueString(), *expected_it);
//   ++expected_it;

//   Pull(&stream, 1);
//   ASSERT_EQ(stream.GetResults().size(), 2U);
//   ASSERT_EQ(stream.GetResults()[1].size(), 4U);
//   ASSERT_EQ(stream.GetResults()[1][0].ValueString(), *expected_it);
//   ++expected_it;

//   Pull(&stream);
//   ASSERT_EQ(stream.GetResults().size(), 3U);
//   ASSERT_EQ(stream.GetResults()[2].size(), 4U);
//   ASSERT_EQ(stream.GetResults()[2][0].ValueString(), *expected_it);

//   // We should have a plan cache for MATCH ...
//   EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   // We should have AST cache for PROFILE ... and for inner MATCH ...
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 2U);
//   Interpret("MATCH (n) RETURN *;");
//   EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 2U);
// }

// TEST_F(InterpreterTest, ProfileQueryInMulticommandTransaction) {
//   Interpret("BEGIN");
//   ASSERT_THROW(Interpret("PROFILE MATCH (n) RETURN *;"),
//   memgraph::query::v2::ProfileInMulticommandTxException); Interpret("ROLLBACK");
// }

// TEST_F(InterpreterTest, ProfileQueryWithParams) {
//   const auto &interpreter_context = default_interpreter.interpreter_context;

//   EXPECT_EQ(interpreter_context.plan_cache.size(), 0U);
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 0U);
//   auto stream =
//       Interpret("PROFILE MATCH (n) WHERE n.id = $id RETURN *;", {{"id",
//       memgraph::storage::v3::PropertyValue(42)}});
//   std::vector<std::string> expected_header{"OPERATOR", "ACTUAL HITS", "RELATIVE TIME", "ABSOLUTE
//   TIME"}; EXPECT_EQ(stream.GetHeader(), expected_header); std::vector<std::string> expected_rows{"*
//   Produce", "* Filter", "* ScanAll", "* Once"}; ASSERT_EQ(stream.GetResults().size(),
//   expected_rows.size()); auto expected_it = expected_rows.begin(); for (const auto &row :
//   stream.GetResults()) {
//     ASSERT_EQ(row.size(), 4U);
//     EXPECT_EQ(row.front().ValueString(), *expected_it);
//     ++expected_it;
//   }
//   // We should have a plan cache for MATCH ...
//   EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   // We should have AST cache for PROFILE ... and for inner MATCH ...
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 2U);
//   Interpret("MATCH (n) WHERE n.id = $id RETURN *;", {{"id",
//   memgraph::storage::v3::PropertyValue("something else")}});
//   EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 2U);
// }

// TEST_F(InterpreterTest, ProfileQueryWithLiterals) {
//   const auto &interpreter_context = default_interpreter.interpreter_context;
//   ASSERT_NO_THROW(Interpret("CREATE SCHEMA ON :Node(id INTEGER)"));

//   EXPECT_EQ(interpreter_context.plan_cache.size(), 0U);
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 1U);
//   auto stream = Interpret("PROFILE UNWIND range(1, 1000) AS x CREATE (:Node {id: x});", {});
//   std::vector<std::string> expected_header{"OPERATOR", "ACTUAL HITS", "RELATIVE TIME", "ABSOLUTE
//   TIME"}; EXPECT_EQ(stream.GetHeader(), expected_header); std::vector<std::string> expected_rows{"*
//   CreateNode", "* Unwind", "* Once"}; ASSERT_EQ(stream.GetResults().size(), expected_rows.size());
//   auto expected_it = expected_rows.begin();
//   for (const auto &row : stream.GetResults()) {
//     ASSERT_EQ(row.size(), 4U);
//     EXPECT_EQ(row.front().ValueString(), *expected_it);
//     ++expected_it;
//   }
//   // We should have a plan cache for UNWIND ...
//   EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   // We should have AST cache for PROFILE ... and for inner UNWIND ...
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 3U);
//   Interpret("UNWIND range(42, 4242) AS x CREATE (:Node {id: x});", {});
//   EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   EXPECT_EQ(interpreter_context.ast_cache.size(), 3U);
// }

// TEST_F(InterpreterTest, Transactions) {
//   auto &interpreter = default_interpreter.interpreter;
//   {
//     ASSERT_THROW(interpreter.CommitTransaction(),
//     memgraph::query::v2::ExplicitTransactionUsageException);
//     ASSERT_THROW(interpreter.RollbackTransaction(),
//     memgraph::query::v2::ExplicitTransactionUsageException); interpreter.BeginTransaction();
//     ASSERT_THROW(interpreter.BeginTransaction(),
//     memgraph::query::v2::ExplicitTransactionUsageException); auto [stream, qid] = Prepare("RETURN 2");
//     ASSERT_EQ(stream.GetHeader().size(), 1U);
//     EXPECT_EQ(stream.GetHeader()[0], "2");
//     Pull(&stream, 1);
//     ASSERT_EQ(stream.GetSummary().count("has_more"), 1);
//     ASSERT_FALSE(stream.GetSummary().at("has_more").ValueBool());
//     ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0][0].ValueInt(), 2);
//     interpreter.CommitTransaction();
//   }
//   {
//     interpreter.BeginTransaction();
//     auto [stream, qid] = Prepare("RETURN 2");
//     ASSERT_EQ(stream.GetHeader().size(), 1U);
//     EXPECT_EQ(stream.GetHeader()[0], "2");
//     Pull(&stream, 1);
//     ASSERT_EQ(stream.GetSummary().count("has_more"), 1);
//     ASSERT_FALSE(stream.GetSummary().at("has_more").ValueBool());
//     ASSERT_EQ(stream.GetResults()[0].size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0][0].ValueInt(), 2);
//     interpreter.RollbackTransaction();
//   }
// }

// TEST_F(InterpreterTest, Qid) {
//   auto &interpreter = default_interpreter.interpreter;
//   {
//     interpreter.BeginTransaction();
//     auto [stream, qid] = Prepare("RETURN 2");
//     ASSERT_TRUE(qid);
//     ASSERT_THROW(Pull(&stream, {}, *qid + 1), memgraph::query::v2::InvalidArgumentsException);
//     interpreter.RollbackTransaction();
//   }
//   {
//     interpreter.BeginTransaction();
//     auto [stream1, qid1] = Prepare("UNWIND(range(1,3)) as n RETURN n");
//     ASSERT_TRUE(qid1);
//     ASSERT_EQ(stream1.GetHeader().size(), 1U);
//     EXPECT_EQ(stream1.GetHeader()[0], "n");

//     auto [stream2, qid2] = Prepare("UNWIND(range(4,6)) as n RETURN n");
//     ASSERT_TRUE(qid2);
//     ASSERT_EQ(stream2.GetHeader().size(), 1U);
//     EXPECT_EQ(stream2.GetHeader()[0], "n");

//     Pull(&stream1, 1, qid1);
//     ASSERT_EQ(stream1.GetSummary().count("has_more"), 1);
//     ASSERT_TRUE(stream1.GetSummary().at("has_more").ValueBool());
//     ASSERT_EQ(stream1.GetResults().size(), 1U);
//     ASSERT_EQ(stream1.GetResults()[0].size(), 1U);
//     ASSERT_EQ(stream1.GetResults()[0][0].ValueInt(), 1);

//     auto [stream3, qid3] = Prepare("UNWIND(range(7,9)) as n RETURN n");
//     ASSERT_TRUE(qid3);
//     ASSERT_EQ(stream3.GetHeader().size(), 1U);
//     EXPECT_EQ(stream3.GetHeader()[0], "n");

//     Pull(&stream2, {}, qid2);
//     ASSERT_EQ(stream2.GetSummary().count("has_more"), 1);
//     ASSERT_FALSE(stream2.GetSummary().at("has_more").ValueBool());
//     ASSERT_EQ(stream2.GetResults().size(), 3U);
//     ASSERT_EQ(stream2.GetResults()[0].size(), 1U);
//     ASSERT_EQ(stream2.GetResults()[0][0].ValueInt(), 4);
//     ASSERT_EQ(stream2.GetResults()[1][0].ValueInt(), 5);
//     ASSERT_EQ(stream2.GetResults()[2][0].ValueInt(), 6);

//     Pull(&stream3, 1, qid3);
//     ASSERT_EQ(stream3.GetSummary().count("has_more"), 1);
//     ASSERT_TRUE(stream3.GetSummary().at("has_more").ValueBool());
//     ASSERT_EQ(stream3.GetResults().size(), 1U);
//     ASSERT_EQ(stream3.GetResults()[0].size(), 1U);
//     ASSERT_EQ(stream3.GetResults()[0][0].ValueInt(), 7);

//     Pull(&stream1, {}, qid1);
//     ASSERT_EQ(stream1.GetSummary().count("has_more"), 1);
//     ASSERT_FALSE(stream1.GetSummary().at("has_more").ValueBool());
//     ASSERT_EQ(stream1.GetResults().size(), 3U);
//     ASSERT_EQ(stream1.GetResults()[1].size(), 1U);
//     ASSERT_EQ(stream1.GetResults()[1][0].ValueInt(), 2);
//     ASSERT_EQ(stream1.GetResults()[2][0].ValueInt(), 3);

//     Pull(&stream3);
//     ASSERT_EQ(stream3.GetSummary().count("has_more"), 1);
//     ASSERT_FALSE(stream3.GetSummary().at("has_more").ValueBool());
//     ASSERT_EQ(stream3.GetResults().size(), 3U);
//     ASSERT_EQ(stream3.GetResults()[1].size(), 1U);
//     ASSERT_EQ(stream3.GetResults()[1][0].ValueInt(), 8);
//     ASSERT_EQ(stream3.GetResults()[2][0].ValueInt(), 9);

//     interpreter.CommitTransaction();
//   }
// }

// namespace {
// // copied from utils_csv_parsing.cpp - tmp dir management and csv file writer
// class TmpDirManager final {
//  public:
//   explicit TmpDirManager(const std::string_view directory)
//       : tmp_dir_{std::filesystem::temp_directory_path() / directory} {
//     CreateDir();
//   }
//   ~TmpDirManager() { Clear(); }

//   const std::filesystem::path &Path() const { return tmp_dir_; }

//  private:
//   std::filesystem::path tmp_dir_;

//   void CreateDir() {
//     if (!std::filesystem::exists(tmp_dir_)) {
//       std::filesystem::create_directory(tmp_dir_);
//     }
//   }

//   void Clear() {
//     if (!std::filesystem::exists(tmp_dir_)) return;
//     std::filesystem::remove_all(tmp_dir_);
//   }
// };

// class FileWriter {
//  public:
//   explicit FileWriter(const std::filesystem::path path) { stream_.open(path); }

//   FileWriter(const FileWriter &) = delete;
//   FileWriter &operator=(const FileWriter &) = delete;

//   FileWriter(FileWriter &&) = delete;
//   FileWriter &operator=(FileWriter &&) = delete;

//   void Close() { stream_.close(); }

//   size_t WriteLine(const std::string_view line) {
//     if (!stream_.is_open()) {
//       return 0;
//     }

//     stream_ << line << std::endl;

//     // including the newline character
//     return line.size() + 1;
//   }

//  private:
//   std::ofstream stream_;
// };

// std::string CreateRow(const std::vector<std::string> &columns, const std::string_view delim) {
//   return memgraph::utils::Join(columns, delim);
// }
// }  // namespace

// TEST_F(InterpreterTest, LoadCsvClause) {
//   auto dir_manager = TmpDirManager("csv_directory");
//   const auto csv_path = dir_manager.Path() / "file.csv";
//   auto writer = FileWriter(csv_path);

//   const std::string delimiter{"|"};

//   const std::vector<std::string> header{"A", "B", "C"};
//   writer.WriteLine(CreateRow(header, delimiter));

//   const std::vector<std::string> good_columns_1{"a", "b", "c"};
//   writer.WriteLine(CreateRow(good_columns_1, delimiter));

//   const std::vector<std::string> bad_columns{"\"\"1", "2", "3"};
//   writer.WriteLine(CreateRow(bad_columns, delimiter));

//   const std::vector<std::string> good_columns_2{"d", "e", "f"};
//   writer.WriteLine(CreateRow(good_columns_2, delimiter));

//   writer.Close();

//   {
//     const std::string query = fmt::format(R"(LOAD CSV FROM "{}" WITH HEADER IGNORE BAD DELIMITER "{}"
//     AS x RETURN x.A)",
//                                           csv_path.string(), delimiter);
//     auto [stream, qid] = Prepare(query);
//     ASSERT_EQ(stream.GetHeader().size(), 1U);
//     EXPECT_EQ(stream.GetHeader()[0], "x.A");

//     Pull(&stream, 1);
//     ASSERT_EQ(stream.GetSummary().count("has_more"), 1);
//     ASSERT_TRUE(stream.GetSummary().at("has_more").ValueBool());
//     ASSERT_EQ(stream.GetResults().size(), 1U);
//     ASSERT_EQ(stream.GetResults()[0][0].ValueString(), "a");

//     Pull(&stream, 1);
//     ASSERT_EQ(stream.GetSummary().count("has_more"), 1);
//     ASSERT_FALSE(stream.GetSummary().at("has_more").ValueBool());
//     ASSERT_EQ(stream.GetResults().size(), 2U);
//     ASSERT_EQ(stream.GetResults()[1][0].ValueString(), "d");
//   }

//   {
//     const std::string query = fmt::format(R"(LOAD CSV FROM "{}" WITH HEADER IGNORE BAD DELIMITER "{}"
//     AS x RETURN x.C)",
//                                           csv_path.string(), delimiter);
//     auto [stream, qid] = Prepare(query);
//     ASSERT_EQ(stream.GetHeader().size(), 1U);
//     EXPECT_EQ(stream.GetHeader()[0], "x.C");

//     Pull(&stream);
//     ASSERT_EQ(stream.GetSummary().count("has_more"), 1);
//     ASSERT_FALSE(stream.GetSummary().at("has_more").ValueBool());
//     ASSERT_EQ(stream.GetResults().size(), 2U);
//     ASSERT_EQ(stream.GetResults()[0][0].ValueString(), "c");
//     ASSERT_EQ(stream.GetResults()[1][0].ValueString(), "f");
//   }
// }

// TEST_F(InterpreterTest, CacheableQueries) {
//   const auto &interpreter_context = default_interpreter.interpreter_context;
//   // This should be cached
//   {
//     SCOPED_TRACE("Cacheable query");
//     Interpret("RETURN 1");
//     EXPECT_EQ(interpreter_context.ast_cache.size(), 1U);
//     EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   }

//   {
//     SCOPED_TRACE("Uncacheable query");
//     // Queries which are calling procedure should not be cached because the
//     // result signature could be changed
//     Interpret("CALL mg.load_all()");
//     EXPECT_EQ(interpreter_context.ast_cache.size(), 1U);
//     EXPECT_EQ(interpreter_context.plan_cache.size(), 1U);
//   }
// }

// TEST_F(InterpreterTest, AllowLoadCsvConfig) {
//   const auto check_load_csv_queries = [&](const bool allow_load_csv) {
//     TmpDirManager directory_manager{"allow_load_csv"};
//     const auto csv_path = directory_manager.Path() / "file.csv";
//     auto writer = FileWriter(csv_path);
//     const std::vector<std::string> data{"A", "B", "C"};
//     writer.WriteLine(CreateRow(data, ","));
//     writer.Close();

//     const std::array<std::string, 2> queries = {
//         fmt::format("LOAD CSV FROM \"{}\" WITH HEADER AS row RETURN row", csv_path.string()),
//         "CREATE TRIGGER trigger ON CREATE BEFORE COMMIT EXECUTE LOAD CSV FROM 'file.csv' WITH HEADER
//         AS row RETURN " "row"};

//     InterpreterFaker interpreter_faker{&db_, {.query = {.allow_load_csv = allow_load_csv}},
//     directory_manager.Path()}; for (const auto &query : queries) {
//       if (allow_load_csv) {
//         SCOPED_TRACE(fmt::format("'{}' should not throw because LOAD CSV is allowed", query));
//         ASSERT_NO_THROW(interpreter_faker.Interpret(query));
//       } else {
//         SCOPED_TRACE(fmt::format("'{}' should throw becuase LOAD CSV is not allowed", query));
//         ASSERT_THROW(interpreter_faker.Interpret(query), memgraph::utils::BasicException);
//       }
//       SCOPED_TRACE(fmt::format("Normal query should not throw (allow_load_csv: {})", allow_load_csv));
//       ASSERT_NO_THROW(interpreter_faker.Interpret("RETURN 1"));
//     }
//   };

//   check_load_csv_queries(true);
//   check_load_csv_queries(false);
// }

// void AssertAllValuesAreZero(const std::map<std::string, memgraph::communication::bolt::Value> &map,
//                             const std::vector<std::string> &exceptions) {
//   for (const auto &[key, value] : map) {
//     if (const auto it = std::find(exceptions.begin(), exceptions.end(), key); it != exceptions.end())
//     continue; ASSERT_EQ(value.ValueInt(), 0) << "Value " << key << " actual: " << value.ValueInt() <<
//     ", expected 0";
//   }
// }

// TEST_F(InterpreterTest, ExecutionStatsIsValid) {
//   {
//     auto [stream, qid] = Prepare("MATCH (n) DELETE n;");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("stats"), 0);
//   }
//   {
//     EXPECT_NO_THROW(Interpret("CREATE SCHEMA ON :L1(name STRING)"));
//     std::array stats_keys{"nodes-created",  "nodes-deleted", "relationships-created",
//     "relationships-deleted",
//                           "properties-set", "labels-added",  "labels-removed"};
//     auto [stream, qid] = Prepare("CREATE (:L1 {name: 'name1'});");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("stats"), 1);
//     ASSERT_TRUE(stream.GetSummary().at("stats").IsMap());
//     auto stats = stream.GetSummary().at("stats").ValueMap();
//     ASSERT_TRUE(
//         std::all_of(stats_keys.begin(), stats_keys.end(), [&stats](const auto &key) { return
//         stats.contains(key);
//         }));
//     AssertAllValuesAreZero(stats, {"nodes-created"});
//   }
// }

// TEST_F(InterpreterTest, ExecutionStatsValues) {
//   EXPECT_NO_THROW(Interpret("CREATE SCHEMA ON :L1(name STRING)"));
//   {
//     auto [stream, qid] =
//         Prepare("CREATE (:L1{name: 'name1'}),(:L1{name: 'name2'}),(:L1{name: 'name3'}),(:L1{name:
//         'name4'});");

//     Pull(&stream);
//     auto stats = stream.GetSummary().at("stats").ValueMap();
//     ASSERT_EQ(stats["nodes-created"].ValueInt(), 4);
//     AssertAllValuesAreZero(stats, {"nodes-created", "labels-added"});
//   }
//   {
//     auto [stream, qid] = Prepare("MATCH (n) DELETE n;");
//     Pull(&stream);

//     auto stats = stream.GetSummary().at("stats").ValueMap();
//     ASSERT_EQ(stats["nodes-deleted"].ValueInt(), 4);
//     AssertAllValuesAreZero(stats, {"nodes-deleted"});
//   }
//   {
//     auto [stream, qid] =
//         Prepare("CREATE (n:L1 {name: 'name5'})-[:TO]->(m:L1{name: 'name6'}), (n)-[:TO]->(m),
//         (n)-[:TO]->(m);");

//     Pull(&stream);

//     auto stats = stream.GetSummary().at("stats").ValueMap();
//     ASSERT_EQ(stats["nodes-created"].ValueInt(), 2);
//     ASSERT_EQ(stats["relationships-created"].ValueInt(), 3);
//     AssertAllValuesAreZero(stats, {"nodes-created", "relationships-created"});
//   }
//   {
//     auto [stream, qid] = Prepare("MATCH (n) DETACH DELETE n;");
//     Pull(&stream);

//     auto stats = stream.GetSummary().at("stats").ValueMap();
//     ASSERT_EQ(stats["nodes-deleted"].ValueInt(), 2);
//     ASSERT_EQ(stats["relationships-deleted"].ValueInt(), 3);
//     AssertAllValuesAreZero(stats, {"nodes-deleted", "relationships-deleted"});
//   }
//   {
//     auto [stream, qid] = Prepare("CREATE (n:L1 {name: 'name7'}) SET n:L2:L3:L4");
//     Pull(&stream);

//     auto stats = stream.GetSummary().at("stats").ValueMap();
//     ASSERT_EQ(stats["nodes-created"].ValueInt(), 1);
//     ASSERT_EQ(stats["labels-added"].ValueInt(), 3);
//     AssertAllValuesAreZero(stats, {"nodes-created", "labels-added"});
//   }
//   {
//     auto [stream, qid] = Prepare("MATCH (n:L1) SET n.name2='test';");
//     Pull(&stream);

//     auto stats = stream.GetSummary().at("stats").ValueMap();
//     ASSERT_EQ(stats["properties-set"].ValueInt(), 1);
//     AssertAllValuesAreZero(stats, {"properties-set"});
//   }
// }

// TEST_F(InterpreterTest, NotificationsValidStructure) {
//   {
//     auto [stream, qid] = Prepare("MATCH (n) DELETE n;");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("notifications"), 0);
//   }
//   {
//     auto [stream, qid] = Prepare("CREATE INDEX ON :Person(id);");
//     Pull(&stream);

//     // Assert notifications list
//     ASSERT_EQ(stream.GetSummary().count("notifications"), 1);
//     ASSERT_TRUE(stream.GetSummary().at("notifications").IsList());
//     auto notifications = stream.GetSummary().at("notifications").ValueList();

//     // Assert one notification structure
//     ASSERT_EQ(notifications.size(), 1);
//     ASSERT_TRUE(notifications[0].IsMap());
//     auto notification = notifications[0].ValueMap();
//     ASSERT_TRUE(notification.contains("severity"));
//     ASSERT_TRUE(notification.contains("code"));
//     ASSERT_TRUE(notification.contains("title"));
//     ASSERT_TRUE(notification.contains("description"));
//     ASSERT_TRUE(notification["severity"].IsString());
//     ASSERT_TRUE(notification["code"].IsString());
//     ASSERT_TRUE(notification["title"].IsString());
//     ASSERT_TRUE(notification["description"].IsString());
//   }
// }

// TEST_F(InterpreterTest, IndexInfoNotifications) {
//   {
//     auto [stream, qid] = Prepare("CREATE INDEX ON :Person;");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("notifications"), 1);
//     auto notifications = stream.GetSummary().at("notifications").ValueList();

//     auto notification = notifications[0].ValueMap();
//     ASSERT_EQ(notification["severity"].ValueString(), "INFO");
//     ASSERT_EQ(notification["code"].ValueString(), "CreateIndex");
//     ASSERT_EQ(notification["title"].ValueString(), "Created index on label Person on properties .");
//     ASSERT_EQ(notification["description"].ValueString(), "");
//   }
//   {
//     auto [stream, qid] = Prepare("CREATE INDEX ON :Person(id);");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("notifications"), 1);
//     auto notifications = stream.GetSummary().at("notifications").ValueList();

//     auto notification = notifications[0].ValueMap();
//     ASSERT_EQ(notification["severity"].ValueString(), "INFO");
//     ASSERT_EQ(notification["code"].ValueString(), "CreateIndex");
//     ASSERT_EQ(notification["title"].ValueString(), "Created index on label Person on properties id.");
//     ASSERT_EQ(notification["description"].ValueString(), "");
//   }
//   {
//     auto [stream, qid] = Prepare("CREATE INDEX ON :Person(id);");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("notifications"), 1);
//     auto notifications = stream.GetSummary().at("notifications").ValueList();

//     auto notification = notifications[0].ValueMap();
//     ASSERT_EQ(notification["severity"].ValueString(), "INFO");
//     ASSERT_EQ(notification["code"].ValueString(), "IndexAlreadyExists");
//     ASSERT_EQ(notification["title"].ValueString(), "Index on label Person on properties id already
//     exists."); ASSERT_EQ(notification["description"].ValueString(), "");
//   }
//   {
//     auto [stream, qid] = Prepare("DROP INDEX ON :Person(id);");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("notifications"), 1);
//     auto notifications = stream.GetSummary().at("notifications").ValueList();

//     auto notification = notifications[0].ValueMap();
//     ASSERT_EQ(notification["severity"].ValueString(), "INFO");
//     ASSERT_EQ(notification["code"].ValueString(), "DropIndex");
//     ASSERT_EQ(notification["title"].ValueString(), "Dropped index on label Person on properties id.");
//     ASSERT_EQ(notification["description"].ValueString(), "");
//   }
//   {
//     auto [stream, qid] = Prepare("DROP INDEX ON :Person(id);");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("notifications"), 1);
//     auto notifications = stream.GetSummary().at("notifications").ValueList();

//     auto notification = notifications[0].ValueMap();
//     ASSERT_EQ(notification["severity"].ValueString(), "INFO");
//     ASSERT_EQ(notification["code"].ValueString(), "IndexDoesNotExist");
//     ASSERT_EQ(notification["title"].ValueString(), "Index on label Person on properties id doesn't
//     exist."); ASSERT_EQ(notification["description"].ValueString(), "");
//   }
// }

// TEST_F(InterpreterTest, ConstraintUniqueInfoNotifications) {
//   {
//     auto [stream, qid] = Prepare("CREATE CONSTRAINT ON (n:Person) ASSERT n.email, n.id IS UNIQUE;");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("notifications"), 1);
//     auto notifications = stream.GetSummary().at("notifications").ValueList();

//     auto notification = notifications[0].ValueMap();
//     ASSERT_EQ(notification["severity"].ValueString(), "INFO");
//     ASSERT_EQ(notification["code"].ValueString(), "CreateConstraint");
//     ASSERT_EQ(notification["title"].ValueString(),
//               "Created UNIQUE constraint on label Person on properties email, id.");
//     ASSERT_EQ(notification["description"].ValueString(), "");
//   }
//   {
//     auto [stream, qid] = Prepare("CREATE CONSTRAINT ON (n:Person) ASSERT n.email, n.id IS UNIQUE;");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("notifications"), 1);
//     auto notifications = stream.GetSummary().at("notifications").ValueList();

//     auto notification = notifications[0].ValueMap();
//     ASSERT_EQ(notification["severity"].ValueString(), "INFO");
//     ASSERT_EQ(notification["code"].ValueString(), "ConstraintAlreadyExists");
//     ASSERT_EQ(notification["title"].ValueString(),
//               "Constraint UNIQUE on label Person on properties email, id already exists.");
//     ASSERT_EQ(notification["description"].ValueString(), "");
//   }
//   {
//     auto [stream, qid] = Prepare("DROP CONSTRAINT ON (n:Person) ASSERT n.email, n.id IS UNIQUE;");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("notifications"), 1);
//     auto notifications = stream.GetSummary().at("notifications").ValueList();

//     auto notification = notifications[0].ValueMap();
//     ASSERT_EQ(notification["severity"].ValueString(), "INFO");
//     ASSERT_EQ(notification["code"].ValueString(), "DropConstraint");
//     ASSERT_EQ(notification["title"].ValueString(),
//               "Dropped UNIQUE constraint on label Person on properties email, id.");
//     ASSERT_EQ(notification["description"].ValueString(), "");
//   }
//   {
//     auto [stream, qid] = Prepare("DROP CONSTRAINT ON (n:Person) ASSERT n.email, n.id IS UNIQUE;");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("notifications"), 1);
//     auto notifications = stream.GetSummary().at("notifications").ValueList();

//     auto notification = notifications[0].ValueMap();
//     ASSERT_EQ(notification["severity"].ValueString(), "INFO");
//     ASSERT_EQ(notification["code"].ValueString(), "ConstraintDoesNotExist");
//     ASSERT_EQ(notification["title"].ValueString(),
//               "Constraint UNIQUE on label Person on properties email, id doesn't exist.");
//     ASSERT_EQ(notification["description"].ValueString(), "");
//   }
// }

// TEST_F(InterpreterTest, ConstraintExistsInfoNotifications) {
//   {
//     auto [stream, qid] = Prepare("CREATE CONSTRAINT ON (n:L1) ASSERT EXISTS (n.name);");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("notifications"), 1);
//     auto notifications = stream.GetSummary().at("notifications").ValueList();

//     auto notification = notifications[0].ValueMap();
//     ASSERT_EQ(notification["severity"].ValueString(), "INFO");
//     ASSERT_EQ(notification["code"].ValueString(), "CreateConstraint");
//     ASSERT_EQ(notification["title"].ValueString(), "Created EXISTS constraint on label L1 on
//     properties name."); ASSERT_EQ(notification["description"].ValueString(), "");
//   }
//   {
//     auto [stream, qid] = Prepare("CREATE CONSTRAINT ON (n:L1) ASSERT EXISTS (n.name);");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("notifications"), 1);
//     auto notifications = stream.GetSummary().at("notifications").ValueList();

//     auto notification = notifications[0].ValueMap();
//     ASSERT_EQ(notification["severity"].ValueString(), "INFO");
//     ASSERT_EQ(notification["code"].ValueString(), "ConstraintAlreadyExists");
//     ASSERT_EQ(notification["title"].ValueString(), "Constraint EXISTS on label L1 on properties name
//     already exists."); ASSERT_EQ(notification["description"].ValueString(), "");
//   }
//   {
//     auto [stream, qid] = Prepare("DROP CONSTRAINT ON (n:L1) ASSERT EXISTS (n.name);");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("notifications"), 1);
//     auto notifications = stream.GetSummary().at("notifications").ValueList();

//     auto notification = notifications[0].ValueMap();
//     ASSERT_EQ(notification["severity"].ValueString(), "INFO");
//     ASSERT_EQ(notification["code"].ValueString(), "DropConstraint");
//     ASSERT_EQ(notification["title"].ValueString(), "Dropped EXISTS constraint on label L1 on
//     properties name."); ASSERT_EQ(notification["description"].ValueString(), "");
//   }
//   {
//     auto [stream, qid] = Prepare("DROP CONSTRAINT ON (n:L1) ASSERT EXISTS (n.name);");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("notifications"), 1);
//     auto notifications = stream.GetSummary().at("notifications").ValueList();

//     auto notification = notifications[0].ValueMap();
//     ASSERT_EQ(notification["severity"].ValueString(), "INFO");
//     ASSERT_EQ(notification["code"].ValueString(), "ConstraintDoesNotExist");
//     ASSERT_EQ(notification["title"].ValueString(), "Constraint EXISTS on label L1 on properties name
//     doesn'texist."); ASSERT_EQ(notification["description"].ValueString(), "");
//   }
// }

// TEST_F(InterpreterTest, TriggerInfoNotifications) {
//   {
//     auto [stream, qid] = Prepare(
//         "CREATE TRIGGER bestTriggerEver ON  CREATE AFTER COMMIT EXECUTE "
//         "CREATE ();");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("notifications"), 1);
//     auto notifications = stream.GetSummary().at("notifications").ValueList();

//     auto notification = notifications[0].ValueMap();
//     ASSERT_EQ(notification["severity"].ValueString(), "INFO");
//     ASSERT_EQ(notification["code"].ValueString(), "CreateTrigger");
//     ASSERT_EQ(notification["title"].ValueString(), "Created trigger bestTriggerEver.");
//     ASSERT_EQ(notification["description"].ValueString(), "");
//   }
//   {
//     auto [stream, qid] = Prepare("DROP TRIGGER bestTriggerEver;");
//     Pull(&stream);

//     ASSERT_EQ(stream.GetSummary().count("notifications"), 1);
//     auto notifications = stream.GetSummary().at("notifications").ValueList();

//     auto notification = notifications[0].ValueMap();
//     ASSERT_EQ(notification["severity"].ValueString(), "INFO");
//     ASSERT_EQ(notification["code"].ValueString(), "DropTrigger");
//     ASSERT_EQ(notification["title"].ValueString(), "Dropped trigger bestTriggerEver.");
//     ASSERT_EQ(notification["description"].ValueString(), "");
//   }
// }

// TEST_F(InterpreterTest, LoadCsvClauseNotification) {
//   auto dir_manager = TmpDirManager("csv_directory");
//   const auto csv_path = dir_manager.Path() / "file.csv";
//   auto writer = FileWriter(csv_path);

//   const std::string delimiter{"|"};

//   const std::vector<std::string> header{"A", "B", "C"};
//   writer.WriteLine(CreateRow(header, delimiter));

//   const std::vector<std::string> good_columns_1{"a", "b", "c"};
//   writer.WriteLine(CreateRow(good_columns_1, delimiter));

//   writer.Close();

//   const std::string query = fmt::format(R"(LOAD CSV FROM "{}" WITH HEADER IGNORE BAD DELIMITER "{}" AS
//   x RETURN x;)",
//                                         csv_path.string(), delimiter);
//   auto [stream, qid] = Prepare(query);
//   Pull(&stream);

//   ASSERT_EQ(stream.GetSummary().count("notifications"), 1);
//   auto notifications = stream.GetSummary().at("notifications").ValueList();

//   auto notification = notifications[0].ValueMap();
//   ASSERT_EQ(notification["severity"].ValueString(), "INFO");
//   ASSERT_EQ(notification["code"].ValueString(), "LoadCSVTip");
//   ASSERT_EQ(notification["title"].ValueString(),
//             "It's important to note that the parser parses the values as strings. It's up to the user
//             to " "convert the parsed row values to the appropriate type. This can be done using the
//             built-in " "conversion functions such as ToInteger, ToFloat, ToBoolean etc.");
//   ASSERT_EQ(notification["description"].ValueString(), "");
// }

// TEST_F(InterpreterTest, CreateSchemaMulticommandTransaction) {
//   Interpret("BEGIN");
//   ASSERT_THROW(Interpret("CREATE SCHEMA ON :label(name STRING, age INTEGER)"),
//                memgraph::query::v2::ConstraintInMulticommandTxException);
//   Interpret("ROLLBACK");
// }

// TEST_F(InterpreterTest, ShowSchemasMulticommandTransaction) {
//   Interpret("BEGIN");
//   ASSERT_THROW(Interpret("SHOW SCHEMAS"), memgraph::query::v2::ConstraintInMulticommandTxException);
//   Interpret("ROLLBACK");
// }

// TEST_F(InterpreterTest, ShowSchemaMulticommandTransaction) {
//   Interpret("BEGIN");
//   ASSERT_THROW(Interpret("SHOW SCHEMA ON :label"),
//   memgraph::query::v2::ConstraintInMulticommandTxException); Interpret("ROLLBACK");
// }

// TEST_F(InterpreterTest, DropSchemaMulticommandTransaction) {
//   Interpret("BEGIN");
//   ASSERT_THROW(Interpret("DROP SCHEMA ON :label"),
//   memgraph::query::v2::ConstraintInMulticommandTxException); Interpret("ROLLBACK");
// }

// TEST_F(InterpreterTest, SchemaTestCreateAndShow) {
//   // Empty schema type map should result with syntax exception.
//   ASSERT_THROW(Interpret("CREATE SCHEMA ON :label();"),
//   memgraph::frontend::opencypher::SyntaxException);

//   // Duplicate properties are should also cause an exception
//   ASSERT_THROW(Interpret("CREATE SCHEMA ON :label(name STRING, name STRING);"),
//   memgraph::query::v2::SemanticException); ASSERT_THROW(Interpret("CREATE SCHEMA ON :label(name
//   STRING, name INTEGER);"),
//                memgraph::query::v2::SemanticException);

//   {
//     // Cannot create same schema twice
//     Interpret("CREATE SCHEMA ON :label(name STRING, age INTEGER)");
//     ASSERT_THROW(Interpret("CREATE SCHEMA ON :label(name STRING);"),
//     memgraph::query::v2::QueryException);
//   }
//   // Show schema
//   {
//     auto stream = Interpret("SHOW SCHEMA ON :label");
//     ASSERT_EQ(stream.GetHeader().size(), 2U);
//     const auto &header = stream.GetHeader();
//     ASSERT_EQ(header[0], "property_name");
//     ASSERT_EQ(header[1], "property_type");
//     ASSERT_EQ(stream.GetResults().size(), 2U);
//     std::unordered_map<std::string, std::string> result_table{{"age", "Integer"}, {"name", "String"}};

//     const auto &result = stream.GetResults().front();
//     ASSERT_EQ(result.size(), 2U);
//     const auto key1 = result[0].ValueString();
//     ASSERT_TRUE(result_table.contains(key1));
//     ASSERT_EQ(result[1].ValueString(), result_table[key1]);

//     const auto &result2 = stream.GetResults().front();
//     ASSERT_EQ(result2.size(), 2U);
//     const auto key2 = result2[0].ValueString();
//     ASSERT_TRUE(result_table.contains(key2));
//     ASSERT_EQ(result[1].ValueString(), result_table[key2]);
//   }
//   // Create Another Schema
//   Interpret("CREATE SCHEMA ON :label2(place STRING, dur DURATION)");

//   // Show schemas
//   {
//     auto stream = Interpret("SHOW SCHEMAS");
//     ASSERT_EQ(stream.GetHeader().size(), 2U);
//     const auto &header = stream.GetHeader();
//     ASSERT_EQ(header[0], "label");
//     ASSERT_EQ(header[1], "primary_key");
//     ASSERT_EQ(stream.GetResults().size(), 2U);
//     std::unordered_map<std::string, std::unordered_set<std::string>> result_table{
//         {"label", {"name::String", "age::Integer"}}, {"label2", {"place::String", "dur::Duration"}}};

//     const auto &result = stream.GetResults().front();
//     ASSERT_EQ(result.size(), 2U);
//     const auto key1 = result[0].ValueString();
//     ASSERT_TRUE(result_table.contains(key1));
//     const auto primary_key_split = StringToUnorderedSet(result[1].ValueString());
//     ASSERT_EQ(primary_key_split.size(), 2);
//     ASSERT_TRUE(primary_key_split == result_table[key1]) << "actual value is: " <<
//     result[1].ValueString();

//     const auto &result2 = stream.GetResults().front();
//     ASSERT_EQ(result2.size(), 2U);
//     const auto key2 = result2[0].ValueString();
//     ASSERT_TRUE(result_table.contains(key2));
//     const auto primary_key_split2 = StringToUnorderedSet(result2[1].ValueString());
//     ASSERT_EQ(primary_key_split2.size(), 2);
//     ASSERT_TRUE(primary_key_split2 == result_table[key2]) << "Real value is: " <<
//     result[1].ValueString();
//   }
// }

// TEST_F(InterpreterTest, SchemaTestCreateDropAndShow) {
//   Interpret("CREATE SCHEMA ON :label(name STRING, age INTEGER)");
//   // Wrong syntax for dropping schema.
//   ASSERT_THROW(Interpret("DROP SCHEMA ON :label();"),
//   memgraph::frontend::opencypher::SyntaxException);
//   // Cannot drop non existant schema.
//   ASSERT_THROW(Interpret("DROP SCHEMA ON :label1;"), memgraph::query::v2::QueryException);

//   // Create Schema and Drop
//   auto get_number_of_schemas = [this]() {
//     auto stream = Interpret("SHOW SCHEMAS");
//     return stream.GetResults().size();
//   };

//   ASSERT_EQ(get_number_of_schemas(), 1);
//   Interpret("CREATE SCHEMA ON :label1(name STRING, age INTEGER)");
//   ASSERT_EQ(get_number_of_schemas(), 2);
//   Interpret("CREATE SCHEMA ON :label2(name STRING, alive BOOL)");
//   ASSERT_EQ(get_number_of_schemas(), 3);
//   Interpret("DROP SCHEMA ON :label1");
//   ASSERT_EQ(get_number_of_schemas(), 2);
//   Interpret("CREATE SCHEMA ON :label3(name STRING, birthday LOCALDATETIME)");
//   ASSERT_EQ(get_number_of_schemas(), 3);
//   Interpret("DROP SCHEMA ON :label2");
//   ASSERT_EQ(get_number_of_schemas(), 2);
//   Interpret("CREATE SCHEMA ON :label4(name STRING, age DURATION)");
//   ASSERT_EQ(get_number_of_schemas(), 3);
//   Interpret("DROP SCHEMA ON :label3");
//   ASSERT_EQ(get_number_of_schemas(), 2);
//   Interpret("DROP SCHEMA ON :label");
//   ASSERT_EQ(get_number_of_schemas(), 1);

//   // Show schemas
//   auto stream = Interpret("SHOW SCHEMAS");
//   ASSERT_EQ(stream.GetHeader().size(), 2U);
//   const auto &header = stream.GetHeader();
//   ASSERT_EQ(header[0], "label");
//   ASSERT_EQ(header[1], "primary_key");
//   ASSERT_EQ(stream.GetResults().size(), 1U);
//   std::unordered_map<std::string, std::unordered_set<std::string>> result_table{
//       {"label4", {"name::String", "age::Duration"}}};

//   const auto &result = stream.GetResults().front();
//   ASSERT_EQ(result.size(), 2U);
//   const auto key1 = result[0].ValueString();
//   ASSERT_TRUE(result_table.contains(key1));
//   const auto primary_key_split = StringToUnorderedSet(result[1].ValueString());
//   ASSERT_EQ(primary_key_split.size(), 2);
//   ASSERT_TRUE(primary_key_split == result_table[key1]);
// }
}  // namespace memgraph::query::v2::tests
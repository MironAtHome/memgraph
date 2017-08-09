//
// Copyright 2017 Memgraph
// Created by Florijan Stamenkovic on 23.03.17.
//

#pragma once

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <thread>
#include <vector>

#include "data_structures/concurrent/skiplist.hpp"
#include "database/dbms.hpp"
#include "database/graph_db_datatypes.hpp"
#include "mvcc/version_list.hpp"
#include "storage/property_value.hpp"
#include "storage/vertex_accessor.hpp"
#include "threading/sync/lock_timeout_exception.hpp"
#include "utils/assert.hpp"

namespace utils {

/**
 * Returns a lambda that generates random ints
 * in the [from, to) range.
 */
auto RandomIntGenerator(int from, int to) {
  permanent_assert(from < to, "Must have from < to");
  int range = to - from;
  return [from, range]() -> int { return rand() % range + from; };
}

/**
 * Random graph generator. Create a graph
 * with a sequence of steps.
 */
class RandomGraphGenerator {
 public:
  explicit RandomGraphGenerator(Dbms &dbms) : dbms_(dbms) {}

  /**
   * Adds a progress listener that gets notified when
   * edges / vertices get created.
   *
   * A listener is a function that gets notified after every
   * vertex / edge insertion. If data creation is multi-threaded,
   * then so is progress listener notification.
   */
  void AddProgressListener(
      std::function<void(RandomGraphGenerator &)> listener) {
    progress_listeners_.emplace_back(listener);
  }

  /**
   * Adds the given number of vertices, with
   * the given labels.
   *
   * @param count the number of vertices to add
   * @param label_names a vector of label names to assign to each
   * created vertex
   * @param thread_count The number of threads in which to add edges
   * @param batch_size The number of vertices to be created in
   *  a single transcation
   */
  void AddVertices(int count, const std::vector<std::string> &label_names,
                   int thread_count, int batch_size = 2000) {
    auto dba = dbms_.active();
    std::vector<GraphDbTypes::Label> labels;
    for (const auto &label_name : label_names)
      labels.push_back(dba->label(label_name));

    Map(
        [&labels, this](GraphDbAccessor &dba) {
          auto vertex = dba.insert_vertex();
          for (auto label : labels) vertex.add_label(label);
          NotifyProgressListeners();
        },
        count, thread_count, batch_size);
  }

  /**
   * Returns the number of vertices created by this generator,
   * regardless of their labels.
   */
  int64_t VertexCount() const { return dbms_.active()->vertices_count(); }

  /**
   * Adds the given number of edges to the graph.
   *
   * @param count The number of edges to add.
   * @param edge_type_name Name of the edge type.
   * @param thread_count The number of threads in which to add edges.
   * @param batch_size The number of vertices to be created in
   *  a single transcation
   * @param from_filter Filter of from vertices for new edges.
   *    By default all vertices are accepted.
   * @param to_filter Filter of to vertices for new edges.
   *    By default all vertices are accepted.
   */
  void AddEdges(int count, const std::string &edge_type_name, int thread_count,
                int batch_size = 50,
                const std::function<bool(VertexAccessor &va)> &from_filter = {},
                const std::function<bool(VertexAccessor &va)> &to_filter = {}) {
    // create two temporary sets of vertices we will poll from
    auto vertices_from = FilterVertices(from_filter);
    auto vertices_to = FilterVertices(to_filter);

    auto dba = dbms_.active();
    auto edge_type = dba->edge_type(edge_type_name);

    // for small vertex counts reduce the batch size
    batch_size = std::min(batch_size,
                          static_cast<int>(dba->vertices_count() / 1000 + 1));

    Map(
        [&vertices_from, &vertices_to, edge_type, this](GraphDbAccessor &dba) {
          auto from =
              dba.Transfer(vertices_from[rand() % vertices_from.size()]);
          auto to = dba.Transfer(vertices_to[rand() % vertices_to.size()]);
          debug_assert(from, "From not visible in current GraphDbAccessor");
          debug_assert(to, "From not visible in current GraphDbAccessor");
          dba.insert_edge(from.value(), to.value(), edge_type);
          NotifyProgressListeners();
        },
        count, thread_count, batch_size);
  }

  /**
   * Returns the number of edges created by this generator,
   * regardless of their types and origin/destination labels.
   */
  int64_t EdgeCount() const { return dbms_.active()->edges_count(); }

  /**
   * Sets a generated property on a random vertex.
   *
   * @tparam TValue Type of value to set.
   * @param prop_name Name of the property.
   * @param predicate Filter that accepts or rejects a Vertex.
   * @param value_generator Function that accepts nothing and
   *    returns a property.
   */
  template <typename TValue>
  void SetVertexProperty(
      const std::string &prop_name, std::function<TValue()> value_generator,
      std::function<bool(VertexAccessor &va)> predicate = {}) {
    if (!predicate) predicate = [](VertexAccessor &) { return true; };
    auto dba = dbms_.active();
    auto property = dba->property(prop_name);
    for (VertexAccessor va : dba->vertices(false))
      if (predicate(va)) va.PropsSet(property, value_generator());
    dba->commit();
  }

 private:
  Dbms &dbms_;

  // progress listeners, they get notified about vertices and edges being
  // created
  std::vector<std::function<void(RandomGraphGenerator &)>> progress_listeners_;

  /**
   * Helper function for filtering.  Accepts a vector of TItems, a predicate
   * that accepts it or not, and returns a vector of reference wrappers to
   * accepted items.
   *
   *
   * @param predicate A predicate. By default always true.
   * @return A vector of vertex accessors. They belong to a GraphDbAccesor
   *   that is dead when this function retuns, make sure to
   *   GraphDbAccessor::Transfer them.
   */
  std::vector<VertexAccessor> FilterVertices(
      std::function<bool(VertexAccessor &item)> predicate = {}) {
    if (!predicate) predicate = [](VertexAccessor &) { return true; };
    std::vector<VertexAccessor> r_val;
    auto dba = dbms_.active();
    for (VertexAccessor &item : dba->vertices(false))
      if (predicate(item)) r_val.emplace_back(item);

    return r_val;
  }

  /** Sends notifications to all progress listeners */
  void NotifyProgressListeners() {
    for (const auto &listener : progress_listeners_) listener(*this);
  }

  /**
   * Performs function `f` `count` times across `thread_count`
   * threads. Returns only once all of the threads have
   * finished.
   */
  void Map(std::function<void(GraphDbAccessor &)> f, int count,
           int thread_count, int elements_per_commit) {
    debug_assert(thread_count > 0, "Can't work on less then 1 thread");

    // split count across thread_count
    int count_per_thread = count / thread_count;
    int count_remainder = count % thread_count;

    std::vector<std::thread> threads;
    for (int thread_ind = 0; thread_ind < thread_count; thread_ind++) {
      if (thread_ind == thread_count - 1) count_per_thread += count_remainder;
      threads.emplace_back([count_per_thread, &f, this, elements_per_commit]() {
        auto dba = dbms_.active();
        for (int i = 0; i < count_per_thread; i++) {
          try {
            f(*dba);
          } catch (LockTimeoutException &e) {
            i--;
            continue;
          } catch (mvcc::SerializationError &e) {
            i--;
            continue;
          }
          if (i == (count_per_thread - 1) ||
              (i >= 0 && i % elements_per_commit == 0)) {
            dba->commit();
            auto dba2 = dbms_.active();
            dba.swap(dba2);
          }
        }
      });
    }
    for (auto &thread : threads) thread.join();
  }
};
}

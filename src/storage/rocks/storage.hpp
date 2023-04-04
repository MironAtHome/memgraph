// Copyright 2023 Memgraph Ltd.
//
// Use of this software is governed by the Business Source License
// included in the file licenses/BSL.txt; by using this file, you agree to be bound by the terms of the Business Source
// License, and you may not use this file except in compliance with the Business Source License.
//
// As of the Change Date specified in that file, in accordance with
// the Business Source License, use of this software will be governed
// by the Apache License, Version 2.0, included in the file
// licenses/APL.txt.

#pragma once

#include <rocksdb/db.h>
#include <rocksdb/iterator.h>
#include <rocksdb/options.h>
#include <rocksdb/status.h>
#include <numeric>
#include <stdexcept>
#include <string>
#include <string_view>
#include "query/db_accessor.hpp"
#include "spdlog/spdlog.h"
#include "storage/rocks/loopback.hpp"
#include "storage/rocks/serialization.hpp"
#include "storage/v2/delta.hpp"
#include "storage/v2/id_types.hpp"
#include "storage/v2/storage.hpp"
#include "storage/v2/vertex.hpp"
#include "storage/v2/vertex_accessor.hpp"
#include "storage/v2/view.hpp"
#include "utils/algorithm.hpp"
#include "utils/exceptions.hpp"
#include "utils/logging.hpp"
#include "utils/string.hpp"

namespace memgraph::storage::rocks {

class RocksDBStorage {
 public:
  explicit RocksDBStorage() {
    options_.create_if_missing = true;
    options_.OptimizeLevelStyleCompaction();
    std::filesystem::path rocksdb_path = "./rocks_experiment_test";
    if (!memgraph::utils::EnsureDir(rocksdb_path)) {
      SPDLOG_ERROR("Unable to create storage folder on disk.");
      // TODO: throw some error
    }
    rocksdb::Status status = rocksdb::DB::Open(options_, rocksdb_path, &db_);
    if (!status.ok()) {
      spdlog::error(status.ToString());
    }
  }

  ~RocksDBStorage() {
    rocksdb::Status status = db_->Close();
    MG_ASSERT(status.ok());
    delete db_;
  }

  /*
  Serialize and store in-memory vertex to the disk.
  */
  bool StoreVertex(const query::VertexAccessor &vertex_acc) {
    const std::string key = SerializeVertex(vertex_acc);
    rocksdb::Status status = db_->Put(rocksdb::WriteOptions(), key, "properties");
    return status.ok();
  }

  /*
  Clear all entries from the database.
  */
  void Clear() {
    rocksdb::Iterator *it = db_->NewIterator(rocksdb::ReadOptions());
    for (it->SeekToFirst(); it->Valid(); it->Next()) {
      db_->Delete(rocksdb::WriteOptions(), it->key().ToString());
    }
  }

  /*
  Read all vertices stored in the database.
  */
  std::vector<std::unique_ptr<query::VertexAccessor>> Vertices(memgraph::query::DbAccessor &dba) {
    std::vector<std::unique_ptr<query::VertexAccessor>> vertices;
    rocksdb::Iterator *it = db_->NewIterator(rocksdb::ReadOptions());
    for (it->SeekToFirst(); it->Valid(); it->Next()) {
      std::string key = it->key().ToString();
      std::string value = it->value().ToString();
      spdlog::debug("Key: {} Value: {}", key, value);
      auto res = DeserializeVertex(key, dba);
      vertices.push_back(std::move(res));
    }
    return vertices;
  }

 protected:
  std::string SerializeLabels(auto &&labels) {
    if (labels.HasError() || (*labels).empty()) {
      return "";
    }
    std::string result = std::to_string((*labels)[0].AsUint());
    std::string ser_labels = std::accumulate(
        std::next((*labels).begin()), (*labels).end(), result,
        [](const std::string &join, const auto &label_id) { return join + "," + std::to_string(label_id.AsUint()); });
    return ser_labels;
  }

  std::string SerializeGid(const memgraph::storage::Gid &gid) { return std::to_string(gid.AsUint()); }

  std::string SerializeVertex(const query::VertexAccessor &vertex_acc) {
    // Serialize labels
    std::string result = SerializeLabels(vertex_acc.Labels(storage::View::OLD));
    result += "|";
    // Serialize gid
    result += SerializeGid(vertex_acc.Gid());
    spdlog::info("Serialized vertex: {}", result);
    return result;
  }

  std::unique_ptr<query::VertexAccessor> DeserializeVertex(const std::string_view key,
                                                           memgraph::query::DbAccessor &dba) {
    // Create vertex
    auto impl = std::make_unique<memgraph::query::VertexAccessor>(dba.InsertVertex());
    spdlog::info("Key to deserialize: {}", key);
    const auto vertex_parts = utils::Split(key, "|");
    const storage::Gid gid = storage::Gid::FromUint(std::stoull(vertex_parts[1]));
    impl->SetGid(gid);
    if (!vertex_parts[0].empty()) {
      const auto labels = utils::Split(vertex_parts[0], ",");
      for (const auto &label : labels) {
        const storage::LabelId label_id = storage::LabelId::FromUint(std::stoull(label));
        auto maybe_error = impl->AddLabel(label_id);
        if (maybe_error.HasError()) {
          switch (maybe_error.GetError()) {
            case storage::Error::SERIALIZATION_ERROR:
              throw utils::BasicException("Serialization");
            case storage::Error::DELETED_OBJECT:
              throw utils::BasicException("Trying to set a label on a deleted node.");
            case storage::Error::VERTEX_HAS_EDGES:
            case storage::Error::PROPERTIES_DISABLED:
            case storage::Error::NONEXISTENT_OBJECT:
              throw utils::BasicException("Unexpected error when setting a label.");
          }
        }
      }
    }
    return impl;
  }

 private:
  // rocksdb internals
  rocksdb::Options options_;
  rocksdb::DB *db_;
  // slk::Loopback loopback_;
  // slk::Encoder encoder_{loopback_.GetBuilder()};
  // slk::Decoder decoder_{loopback_.GetReader()};
};

}  // namespace memgraph::storage::rocks

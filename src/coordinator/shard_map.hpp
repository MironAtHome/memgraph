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

#include <algorithm>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <vector>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include "common/types.hpp"
#include "coordinator/hybrid_logical_clock.hpp"
#include "io/address.hpp"
#include "query/v2/requests.hpp"
#include "storage/v3/config.hpp"
#include "storage/v3/id_types.hpp"
#include "storage/v3/name_id_mapper.hpp"
#include "storage/v3/property_value.hpp"
#include "storage/v3/schemas.hpp"
#include "storage/v3/temporal.hpp"
#include "utils/exceptions.hpp"
#include "utils/print_helpers.hpp"

namespace memgraph::coordinator {

constexpr int64_t kNotExistingId{0};

using memgraph::io::Address;
using memgraph::storage::v3::Config;
using memgraph::storage::v3::EdgeTypeId;
using memgraph::storage::v3::LabelId;
using memgraph::storage::v3::PropertyId;
using memgraph::storage::v3::PropertyValue;
using memgraph::storage::v3::SchemaProperty;

enum class Status : uint8_t {
  CONSENSUS_PARTICIPANT,
  INITIALIZING,
  PENDING_SPLIT,
  // TODO(tyler) this will possibly have more states,
  // depending on the reconfiguration protocol that we
  // implement.
};

struct PeerMetadata {
  memgraph::io::Address address;
  Status status;
  boost::uuids::uuid split_from;

  friend bool operator<(const PeerMetadata &lhs, const PeerMetadata &rhs) { return lhs.address < rhs.address; }

  friend std::ostream &operator<<(std::ostream &in, const PeerMetadata &peer_metadata) {
    in << "PeerMetadata { address: ";
    in << peer_metadata.address;

    if (peer_metadata.status == Status::CONSENSUS_PARTICIPANT) {
      in << ", status: CONSENSUS_PARTICIPANT";
    } else if (peer_metadata.status == Status::INITIALIZING) {
      in << ", status: INITIALIZING";
    } else if (peer_metadata.status == Status::PENDING_SPLIT) {
      in << ", status: PENDING_SPLIT";
    } else {
      MG_ASSERT(false, "failed to update the operator<< implementation for Status");
    }

    in << ", split_from: " << peer_metadata.split_from << " }";

    return in;
  }

  friend bool operator==(const PeerMetadata &lhs, const PeerMetadata &rhs) { return lhs.address == rhs.address; }
};

using PrimaryKey = std::vector<PropertyValue>;

struct ShardMetadata {
  std::vector<PeerMetadata> peers;
  Hlc version;
  std::optional<msgs::SuggestedSplitInfo> pending_split;

  friend std::ostream &operator<<(std::ostream &in, const ShardMetadata &shard) {
    using utils::print_helpers::operator<<;

    in << "ShardMetadata { peers: ";
    in << shard.peers;
    in << " version: ";
    in << shard.version;
    in << " }";

    return in;
  }

  friend bool operator==(const ShardMetadata &lhs, const ShardMetadata &rhs) {
    return lhs.version == rhs.version && lhs.peers == rhs.peers;
  };

  friend bool operator<(const ShardMetadata &lhs, const ShardMetadata &rhs) {
    if (lhs.peers != rhs.peers) {
      return lhs.peers < rhs.peers;
    }

    return lhs.version < rhs.version;
  }
};

using Shards = std::map<PrimaryKey, ShardMetadata>;
using LabelName = std::string;
using PropertyName = std::string;
using EdgeTypeName = std::string;
using PropertyMap = std::map<PropertyName, PropertyId>;
using EdgeTypeIdMap = std::map<EdgeTypeName, EdgeTypeId>;

struct ShardToInitialize {
  boost::uuids::uuid uuid;
  LabelId label_id;
  PrimaryKey min_key;
  std::optional<PrimaryKey> max_key;
  std::vector<SchemaProperty> schema;
  Config config;
  std::unordered_map<uint64_t, std::string> id_to_names;
};

struct ShardToSplit {
  PrimaryKey split_key;
  Hlc old_shard_version;
  Hlc new_shard_version;
  std::map<boost::uuids::uuid, boost::uuids::uuid> uuid_mapping;
};

struct HeartbeatRequest {
  Address from_storage_manager;
  std::set<boost::uuids::uuid> initialized_rsms;
  std::set<msgs::SuggestedSplitInfo> pending_splits;
};

struct HeartbeatResponse {
  std::vector<ShardToInitialize> shards_to_initialize;
  std::vector<ShardToSplit> shards_to_split;
};

PrimaryKey SchemaToMinKey(const std::vector<SchemaProperty> &schema);

struct LabelSpace {
  std::vector<SchemaProperty> schema;
  // Maps between the smallest primary key stored in the shard and the shard
  std::map<PrimaryKey, ShardMetadata> shards;
  size_t replication_factor;

  friend std::ostream &operator<<(std::ostream &in, const LabelSpace &label_space) {
    using utils::print_helpers::operator<<;

    in << "LabelSpace { schema: ";
    in << label_space.schema;
    in << ", shards: ";
    in << label_space.shards;
    in << ", replication_factor: " << label_space.replication_factor << "}";

    return in;
  }
};

struct ShardMap {
  Hlc shard_map_version;
  uint64_t max_property_id{kNotExistingId};
  uint64_t max_edge_type_id{kNotExistingId};
  std::map<PropertyName, PropertyId> properties;
  std::map<EdgeTypeName, EdgeTypeId> edge_types;
  uint64_t max_label_id{kNotExistingId};
  std::map<LabelId, LabelSpace> label_spaces;
  std::map<LabelId, std::vector<SchemaProperty>> schemas;
  std::map<LabelName, LabelId> labels;

  [[nodiscard]] static ShardMap Parse(std::istream &input_stream);
  friend std::ostream &operator<<(std::ostream &in, const ShardMap &shard_map);

  Shards GetShardsForLabel(const LabelName &label) const;

  std::vector<Shards> GetAllShards() const;

  // TODO(gabor) later we will want to update the wallclock time with
  // the given Io<impl>'s time as well
  Hlc IncrementShardMapVersion() noexcept;
  Hlc GetHlc() noexcept;

  std::unordered_map<uint64_t, std::string> IdToNames();

  // Returns the shard UUIDs that have been assigned but not yet acknowledged for this storage manager
  HeartbeatResponse AssignShards(Address storage_manager, std::set<boost::uuids::uuid> initialized,
                                 std::set<msgs::SuggestedSplitInfo> pending_splits);

  bool SplitShard(Hlc previous_shard_map_version, LabelId label_id, const PrimaryKey &key);

  std::optional<LabelId> InitializeNewLabel(std::string label_name, std::vector<SchemaProperty> schema,
                                            size_t replication_factor, Hlc last_shard_map_version);

  boost::uuids::uuid NewShardUuid();

  void AddServer(Address server_address);

  std::optional<LabelId> GetLabelId(const std::string &label) const;
  // TODO(antaljanosbenjamin): Remove this and instead use NameIdMapper
  const std::string &GetLabelName(LabelId label) const;
  std::optional<PropertyId> GetPropertyId(const std::string &property_name) const;
  const std::string &GetPropertyName(PropertyId property) const;
  std::optional<EdgeTypeId> GetEdgeTypeId(const std::string &edge_type) const;
  const std::string &GetEdgeTypeName(EdgeTypeId property) const;

  Shards GetShardsForRange(const LabelName &label_name, const PrimaryKey &start_key, const PrimaryKey &end_key) const;

  ShardMetadata GetShardForKey(const LabelName &label_name, const PrimaryKey &key) const;

  ShardMetadata GetShardForKey(const LabelId &label_id, const PrimaryKey &key) const;

  PropertyMap AllocatePropertyIds(const std::vector<PropertyName> &new_properties);

  EdgeTypeIdMap AllocateEdgeTypeIds(const std::vector<EdgeTypeName> &new_edge_types);

  /// Returns true if all shards have the desired number of replicas and they are in
  /// the CONSENSUS_PARTICIPANT state. Note that this does not necessarily mean that
  /// there is also an active leader for each shard.
  bool ClusterInitialized() const;
};

}  // namespace memgraph::coordinator

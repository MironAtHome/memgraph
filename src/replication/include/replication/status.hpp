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

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>

#include "json/json.hpp"

#include "replication/config.hpp"
#include "replication/epoch.hpp"
#include "replication/role.hpp"

namespace memgraph::replication::durability {

// Keys
constexpr auto *kReplicationRoleName{"__replication_role"};
constexpr auto *kReplicationReplicaPrefix{"__replication_replica:"};  // introduced in V2

#ifdef MG_ENTERPRISE
constexpr auto *kReplicationMainPrefix("__replication_main:");
#endif

enum class DurabilityVersion : uint8_t {
  V1,  // no distinct key for replicas
  V2,  // this version, epoch, replica prefix introduced
};

// fragment of key: "__replication_role"
struct MainRole {
  ReplicationEpoch epoch{};
#ifdef MG_ENTERPRISE
  ReplicationServerConfig config{};
#endif
  friend bool operator==(MainRole const &, MainRole const &) = default;
};

// fragment of key: "__replication_role"
struct ReplicaRole {
  ReplicationServerConfig config{};
  friend bool operator==(ReplicaRole const &, ReplicaRole const &) = default;
};

#ifdef MG_ENTERPRISE
// fragment of key: "__replication_role"
// TODO: (andi) Check if you will need to add some epoch or something for managing coordinator role
struct CoordinatorRole {
  friend bool operator==(CoordinatorRole const &, CoordinatorRole const &) = default;
};
#endif

// from key: "__replication_role"
struct ReplicationRoleEntry {
  DurabilityVersion version =
      DurabilityVersion::V2;  // if not latest then migration required for kReplicationReplicaPrefix

#ifdef MG_ENTERPRISE
  std::variant<MainRole, ReplicaRole, CoordinatorRole> role;
#else
  std::variant<MainRole, ReplicaRole> role;
#endif

  friend bool operator==(ReplicationRoleEntry const &, ReplicationRoleEntry const &) = default;
};

// used for main's and replicas' clients
struct ReplicationClientConfigEntry {
  ReplicationClientConfig config;
  friend bool operator==(ReplicationClientConfigEntry const &, ReplicationClientConfigEntry const &) = default;
};

void to_json(nlohmann::json &j, const ReplicationRoleEntry &p);
void from_json(const nlohmann::json &j, ReplicationRoleEntry &p);

void to_json(nlohmann::json &j, const ReplicationClientConfigEntry &p);
void from_json(const nlohmann::json &j, ReplicationClientConfigEntry &p);
}  // namespace memgraph::replication::durability

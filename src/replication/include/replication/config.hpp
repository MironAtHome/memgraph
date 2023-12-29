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
#include "replication/mode.hpp"

namespace memgraph::replication {

inline constexpr uint16_t kDefaultReplicationPort = 10000;
inline constexpr auto *kDefaultReplicationServerIp = "0.0.0.0";

#ifdef MG_ENTERPRISE
// Default name which coordinator uses to distinguish main's ReplicationClient from replicas'.
inline constexpr auto *kDefaultMainName = "main";
#endif

struct ReplicationClientConfig {
  std::string name;
#ifdef MG_ENTERPRISE
  std::optional<ReplicationMode> mode;
#else
  ReplicationMode mode{};
#endif
  std::string ip_address;
  uint16_t port{};

  // The default delay between coordinator/main checking/pinging main and replicas/replicas is 1s because
  // that seems like a reasonable timeframe in which coordinator/main should notice a
  // main or replica is down.
  std::chrono::seconds check_frequency{1};

  struct SSL {
    std::string key_file;
    std::string cert_file;

    friend bool operator==(const SSL &, const SSL &) = default;
  };

  std::optional<SSL> ssl;

  friend bool operator==(ReplicationClientConfig const &, ReplicationClientConfig const &) = default;
};

struct ReplicationServerConfig {
  std::string ip_address;
  uint16_t port{};
  struct SSL {
    std::string key_file;
    std::string cert_file;
    std::string ca_file;
    bool verify_peer{};
    friend bool operator==(SSL const &, SSL const &) = default;
  };

  std::optional<SSL> ssl;

  friend bool operator==(ReplicationServerConfig const &, ReplicationServerConfig const &) = default;
};
}  // namespace memgraph::replication

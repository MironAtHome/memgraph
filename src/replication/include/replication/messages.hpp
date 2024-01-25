// Copyright 2024 Memgraph Ltd.
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

#include "rpc/messages.hpp"
#include "slk/serialization.hpp"

namespace memgraph::replication {
struct SystemHeartbeatReq {
  static const utils::TypeInfo kType;
  static const utils::TypeInfo &GetTypeInfo() { return kType; }

  static void Load(SystemHeartbeatReq *self, memgraph::slk::Reader *reader);
  static void Save(const SystemHeartbeatReq &self, memgraph::slk::Builder *builder);
  SystemHeartbeatReq() = default;
};

struct SystemHeartbeatRes {
  static const utils::TypeInfo kType;
  static const utils::TypeInfo &GetTypeInfo() { return kType; }

  static void Load(SystemHeartbeatRes *self, memgraph::slk::Reader *reader);
  static void Save(const SystemHeartbeatRes &self, memgraph::slk::Builder *builder);
  SystemHeartbeatRes() = default;
  explicit SystemHeartbeatRes(uint64_t system_timestamp) : system_timestamp(system_timestamp) {}

  uint64_t system_timestamp;
};

using SystemHeartbeatRpc = rpc::RequestResponse<SystemHeartbeatReq, SystemHeartbeatRes>;
}  // namespace memgraph::replication

namespace memgraph::slk {
void Save(const memgraph::replication::SystemHeartbeatRes &self, memgraph::slk::Builder *builder);
void Load(memgraph::replication::SystemHeartbeatRes *self, memgraph::slk::Reader *reader);
void Save(const memgraph::replication::SystemHeartbeatReq & /*self*/, memgraph::slk::Builder * /*builder*/);
void Load(memgraph::replication::SystemHeartbeatReq * /*self*/, memgraph::slk::Reader * /*reader*/);
}  // namespace memgraph::slk
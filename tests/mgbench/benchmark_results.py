# Copyright 2023 Memgraph Ltd.
#
# Use of this software is governed by the Business Source License
# included in the file licenses/BSL.txt; by using this file, you agree to be bound by the terms of the Business Source
# License, and you may not use this file except in compliance with the Business Source License.
#
# As of the Change Date specified in that file, in accordance with
# the Business Source License, use of this software will be governed
# by the Apache License, Version 2.0, included in the file
# licenses/APL.txt.

import helpers


class BenchmarkResults:
    def __init__(self) -> None:
        self.in_memory_txn_results = helpers.RecursiveDict()
        self.in_memory_analytical_results = helpers.RecursiveDict()
        self.disk_results = helpers.RecursiveDict()

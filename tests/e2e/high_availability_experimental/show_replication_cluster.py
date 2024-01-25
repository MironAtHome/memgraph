# Copyright 2022 Memgraph Ltd.
#
# Use of this software is governed by the Business Source License
# included in the file licenses/BSL.txt; by using this file, you agree to be bound by the terms of the Business Source
# License, and you may not use this file except in compliance with the Business Source License.
#
# As of the Change Date specified in that file, in accordance with
# the Business Source License, use of this software will be governed
# by the Apache License, Version 2.0, included in the file
# licenses/APL.txt.

import os
import sys

import interactive_mg_runner
import pytest
from common import execute_and_fetch_all
from mg_utils import mg_sleep_and_assert

interactive_mg_runner.SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
interactive_mg_runner.PROJECT_DIR = os.path.normpath(
    os.path.join(interactive_mg_runner.SCRIPT_DIR, "..", "..", "..", "..")
)
interactive_mg_runner.BUILD_DIR = os.path.normpath(os.path.join(interactive_mg_runner.PROJECT_DIR, "build"))
interactive_mg_runner.MEMGRAPH_BINARY = os.path.normpath(os.path.join(interactive_mg_runner.BUILD_DIR, "memgraph"))

MEMGRAPH_INSTANCES_DESCRIPTION = {
    "instance_1": {
        "args": ["--bolt-port", "7688", "--log-level", "TRACE", "--coordinator-server-port", "10011"],
        "log_file": "replica1.log",
        "setup_queries": ["SET REPLICATION ROLE TO REPLICA WITH PORT 10001;"],
    },
    "instance_2": {
        "args": ["--bolt-port", "7689", "--log-level", "TRACE", "--coordinator-server-port", "10012"],
        "log_file": "replica2.log",
        "setup_queries": ["SET REPLICATION ROLE TO REPLICA WITH PORT 10002;"],
    },
    "instance_3": {
        "args": ["--bolt-port", "7687", "--log-level", "TRACE", "--coordinator-server-port", "10013"],
        "log_file": "main.log",
        "setup_queries": [
            "REGISTER REPLICA instance_1 SYNC TO '127.0.0.1:10001'",
            "REGISTER REPLICA instance_2 SYNC TO '127.0.0.1:10002'",
        ],
    },
    "coordinator": {
        "args": ["--bolt-port", "7690", "--log-level=TRACE", "--coordinator"],
        "log_file": "replica3.log",
        "setup_queries": [
            "REGISTER REPLICA instance_1 SYNC TO '127.0.0.1:10001' WITH COORDINATOR SERVER ON '127.0.0.1:10011';",
            "REGISTER REPLICA instance_2 SYNC TO '127.0.0.1:10002' WITH COORDINATOR SERVER ON '127.0.0.1:10012';",
            "REGISTER MAIN instance_3 WITH COORDINATOR SERVER ON '127.0.0.1:10013';",
        ],
    },
}


def test_show_replication_cluster(connection):
    # Goal of this test is to check the SHOW REPLICATION CLUSTER command.
    # 1. We start all replicas, main and coordinator manually: we want to be able to kill them ourselves without relying on external tooling to kill processes.
    # 2. We check that all replicas and main have the correct state: they should all be alive.
    # 3. We kill one replica. It should not appear anymore in the SHOW REPLICATION CLUSTER command.
    # 4. We kill main. It should not appear anymore in the SHOW REPLICATION CLUSTER command.

    # 1.
    interactive_mg_runner.start_all(MEMGRAPH_INSTANCES_DESCRIPTION)

    cursor = connection(7690, "coordinator").cursor()

    # 2.

    # We leave some time for the coordinator to realise the replicas are down.
    def retrieve_data():
        return sorted(list(execute_and_fetch_all(cursor, "SHOW REPLICATION CLUSTER;")))

    expected_data = [
        ("instance_1", "127.0.0.1:10011", True, "replica"),
        ("instance_2", "127.0.0.1:10012", True, "replica"),
        ("instance_3", "127.0.0.1:10013", True, "main"),
    ]
    mg_sleep_and_assert(expected_data, retrieve_data)

    # 3.
    interactive_mg_runner.kill(MEMGRAPH_INSTANCES_DESCRIPTION, "instance_1")

    expected_data = [
        ("instance_1", "127.0.0.1:10011", False, "replica"),
        ("instance_2", "127.0.0.1:10012", True, "replica"),
        ("instance_3", "127.0.0.1:10013", True, "main"),
    ]
    mg_sleep_and_assert(expected_data, retrieve_data)

    # 4.
    interactive_mg_runner.kill(MEMGRAPH_INSTANCES_DESCRIPTION, "instance_3")

    expected_data = [
        ("instance_1", "127.0.0.1:10011", False, "replica"),
        ("instance_2", "127.0.0.1:10012", True, "replica"),
        ("instance_3", "127.0.0.1:10013", False, "main"),
    ]
    mg_sleep_and_assert(expected_data, retrieve_data)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-rA"]))
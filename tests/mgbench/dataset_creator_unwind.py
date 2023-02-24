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

import argparse
import random

import helpers

# Explaination of datasets:
#   - empty_only_index: contains index; contains no data
#   - small: contains index; contains data (small dataset)
#
# Datamodel is as follow:
#
#                               ┌──────────────┐
#                               │ Permission   │
#         ┌────────────────┐    │  Schema:uuid │   ┌────────────┐
#         │:IS_FOR_IDENTITY├────┤  Index:name  ├───┤:IS_FOR_FILE│
#         └┬───────────────┘    └──────────────┘   └────────────┤
#          │                                                    │
#   ┌──────▼──────────────┐                                  ┌──▼────────────────┐
#   │  Identity           │                                  │ File              │
#   │   Schema:uuid       │                                  │  Schema:uuid      │
#   │   Index:email       │                                  │  Index:name       │
#   └─────────────────────┘                                  │  Index:platformId │
#                                                            └───────────────────┘
#
#   - File: attributes: ["uuid", "name", "platformId"]
#   - Permission: attributes: ["uuid", "name"]
#   - Identity: attributes: ["uuid", "email"]
#
# Indexes:
#   - File: [File(uuid), File(platformId), File(name)]
#   - Permission: [Permission(uuid), Permission(name)]
#   - Identity: [Identity(uuid), Identity(email)]
#
# Edges:
#   - (:Permission)-[:IS_FOR_FILE]->(:File)
#   - (:Permission)-[:IS_FOR_IDENTITYR]->(:Identity)
#
# AccessControl specific: uuid is the schema


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--number_of_identities", type=int, default=10)
    parser.add_argument("--number_of_files", type=int, default=10)
    parser.add_argument("--percentage_of_permissions", type=float, default=1.0)
    parser.add_argument("--filename", default="dataset.cypher")

    args = parser.parse_args()

    number_of_identities = args.number_of_identities
    number_of_files = args.number_of_files
    percentage_of_permissions = args.percentage_of_permissions
    filename = args.filename

    assert number_of_identities >= 0
    assert number_of_files >= 0
    assert percentage_of_permissions > 0.0 and percentage_of_permissions <= 1.0
    assert filename != ""

    with open(filename, "w") as f:
        f.write("MATCH (n) DETACH DELETE n;\n")

        # Create the indexes
        f.write("CREATE INDEX ON :File;\n")
        f.write("CREATE INDEX ON :Permission;\n")
        f.write("CREATE INDEX ON :Identity;\n")
        f.write("CREATE INDEX ON :File(platformId);\n")
        f.write("CREATE INDEX ON :File(name);\n")
        f.write("CREATE INDEX ON :Permission(name);\n")
        f.write("CREATE INDEX ON :Identity(email);\n")

        # Create extra index: in distributed, this will be the schema
        f.write("CREATE INDEX ON :File(uuid);\n")
        f.write("CREATE INDEX ON :Permission(uuid);\n")
        f.write("CREATE INDEX ON :Identity(uuid);\n")

        uuid = 1

        # Create the nodes File
        f.write("UNWIND [")
        for index in range(0, number_of_files):
            if index != 0:
                f.write(",")
            f.write(f' {{uuid: {uuid}, platformId: "platform_id", name: "name_file_{uuid}"}}')
            uuid += 1
        f.write("] AS props CREATE (:File {uuid: props.uuid, platformId: props.platformId, name: props.name});\n")

        identities = []
        f.write("UNWIND [")
        # Create the nodes Identity
        for index in range(0, number_of_identities):
            if index != 0:
                f.write(",")
            f.write(f' {{uuid: {uuid}, name: "mail_{uuid}@something.com"}}')
            uuid += 1
        f.write("] AS props CREATE (:Identity {uuid: props.uuid, name: props.name});\n")

        f.write("UNWIND [")
        created = 0
        for outer_index in range(0, number_of_files):
            for inner_index in range(0, number_of_identities):

                file_uuid = outer_index + 1
                identity_uuid = number_of_files + inner_index + 1

                if random.random() <= percentage_of_permissions:

                    if created > 0:
                        f.write(",")

                    f.write(
                        f' {{permUuid: {uuid}, permName: "name_permission_{uuid}", fileUuid: {file_uuid}, identityUuid: {identity_uuid}}}'
                    )
                    created += 1
                    uuid += 1

                    if created == 5000:
                        f.write(
                            "] AS props MATCH (file:File {uuid:props.fileUuid}), (identity:Identity {uuid: props.identityUuid}) CREATE (permission:Permission {uuid: props.permUuid, name: props.permName}) CREATE (permission)-[: IS_FOR_FILE]->(file) CREATE (permission)-[: IS_FOR_IDENTITY]->(identity);\nUNWIND ["
                        )
                        created = 0
        f.write(
            "] AS props MATCH (file:File {uuid:props.fileUuid}), (identity:Identity {uuid: props.identityUuid}) CREATE (permission:Permission {uuid: props.permUuid, name: props.permName}) CREATE (permission)-[: IS_FOR_FILE]->(file) CREATE (permission)-[: IS_FOR_IDENTITY]->(identity);\n"
        )


if __name__ == "__main__":
    main()
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

#include <cstdint>
#include "strong_type/strong_type.hpp"
#include "utils/logging.hpp"

namespace memgraph::storage {

using Srid = strong::type<uint16_t, struct Srid_, strong::regular, strong::partially_ordered>;

enum class CoordinateReferenceSystem : uint8_t {
  WGS84_2d,
  WGS84_3d,
  Cartesian_2d,
  Cartesian_3d,
};

constexpr auto kSrid_WGS85_2D = 4326;
constexpr auto kSrid_WGS85_3D = 4979;
constexpr auto kSrid_Cartesian_2D = 7203;
constexpr auto kSrid_Cartesian_3D = 9757;

auto CrsToSrid(CoordinateReferenceSystem val) -> Srid {
  switch (val) {
    using enum CoordinateReferenceSystem;
    case WGS84_2d:
      return Srid{kSrid_WGS85_2D};
    case WGS84_3d:
      return Srid{kSrid_WGS85_3D};
    case Cartesian_2d:
      return Srid{kSrid_Cartesian_2D};
    case Cartesian_3d:
      return Srid{kSrid_Cartesian_3D};
  }
};

auto SridToCrs(Srid val) -> std::optional<CoordinateReferenceSystem> {
  switch (val.value_of()) {
    using enum CoordinateReferenceSystem;
    case kSrid_WGS85_2D:
      return WGS84_2d;
    case kSrid_WGS85_3D:
      return WGS84_3d;
    case kSrid_Cartesian_2D:
      return Cartesian_2d;
    case kSrid_Cartesian_3D:
      return Cartesian_3d;
  }
  return std::nullopt;
}

namespace {
bool valid2d(CoordinateReferenceSystem val) {
  using enum CoordinateReferenceSystem;
  return val == WGS84_2d || val == Cartesian_2d;
}
bool valid3d(CoordinateReferenceSystem val) {
  using enum CoordinateReferenceSystem;
  return val == WGS84_3d || val == Cartesian_3d;
}
}  // namespace

struct Point2d {
  Point2d() = default;  // needed for slk

  Point2d(CoordinateReferenceSystem crs, double x, double y) : crs_{crs}, x_{x}, y_{y} {
    DMG_ASSERT(valid2d(crs), "Not a valid 2d Coordinate Reference System");
  }

  auto crs() const -> CoordinateReferenceSystem { return crs_; }
  auto x() const -> double { return x_; }
  auto y() const -> double { return y_; }

  // TODO Ivan: Check how to print CRS
  static std::string ToString(const Point2d &point) { return fmt::format("{}, {}", point.x_, point.y_); }

  friend auto operator<(Point2d const &A, Point2d const &B) -> bool {
    // TODO Ivan: check that crs is in right order
    if (A.crs_ != B.crs_) return A.crs_ < B.crs_;
    return A.x_ < B.x_ && A.y_ < B.y_;
  }

  friend auto operator==(Point2d const &A, Point2d const &B) -> bool {
    return A.crs_ == B.crs_ && A.x_ == B.x_ && A.y_ == B.y_;
  }

  friend std::ostream &operator<<(std::ostream &os, const Point2d &point) {
    return os << memgraph::storage::Point2d::ToString(point);
  }

 private:
  CoordinateReferenceSystem crs_;
  double x_;
  double y_;
};

struct Point3d {
  Point3d() = default;  // needed for slk

  Point3d(CoordinateReferenceSystem crs, double x, double y, double z) : crs_{crs}, x_{x}, y_{y}, z_{z} {
    DMG_ASSERT(valid3d(crs), "Not a valid 3d Coordinate Reference System");
  }

  auto crs() const -> CoordinateReferenceSystem { return crs_; }
  auto x() const -> double { return x_; }
  auto y() const -> double { return y_; }
  auto z() const -> double { return z_; }

  // TODO Ivan: Check how to print CRS
  static std::string ToString(const Point3d &point) { return fmt::format("{}, {} {}", point.x_, point.y_, point.z_); }

  friend auto operator<(Point3d const &A, Point3d const &B) -> bool {
    // TODO Ivan: check that crs is in right order
    if (A.crs_ != B.crs_) return A.crs_ < B.crs_;
    return A.x_ < B.x_ && A.y_ < B.y_ && A.z_ < B.z_;
  }

  friend auto operator==(Point3d const &A, Point3d const &B) -> bool {
    return A.crs_ == B.crs_ && A.x_ == B.x_ && A.y_ == B.y_ && A.z_ == B.z_;
  }

  friend std::ostream &operator<<(std::ostream &os, const Point3d &point) {
    return os << memgraph::storage::Point3d::ToString(point);
  }

 private:
  CoordinateReferenceSystem crs_;
  double x_;
  double y_;
  double z_;
};

static_assert(std::is_trivially_destructible_v<Point2d>);
static_assert(std::is_trivially_destructible_v<Point3d>);
}  // namespace memgraph::storage

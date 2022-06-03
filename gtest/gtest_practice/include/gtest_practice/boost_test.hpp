#pragma once

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/assert.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/assign/list_of.hpp>

namespace behavior_velocity_planner
{

namespace bg = boost::geometry;
struct Point3d;
using Point2d = boost::geometry::model::d2::point_xy<double>;
using LineString2d = boost::geometry::model::linestring<Point2d>;
using LineString3d = boost::geometry::model::linestring<Point3d>;
using Polygon2d =
  boost::geometry::model::polygon<Point2d, false, false>;  // counter-clockwise, open
struct Point3d : public Eigen::Vector3d
{
  Point3d() = default;
  Point3d(const double x, const double y, const double z) : Eigen::Vector3d(x, y, z) {}

  [[nodiscard]] Point2d to_2d() const;
};
}

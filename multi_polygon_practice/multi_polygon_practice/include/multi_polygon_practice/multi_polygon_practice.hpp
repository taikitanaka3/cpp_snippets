#pragma once
#include <bits/stdc++.h>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <geometry_msgs/msg/pose.hpp>

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

struct PolarCoordinates {
  double radius;
  double theta;
};

struct CartesianCoordinates {
  double x;
  double y;
};

PolarCoordinates toPolarCoordinates(const Point &origin, const Point &point) {
  const double dy = point.y - origin.y;
  const double dx = point.x - origin.x;
  const double radius = std::hypot(dy, dx);
  const double theta = std::atan2(dy, dx);
  return {radius, theta};
}

CartesianCoordinates fromPolarCoordinates() {
  CartesianCoordinates cart_coord;
  return cart_coord;
}

namespace multi_polygon_practice {

namespace bg = boost::geometry;
struct Point3d;
using Point2d = boost::geometry::model::d2::point_xy<double>;
using LineString2d = boost::geometry::model::linestring<Point2d>;
using LineString3d = boost::geometry::model::linestring<Point3d>;
using Polygon2d =
    boost::geometry::model::polygon<Point2d, false,
                                    false>; // counter-clockwise, open
using Polygons2d = std::vector<Polygon2d>;
std::vector<Polygon2d> daPoly() {
  Polygon2d poly, bx;
  bg::exterior_ring(poly) =
      boost::assign::list_of<Point2d>(1, 1)(5, 5)(5, 1)(1, 1);
  bg::exterior_ring(bx) = boost::assign::list_of<Point2d>
        (2, 1)
        (5, 5)
        (5, 1)
        (1, 1)
        ;

  std::vector<Polygon2d> out;
  bg::union_(bx, poly, out);
  return out;
}
struct Point3d : public Eigen::Vector3d {
  Point3d() = default;
  Point3d(const double x, const double y, const double z)
      : Eigen::Vector3d(x, y, z) {}

  [[nodiscard]] Point2d to_2d() const;
};

} // namespace multi_polygon_practice
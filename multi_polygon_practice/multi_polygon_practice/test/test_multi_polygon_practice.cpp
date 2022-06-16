#include <bits/stdc++.h>
#include <boost/assign/list_of.hpp>
#include <gtest/gtest.h>
#include <multi_polygon_practice/multi_polygon_practice.hpp>
using namespace multi_polygon_practice;

TEST(boost_test, intersects) {
  Polygons2d polys;
  Polygon2d occupancy_poly;
  //    |xx/|
  //    |o/ |
  //    +/  |
  //        |
  boost::geometry::read_wkt("polygon((-10 -10, 10 -10, 10 10, -10 10))",
                            occupancy_poly);
  std::cout << boost::geometry::wkt(occupancy_poly) << std::endl;

  Point2d ego_pos = {0, 0};
  Point2d obj_min_theta_pos = {2, 2};
  Point2d obj_max_theta_pos = {0, 2};
  const double obj_min_theta = std::atan2(obj_min_theta_pos.y() - ego_pos.y(),
                                          obj_min_theta_pos.x() - ego_pos.x());
  const double obj_max_theta = std::atan2(obj_max_theta_pos.y() - ego_pos.y(),
                                          obj_max_theta_pos.x() - ego_pos.x());
  const double ray_max_length = 50.0;
  LineString2d theta_min_ray = {ego_pos,
                                {ray_max_length * std::cos(obj_min_theta),
                                 ray_max_length * std::sin(obj_min_theta)}};
  LineString2d theta_max_ray = {ego_pos,
                                {ray_max_length * std::cos(obj_max_theta),
                                 ray_max_length * std::sin(obj_max_theta)}};
  std::vector<Point2d> min_intersections, max_intersections;
  bg::intersection(occupancy_poly, theta_min_ray, min_intersections);
  std::cout << boost::geometry::wkt(min_intersections.at(0)) << std::endl;
  bg::intersection(occupancy_poly, theta_max_ray, max_intersections);
  std::cout << boost::geometry::wkt(max_intersections.at(0)) << std::endl;

  Polygon2d occlusion_poly;
  occlusion_poly.outer() = {obj_min_theta_pos, min_intersections.at(0),
                            max_intersections.at(0), obj_max_theta_pos};
  std::cout << boost::geometry::wkt(occlusion_poly) << std::endl;
  occlusion_poly.outer() = {min_intersections.at(0), obj_min_theta_pos,
                            max_intersections.at(0), obj_max_theta_pos};
  boost::geometry::correct(occlusion_poly);
  Polygon2d hull;
  boost::geometry::convex_hull(occlusion_poly, hull);
  std::cout << boost::geometry::wkt(occlusion_poly) << std::endl;
  std::cout << boost::geometry::wkt(hull) << std::endl;
  {
    namespace bg = boost::geometry;
    typedef bg::model::d2::point_xy<double> point;
    typedef bg::model::polygon<point> polygon;
    polygon occ;
    bg::exterior_ring(occ) = boost::assign::list_of<point>(1, 2)(2, 3);
    // std::cout << boost::geometry::wkt(occ) << std::endl;
  }
}

TEST(boost_test, boost_test) {
  Polygons2d polys;
  LineString2d l1;
  Polygon2d poly;
  boost::geometry::read_wkt(
      "polygon((2.0 1.3, 2.4 1.7, 2.8 1.8, 3.4 1.2, 3.7 1.6, 3.4 2.0, 4.1 3.0, "
      "5.3 2.6, 5.4 1.2, 4.9 0.8, 2.9 0.7, 2.0 1.3))",
      poly);
  std::cout << boost::geometry::wkt(poly) << std::endl;
  l1.emplace_back(Point2d{0, 2});
  l1.emplace_back(Point2d{2, 2});

  //  line2
  //    |
  // ---+---- line1
  //    |
  {
    LineString2d l2;
    l2.emplace_back(Point2d{1, 0});
    l2.emplace_back(Point2d{1, 3});
    const bool result = bg::intersects(l1, l2);
    EXPECT_TRUE(result); // 交点を持っている
  }
  // -------- line1
  // -------- line2
  {
    LineString2d l2;
    l2.emplace_back(Point2d{0, 0});
    l2.emplace_back(Point2d{2, 0});
    const bool result = bg::intersects(l1, l2);
    EXPECT_FALSE(result); // 交点を持っている
  }
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

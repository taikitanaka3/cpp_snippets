#include <bits/stdc++.h>
#include <boost/optional.hpp>
#include <boost_reference/boost_reference.hpp>
#include <gtest/gtest.h>

boost::optional<double> returnEmpty() { return {}; }

boost::optional<size_t> returnNone() { return boost::none; }

TEST(test, nominal) {
  {
    boost::optional<double> a;
    boost::optional<double> b = boost::none;
    EXPECT_EQ(a, b);
  }

  {
    auto a = returnEmpty();
    boost::optional<double> b = boost::none;
    EXPECT_EQ(a, b);
  }
}

TEST(test, compare) {
  // fail
  // {
  // auto seg_idx = returnNone();
  // const size_t idx = *seg_idx ? *seg_idx + 1 : 0;
  // std::cout<<idx<<std::endl;
  // }

  // ok
  {
    auto seg_idx = returnNone();
    const size_t idx = seg_idx ? *seg_idx + 1 : 0;
    std::cout << idx << std::endl;
  }
}

TEST(test, set_value) {
  {
    boost::optional<double> a;
    a = 1;
    std::cout << "a ptr" << *a << std::endl;
    // std::cout<<"a val"<<a<<std::endl;
    *a = 1;
    std::cout << "a ptr" << *a << std::endl;
    // std::cout<<"a val"<<a<<std::endl;

    a.get() = 1;
    std::cout << "a ptr" << *a << std::endl;
  }
}

#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <iostream>

namespace bg = boost::geometry;
namespace trans = bg::strategy::transform;

typedef bg::model::d2::point_xy<double> point;
typedef bg::model::polygon<point> polygon;

int test() {
  polygon poly;
  // bg::exterior_ring(poly) = boost::assign::list_of<point>
  //     (0, 0)
  //     (3, 3)
  //     (3, 0)
  //     (0, 0)
  //     ;
  bg::exterior_ring(poly) = boost::assign::list_of<point>(0, 0)(3, 0)(3, 3);
  trans::scale_transformer<double, 2, 2> translate(3.0);

  polygon result;
  bg::transform(poly, result, translate);

  std::cout << bg::dsv(result) << std::endl;
  return 0;
}

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

int buffer_test() {
  typedef double coordinate_type;
  typedef boost::geometry::model::d2::point_xy<coordinate_type> point;
  typedef boost::geometry::model::polygon<point> polygon;

  // Declare strategies
  const double buffer_distance = 1.0;
  // Declare the join_miter strategy
  boost::geometry::strategy::buffer::join_miter join_strategy;

  // Declare other strategies
  boost::geometry::strategy::buffer::distance_symmetric<double>
      distance_strategy(0.5);
  boost::geometry::strategy::buffer::end_flat end_strategy;
  boost::geometry::strategy::buffer::side_straight side_strategy;
  boost::geometry::strategy::buffer::point_circle point_strategy(1);

  // Declare/fill a multi polygon
  boost::geometry::model::polygon<point> mp;
  boost::geometry::read_wkt("POLYGON((0 0,7 8,9 5,5 5))", mp);

  // Create the buffered geometry with sharp corners
  boost::geometry::model::multi_polygon<polygon> result;
  boost::geometry::buffer(mp, result, distance_strategy, side_strategy,
                          join_strategy, end_strategy, point_strategy);
  boost::geometry::model::polygon<point> poly = result.front();
  std::cout << boost::geometry::wkt(mp) << std::endl;
  std::cout << boost::geometry::wkt(poly) << std::endl;

  return 0;
}

TEST(test, buffer_) { buffer_test(); }

TEST(test, scale) { test(); }

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


#include <gtest/gtest.h>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "gtest_practice/boost_test.hpp"
#include <boost/assert.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/assign/list_of.hpp>

namespace bg = boost::geometry;
using namespace behavior_velocity_planner;

TEST(boost_test, boost_test) {
    LineString2d l1;
    l1.emplace_back(Point2d{0,2});
    l1.emplace_back(Point2d{2,2});

    //  line2
    //    |
    // ---+---- line1
    //    |
    {
        LineString2d l2;
        l2.emplace_back(Point2d{1,0});
        l2.emplace_back(Point2d{1,3});
        const bool result = bg::intersects(l1,l2);
        EXPECT_TRUE(result); // 交点を持っている
    }
    // -------- line1
    // -------- line2
    {
        LineString2d l2;
        l2.emplace_back(Point2d{0,0});
        l2.emplace_back(Point2d{2,0});
        const bool result = bg::intersects(l1,l2);
        EXPECT_FALSE(result); // 交点を持っている
    }
}

Polygon2d pointsToPoly(const Point2d p0,const Point2d p1){
    LineString2d line={p0,p1};
    const double angle = atan2(p0.y()-p1.y(),p0.x()-p1.x());
    const double r = 0.5;
    Polygon2d line_poly;
    // add polygon counter clockwise
    line_poly.outer().emplace_back(p0.x() + r*sin(angle),p0.y() - r*cos(angle));
    line_poly.outer().emplace_back(p1.x() + r*sin(angle),p1.y() - r*cos(angle));
    line_poly.outer().emplace_back(p1.x() - r*sin(angle),p1.y() + r*cos(angle));
    line_poly.outer().emplace_back(p0.x() - r*sin(angle),p0.y() + r*cos(angle));
    std::cout << boost::geometry::wkt(line_poly) << std::endl;
    std::cout << boost::geometry::wkt(line) << std::endl;
    return line_poly;
}

TEST(boost_test, boost_test2) {

    for (double i = -2; i <= 2; i++)
    {
       for (double j = -2; j <= 2; j++)
       {
           Point2d p0 = {i,-j};
           Point2d p1 = {j,i};
           Polygon2d poly = pointsToPoly(p0,p1);
       }
       
    }
    
    
}
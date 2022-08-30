#include <autoware_auto_planning_msgs/msg/PathWithLaneId.hpp>
#include <bits/stdc++.h>
#include <gtest/gtest.h>
#include <interp/interp.hpp>

TEST(arclength, lane_id) {
  // case Too narrow
  std::vector<double> base_keys = {0.0, 1.0, 1.0001, 2.0, 3.0, 4.0};
  std::vector<double> base_vals = {0.0, 0.0, 0.1, 0.1, 0.1, 0.1};
  std::vector<double> query_keys = {0.0, 1.0, 1.5, 2.0, 3.0, 4.0};
  std::vector<double> s_query_vals =
      ::interpolation::slerp(base_keys, base_vals, query_keys);
  std::vector<double> l_query_vals =
      ::interpolation::lerp(base_keys, base_vals, query_keys);
  for (size_t i = 0; i < query_keys.size(); i++) {
    std::cout << "s key : " << query_keys.at(i)
              << " s val : " << s_query_vals.at(i) << std::endl;
    std::cout << "l key : " << query_keys.at(i)
              << " l val : " << l_query_vals.at(i) << std::endl;
  }

  // case narrow
  std::cout << "case easier" << std::endl;
  base_keys = {0.0, 1.0, 1.001, 2.0, 3.0, 4.0};
  s_query_vals = ::interpolation::slerp(base_keys, base_vals, query_keys);
  l_query_vals = ::interpolation::lerp(base_keys, base_vals, query_keys);
  for (size_t i = 0; i < query_keys.size(); i++) {
    std::cout << "s key : " << query_keys.at(i)
              << " s val : " << s_query_vals.at(i) << std::endl;
    std::cout << "l key : " << query_keys.at(i)
              << " l val : " << l_query_vals.at(i) << std::endl;
  }
}

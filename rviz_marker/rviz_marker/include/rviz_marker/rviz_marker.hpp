#pragma once
#include <bits/stdc++.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <rclcpp/rclcpp.hpp>

using autoware_auto_planning_msgs::msg::PathWithLaneId;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;


MarkerArray createPathMarkerArray(
  const PathWithLaneId & path, const std::string & ns, const int64_t lane_id, const double r,
  const double g, const double b);
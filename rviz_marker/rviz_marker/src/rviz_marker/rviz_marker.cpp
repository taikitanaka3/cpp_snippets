#include <bits/stdc++.h>
#include <rviz_marker/rviz_marker.hpp>

/*
MarkerArray createPathMarkerArray(const PathWithLaneId &path,
                                  const std::string &ns, const double r,
                                  const double g, const double b) {
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;
  int32_t i = 0;
  int32_t idx = 0;
  Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;
  marker.id = i++;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.type = Marker::ARROW;
  marker.action = Marker::ADD;
  // marker.scale = tier4_autoware_utils::createMarkerScale(0.2, 0.1, 0.3);
  // marker.color = tier4_autoware_utils::createMarkerColor(r, g, b, 0.999);
  for (const auto &p : path.points) {
    marker.pose = p.point.pose;
    msg.markers.push_back(marker);
    ++idx;
  }
  return msg;
}
*/
int main() { return 0; }

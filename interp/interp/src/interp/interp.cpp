#include <bits/stdc++.h>
#include <interp/interp.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

using namespace std::chrono_literals;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    pub_traj_ = this->create_publisher<Trajectory>("trajectory", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    Trajectory traj;
    traj.header.frame_id = "map";
    traj.header.stamp = rclcpp::Clock().now();
    for (int i = 0; i < 4; i++) {
      TrajectoryPoint tp;
      tp.pose.position.x = i;
      tp.pose.position.y = 0;
      traj.points.emplace_back(tp);
    }
    pub_traj_->publish(traj);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_traj_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

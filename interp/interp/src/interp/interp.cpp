#include <bits/stdc++.h>
#include <interp/interp.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <motion_utils/resample/resample.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

using namespace std::chrono_literals;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    pub_traj_ = this->create_publisher<Trajectory>("trajectory", 10);
    pub_resampled_traj_ =
        this->create_publisher<Trajectory>("trajectory_resampled", 10);
    pub_re_resampled_traj_ =
        this->create_publisher<Trajectory>("trajectory_re_resampled", 10);

    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    Trajectory traj;
    traj.header.frame_id = "map";
    traj.header.stamp = rclcpp::Clock().now();
    for (int i = 0; i < 3; i++) {
      TrajectoryPoint tp;
      tp.pose.position.x = i;
      tp.pose.position.y = 2 * sin(M_PI * i / 2);
      traj.points.emplace_back(tp);
    }

    Trajectory resampled_traj = motion_utils::resampleTrajectory(traj, 0.5);
    const auto new_pose = motion_utils::calcLongitudinalOffsetPose(
        resampled_traj.points, traj.points.front().pose.position, 0.0001);
    auto p = *new_pose;
    size_t idx = 0;
    motion_utils::insertTargetPoint(idx, p.position, resampled_traj.points);
    Trajectory re_resampled_traj =
        motion_utils::resampleTrajectory(resampled_traj, 0.5);
    motion_utils::insertOrientation(resampled_traj.points, true);
    pub_traj_->publish(traj);
    pub_resampled_traj_->publish(resampled_traj);
    pub_re_resampled_traj_->publish(re_resampled_traj);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_traj_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_resampled_traj_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_re_resampled_traj_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

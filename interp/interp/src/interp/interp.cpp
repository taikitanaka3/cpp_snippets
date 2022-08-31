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
#include <tier4_autoware_utils/ros/update_param.hpp>

using namespace std::chrono_literals;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::PoseStamped;
using std::placeholders::_1;

class MinimalPublisher : public rclcpp::Node {
public:
  // main
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    pub_pose_ = this->create_publisher<PoseStamped>("inserted_pose", 1);
    pub_traj_ = this->create_publisher<Trajectory>("trajectory", 1);
    pub_resampled_traj_ =
        this->create_publisher<Trajectory>("trajectory_resampled", 1);
    pub_re_resampled_traj_ =
        this->create_publisher<Trajectory>("trajectory_re_resampled", 1);
    pub_resampled_inserted_original_traj_ = this->create_publisher<Trajectory>(
        "resampled_inserted_original_traj", 1);

    resample_based_insert_dist_s =
        declare_parameter("resample_based_insert_dist_s", 0.1);
    resample_interval = declare_parameter("resample_interval", 0.5);
    // set parameter callback
    set_param_res_ = this->add_on_set_parameters_callback(
        std::bind(&MinimalPublisher::paramCallback, this, _1));

    timer_ = this->create_wall_timer(
        5ms, std::bind(&MinimalPublisher::timerCallback, this));
  };

private:
  // call back
  void timerCallback() {
    Trajectory traj;
    PoseStamped pose_stamped;
    traj.header.frame_id = "map";
    traj.header.stamp = rclcpp::Clock().now();
    pose_stamped.header.frame_id = "map";
    pose_stamped.header.stamp = rclcpp::Clock().now();
    for (int i = 0; i < 5; i++) {
      TrajectoryPoint tp;
      tp.pose.position.x = 5 * cos(M_PI * i / 4);
      tp.pose.position.y = 5 * sin(M_PI * i / 4);
      traj.points.emplace_back(tp);
    }
    motion_utils::insertOrientation(traj.points, true);
    // publish original
    { pub_traj_->publish(traj); }
    const double behavior_velocity_resample_interval = 1.0;
    {
      // resampled original
      Trajectory resampled_traj = motion_utils::resampleTrajectory(
          traj, behavior_velocity_resample_interval);
      pub_resampled_traj_->publish(resampled_traj);
    }

    // inserted
    {
      Trajectory resampled_traj =
          motion_utils::resampleTrajectory(traj, resample_interval);
      const auto new_pose = motion_utils::calcLongitudinalOffsetPose(
          resampled_traj.points, traj.points.front().pose.position,
          resample_based_insert_dist_s);
      if (!new_pose) {
        // if over last point back to original point
        resample_based_insert_dist_s = 0;
        return;
      }
      const auto p = new_pose.get().position;
      const size_t base_idx =
          motion_utils::findNearestSegmentIndex(traj.points, p);
      auto original_inserted_pose_idx =
          motion_utils::insertTargetPoint(base_idx, p, traj.points);
      if (!original_inserted_pose_idx)
        return;
      Trajectory original_resampled_traj = motion_utils::resampleTrajectory(
          traj, behavior_velocity_resample_interval);
      motion_utils::insertOrientation(original_resampled_traj.points, true);
      pub_resampled_inserted_original_traj_->publish(original_resampled_traj);
      pose_stamped.pose = traj.points.at(original_inserted_pose_idx.get()).pose;
      pub_pose_->publish(pose_stamped);
    }

    // test re-resampled original
    {
      Trajectory re_resampled_original_traj =
          motion_utils::resampleTrajectory(traj, resample_interval);
      pub_re_resampled_traj_->publish(re_resampled_original_traj);
    }
    // increment s little by little
    resample_based_insert_dist_s += 0.001;
  };

  rcl_interfaces::msg::SetParametersResult
  paramCallback(const std::vector<rclcpp::Parameter> &parameters) {
    using tier4_autoware_utils::updateParam;

    { // option parameter
      updateParam<double>(parameters, "resample_based_insert_dist_s",
                          resample_based_insert_dist_s);
      updateParam<double>(parameters, "resample_interval", resample_interval);
    }
    rcl_interfaces::msg::SetParametersResult result{};
    result.successful = true;
    result.reason = "success";
    return result;
  };

  // parameters
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_traj_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_resampled_traj_;
  rclcpp::Publisher<Trajectory>::SharedPtr
      pub_resampled_inserted_original_traj_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_re_resampled_traj_;
  size_t count_;
  double resample_based_insert_dist_s;
  double resample_interval;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

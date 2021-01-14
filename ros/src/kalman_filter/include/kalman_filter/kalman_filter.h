/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TRIGGER_H
#define TRIGGER_H
#define _USE_MATH_DEFINES

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <functional>
#include <iostream>
#include <queue>
#include <string>
#include <tuple>
#include <vector>
#include <Eigen/Eigen>
#include <random>
#include <cmath>

class KalmanFilter
{
public:
  KalmanFilter();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Publisher
  ros::Publisher example_pub_;
  //subscriber
  ros::Subscriber twist_sub_;
  ros::Subscriber example_sub_;

  // control input
  Eigen::Vector2f u;
  // nosie control input
  Eigen::Vector2f ud;
  // observation z
  Eigen::Vector2f z;
  // dead reckoning
  Eigen::Vector4f xDR;
  // ground truth reading
  Eigen::Vector4f xTrue;
  // Estimation
  Eigen::Vector4f xEst;
  std::vector<Eigen::Vector4f> hxDR;
  std::vector<Eigen::Vector4f> hxTrue;
  std::vector<Eigen::Vector4f> hxEst;
  std::vector<Eigen::Vector2f> hz;
  Eigen::Matrix4f PEst = Eigen::Matrix4f::Identity();
  // Motional model covariance
  Eigen::Matrix4f Q = Eigen::Matrix4f::Identity();
  // Observation model covariance
  Eigen::Matrix2f R = Eigen::Matrix2f::Identity();
  // Motion model simulation error
  Eigen::Matrix2f Qsim = Eigen::Matrix2f::Identity();
  // Observation model simulation error
  Eigen::Matrix2f Rsim = Eigen::Matrix2f::Identity();

  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> gaussian_d{0, 1};

  void estimate();

  //kalman_filter state
  double prev_time_ = ros::Time::now().toSec();
  /* from ros parameter server*/
  std::vector<double> example_position_x_;
  std::vector<double> example_position_y_;
  ros::Timer example_timer_; //!< @brief timer for kalman_filter command computation
  void timer_callback(const ros::TimerEvent &e);
  /**
   * @brief compute and publish control command for path follow with a constant control period
   */
  void onTimer(const ros::TimerEvent &);
  /**
   * @brief update current_pose from tf
   */
  void updateCurrentPose();
  geometry_msgs::PoseStamped::ConstPtr current_pose_ptr_;      //!< @brief measured pose
  geometry_msgs::TwistStamped::ConstPtr current_velocity_ptr_; //!< @brief measured velocity
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_; //!< @brief tf listener
  /* debug point*/
  ros::Publisher pub_debug_gnss_point_;
  ros::Publisher pub_debug_pred_point_;
  ros::Publisher pub_debug_est_point_;
  geometry_msgs::Point e_est_;
  visualization_msgs::MarkerArray debug_estimate_points_;
  visualization_msgs::MarkerArray debug_true_points_;

  geometry_msgs::PoseStamped debug_start_pose_;
  visualization_msgs::MarkerArray debug_example_points_;
  ros::Publisher debug_marker_array_pub_;
  visualization_msgs::Marker createDebugMarker();
};
#endif

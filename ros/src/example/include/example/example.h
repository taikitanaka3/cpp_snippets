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

class Example
{
public:
  Example();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Publisher
  ros::Publisher example_pub_;
  //subscriber
  ros::Subscriber twist_sub_;
  ros::Subscriber example_sub_;

  //example state
  double prev_time_ = ros::Time::now().toSec();
  /* from ros parameter server*/
  std::vector<double> example_position_x_;
  std::vector<double> example_position_y_;
  ros::Timer example_timer_; //!< @brief timer for example command computation
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
  ros::Publisher debug_start_pub_;
  geometry_msgs::PoseStamped debug_start_pose_;
  visualization_msgs::MarkerArray debug_example_points_;
  ros::Publisher debug_marker_array_pub_;
  void publishDebugMarker();
};
#endif

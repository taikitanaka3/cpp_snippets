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

#ifndef SWITCHER
#define SWITCHER

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <functional>
#include <iostream>
#include <queue>
#include <string>
#include <tuple>
#include <vector>

class Vehicle
{
public:
  Vehicle();
  int COUNT_DEFAULT = 20;

private:
  //Subscriber
  ros::Subscriber switch_sub_;
  ros::Subscriber trajectory_sub_1_;
  ros::Subscriber trajectory_sub_2_;

  // Publisher
  ros::Publisher trajectory_pub_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  enum DRIVING { LANE_DRIVING = 0, K_TURN = 1 };

  std_msgs::Int32ConstPtr state_;
  autoware_planning_msgs::Trajectory::ConstPtr trajectory_1_;
  autoware_planning_msgs::Trajectory::ConstPtr trajectory_2_;
  void onState(const std_msgs::Int32ConstPtr & msg);
  void ll2TimerCallback(const ros::TimerEvent & e);

  ros::Timer ll2_timer_;
  int counter_;
  int ll2_counter_;
};

#endif

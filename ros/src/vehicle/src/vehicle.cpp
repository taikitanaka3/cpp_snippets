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
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <vehicle/vehicle.h>


Vehicle::Vehicle() : nh_(""), pnh_("~")
{
	/*
  trajectory_sub_1_ = pnh_.subscribe("input/trajectory_1", 1, &Vehicle::inputTrajectory1, this);
  trajectory_sub_2_ = pnh_.subscribe("input/trajectory_2", 1, &Vehicle::inputTrajectory2, this);
  trajectory_pub_ =
    pnh_.advertise<autoware_planning_msgs::Trajectory>("output/trajectory", 1, true);
  switch_sub_ = pnh_.subscribe("input/switch", 1, &Vehicle::onState, this);
  ll2_timer_ = nh_.createTimer(ros::Duration(0.05), &Vehicle::ll2TimerCallback, this, false);
  counter_ = 0;
  ll2_counter_ = 0;
}

void Vehicle::ll2TimerCallback(const ros::TimerEvent & e)
{
  if (trajectory_1_ && state_->data == Vehicle::LANE_DRIVING && ll2_counter_ > 0) {
    ROS_DEBUG_THROTTLE(2.0, "[vehicle] trajectory 1 is selected");
    trajectory_pub_.publish(trajectory_1_);
  }
  if (trajectory_2_ && (state_->data >= Vehicle::K_TURN) && ll2_counter_ > 0) {
    ROS_DEBUG_THROTTLE(2.0, "[vehicle] trajectory 2 is selected");
    trajectory_pub_.publish(trajectory_2_);
  }
  if (ll2_counter_ > 0) counter_--;*/
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "vehicle");
  Vehicle node;
  ros::spin();
  return 0;
}

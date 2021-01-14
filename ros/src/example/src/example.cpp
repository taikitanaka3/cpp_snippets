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

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <example/example.h>

void Example::updateCurrentPose()
{
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_DELAYED_THROTTLE(
        1.0, "[example] cannot get map to because_link transform. %s", ex.what());
    return;
  }
  geometry_msgs::PoseStamped ps;
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  current_pose_ptr_ = boost::make_shared<geometry_msgs::PoseStamped>(ps);
}

void Example::publishDebugMarker()
{
  for (int i = 0; i < 3; i++)
  {
    auto &m = debug_example_points_.markers[i];
    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();
    m.ns = "example";
    m.id = i;
    m.lifetime = ros::Duration();
    m.pose.position.x = example_position_x_[i];
    m.pose.position.y = example_position_y_[i];
    m.type = visualization_msgs::Marker::CUBE;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 100.0;
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;
    m.color.a = 1.0f;
  }
  debug_marker_array_pub_.publish(debug_example_points_);
}

void Example::onTimer(const ros::TimerEvent &te)
{
  updateCurrentPose();
  std_msgs::Int32 msg;
  example_pub_.publish(msg);
  publishDebugMarker();
}

Example::Example() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  pnh_.getParam("example_position_x", example_position_x_);
  pnh_.getParam("example_position_x", example_position_x_);
  pnh_.getParam("example_position_y", example_position_y_);
  debug_marker_array_pub_ =
      pnh_.advertise<visualization_msgs::MarkerArray>("check_points", 1, true);
  example_pub_ = nh_.advertise<std_msgs::Int32>("/switcher/switch", 1, true);
  /* wait to get vehicle position */
  while (ros::ok())
  {
    try
    {
      tf_buffer_.lookupTransform("map", "base_link", ros::Time::now(), ros::Duration(0.2));
      break;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_INFO_THROTTLE(
          2.0, "[example] is waiting to get map to base_link transform. %s", ex.what());
      continue;
    }
  }

  while (ros::ok())
  {
    ros::Duration(2.0).sleep();
    ros::spinOnce();
  }
  debug_example_points_.markers.resize(3);
  example_timer_ = pnh_.createTimer(ros::Duration(0.1), &Example::onTimer, this);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "example");
  Example node;
  ros::spin();
  return 0;
}

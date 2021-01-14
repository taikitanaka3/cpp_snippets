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
#include <kalman_filter/kalman_filter.h>

#define DT 0.1

// x_{t+1} = F@x_{t}+B@u_t
Eigen::Vector4f motion_model(Eigen::Vector4f x, Eigen::Vector2f u)
{
  Eigen::Matrix4f F_;
  F_ << 1.0, 0, 0, 0,
      0, 1.0, 0, 0,
      0, 0, 1.0, 0,
      0, 0, 0, 1.0;

  Eigen::Matrix<float, 4, 2> B_;
  B_ << DT * std::cos(x(2, 0)), 0,
      DT * std::sin(x(2, 0)), 0,
      0.0, DT,
      1.0, 0.0;

  return F_ * x + B_ * u;
};

Eigen::Matrix4f jacobF(Eigen::Vector4f x, Eigen::Vector2f u)
{
  Eigen::Matrix4f jF_ = Eigen::Matrix4f::Identity();
  float yaw = x(2);
  float v = u(0);
  jF_(0, 2) = -DT * v * std::sin(yaw);
  jF_(0, 3) = DT * std::cos(yaw);
  jF_(1, 2) = DT * v * std::cos(yaw);
  jF_(1, 3) = DT * std::sin(yaw);
  return jF_;
};

//observation mode H
Eigen::Vector2f observation_model(Eigen::Vector4f x)
{
  Eigen::Matrix<float, 2, 4> H_;
  H_ << 1, 0, 0, 0,
      0, 1, 0, 0;
  return H_ * x;
};

Eigen::Matrix<float, 2, 4> jacobH()
{
  Eigen::Matrix<float, 2, 4> jH_;
  jH_ << 1, 0, 0, 0,
      0, 1, 0, 0;
  return jH_;
};

void ekf_estimation(Eigen::Vector4f &xEst, Eigen::Matrix4f &PEst,
                    Eigen::Vector2f z, Eigen::Vector2f u,
                    Eigen::Matrix4f Q, Eigen::Matrix2f R)
{
  Eigen::Vector4f xPred = motion_model(xEst, u);
  Eigen::Matrix4f jF = jacobF(xPred, u);
  Eigen::Matrix4f PPred = jF * PEst * jF.transpose() + Q;

  Eigen::Matrix<float, 2, 4> jH = jacobH();
  Eigen::Vector2f zPred = observation_model(xPred);
  Eigen::Vector2f y = z - zPred;
  Eigen::Matrix2f S = jH * PPred * jH.transpose() + R;
  Eigen::Matrix<float, 4, 2> K = PPred * jH.transpose() * S.inverse();
  xEst = xPred + K * y;
  PEst = (Eigen::Matrix4f::Identity() - K * jH) * PPred;
};

void KalmanFilter::updateCurrentPose()
{
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_DELAYED_THROTTLE(
        1.0, "[kalman_filter] cannot get map to because_link transform. %s", ex.what());
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

visualization_msgs::Marker KalmanFilter::createDebugMarker()
{
  visualization_msgs::Marker m;
  m.header.frame_id = "/map";
  m.header.stamp = ros::Time::now();
  m.ns = "kalman_filter";
  m.id = 0;
  m.lifetime = ros::Duration();
  m.pose.position.x = 1.0;
  m.pose.position.y = 1.0;
  m.type = visualization_msgs::Marker::POINTS;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 1.0f;
  m.color.g = 0.0f;
  m.color.b = 0.0f;
  m.color.a = 1.0f;
  return m;
}

void KalmanFilter::estimate()
{
  // noise control input
  ud(0) = u(0) + gaussian_d(gen) * Qsim(0, 0);
  ud(1) = u(1) + gaussian_d(gen) * Qsim(1, 1);
  xTrue = motion_model(xTrue, u);
  xDR = motion_model(xDR, ud);
  // observation z
  z(0) = xTrue(0) + gaussian_d(gen) * Rsim(0, 0);
  z(1) = xTrue(1) + gaussian_d(gen) * Rsim(1, 1);
  ekf_estimation(xEst, PEst, z, ud, Q, R);
  hxDR.push_back(xDR);
  hxTrue.push_back(xTrue);
  hxEst.push_back(xEst);
  hz.push_back(z);
  e_est_.x = xEst[0];
  e_est_.y = xEst[1];

  std::cout << ud << std::endl;
  std::cout << z << std::endl;
}

void KalmanFilter::onTimer(const ros::TimerEvent &te)
{
  //updateCurrentPose();
  estimate();
  visualization_msgs::Marker m = createDebugMarker();
  pub_debug_gnss_point_.publish(m);
}

KalmanFilter::KalmanFilter() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  pnh_.getParam("example_position_x", example_position_x_);
  pnh_.getParam("example_position_x", example_position_x_);
  pnh_.getParam("example_position_y", example_position_y_);
  u << 1.0, 0.1;
  xDR << 0.0, 0.0, 0.0, 0.0;
  xTrue << 0.0, 0.0, 0.0, 0.0;
  xEst << 0.0, 0.0, 0.0, 0.0;
  // Motional model covariance
  Q(0, 0) = 0.1 * 0.1;
  Q(1, 1) = 0.1 * 0.1;
  Q(2, 2) = (1.0 / 180 * M_PI) * (1.0 / 180 * M_PI);
  Q(3, 3) = 0.1 * 0.1;
  // Observation model covariance
  R(0, 0) = 1.0;
  R(1, 1) = 1.0;
  // Motion model simulation error
  Qsim(0, 0) = 1.0;
  Qsim(1, 1) = (30.0 / 180 * M_PI) * (30.0 / 180 * M_PI);
  // Observation model simulation error
  Rsim(0, 0) = 0.5 * 0.5;
  Rsim(1, 1) = 0.5 * 0.5;

  pub_debug_gnss_point_ =
      pnh_.advertise<visualization_msgs::Marker>("gnss_point", 1, true);
  pub_debug_pred_point_ =
      pnh_.advertise<visualization_msgs::Marker>("pred_point", 1, true);

  example_pub_ = nh_.advertise<std_msgs::Int32>("/switcher/switch", 1, true);

  debug_example_points_.markers.resize(3);
  example_timer_ = pnh_.createTimer(ros::Duration(0.1), &KalmanFilter::onTimer, this);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "kalman_filter");
  KalmanFilter node;
  ros::spin();
  return 0;
}

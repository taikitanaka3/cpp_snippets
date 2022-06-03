// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace behavior_velocity_planner {
namespace occlusion_spot_utils {

struct DetectionArea {
  double max_lateral_distance;    // [m] distance to care about occlusion spot
  double slice_length;            // [m] size of each slice
  double min_occlusion_spot_size; // [m] minumum size to care about the
                                  // occlusion spot
};
struct Velocity {
  double safety_ratio;         // [-] safety margin for planning error
  double max_stop_jerk;        // [m/s^3] emergency braking system jerk
  double max_stop_accel;       // [m/s^2] emergency braking system deceleration
  double max_slow_down_accel;  // [m/s^2] maximum allowed deceleration
  double min_allowed_velocity; // [m/s]   minimum allowed velocity not to stop
  double a_ego;                // [m/s^2] current ego acceleration
  double v_ego;                // [m/s]   current ego velocity
  double delay_time;           // [s] safety time buffer for delay response
  double safe_margin;          // [m] maximum safety distance for any error
};

struct LatLon {
  double lateral_distance;      // [m] lateral distance
  double longitudinal_distance; // [m] longitudinal distance
};

struct PlannerParam {
  // parameters in yaml
  double detection_area_length; // [m]
  double stuck_vehicle_vel;     // [m/s]
  double lateral_distance_thr;  // [m] lateral distance threshold to consider
  double pedestrian_vel;        // [m/s]

  double dist_thr;      // [m]
  double angle_thr;     // [rad]
  bool show_debug_grid; // [-]

  // vehicle info
  double half_vehicle_width; // [m]  half vehicle_width from vehicle info
  double baselink_to_front;  // [m]  wheel_base + front_overhang

  Velocity v;
  DetectionArea detection_area;
};

struct SafeMotion {
  double stop_dist;
  double safe_velocity;
};




inline double binarySearch(const double j,const double a,const double v, const double d, const double min, const double max) {
  auto f = [](const double t, const double j, const double a, const double v,
              const double d) {
    return j * t * t * t / 6.0 + a * t * t / 2.0 + v * t - d;
  };
  const double eps = 1e-3;
  double lower = min;
  double upper = max;
  double t;
  for (int i=0;i<20;i++) {
    t = (lower + upper) / 2;
    const double fx = f(t, j, a, v, 0.0);
    //std::cout<<"fx"<<fx<<"upper"<<upper<<"lower"<<lower<<"t"<<t<<std::endl;
    if (std::abs(fx) < eps){
      //std::cout<<"break"<<std::endl;
      break;
    }
    else if (fx > 0)
    {  
      upper = t;
    }
    else {
      lower = t;
    }
  }
  return 0;
}


inline double binarySearch(double x,double b,const double min, const double max) {
  auto f = [](double x,double b) {
    return x + b;
  };
  const double eps = 1e-3;
  double lower = min;
  double upper = max;
  double t;
  for (int i=0;i<20;i++) {
    t = (lower + upper) / 2;
    const double fx = f(t,b);
    //std::cout<<"b"<<b<<"fx"<<fx<<"upper"<<upper<<"lower"<<lower<<"t"<<t<<std::endl;
    if (std::abs(fx) < eps){
      //std::cout<<"break"<<std::endl;
      break;
    }
    else if (fx > 0)
    {  
      upper = t;
    }
    else {
      lower = t;
    }
  }
  return 0;
}

/**
 * @param: v: ego velocity config
 * @param: ttc: time to collision
 * @return safe motion
 **/
inline double calculatePredictedVelocity(const Velocity &v, const double l) {

  auto ft = [](const double t, const double j, const double a, const double v,
              const double d) {
    return j * t * t * t / 6.0 + a * t * t / 2.0 + v * t - d;
  };
  auto vt = [](const double t, const double j, const double a,
                const double v) { return j * t * t / 2.0 + a * t + v; };
  const double j = v.max_stop_jerk;
  const double a0 = v.a_ego;
  const double a_max = v.max_stop_accel;
  const double v0 = v.v_ego;
  const double t_const_jerk = (v.max_stop_accel - v.a_ego) / v.max_stop_jerk;
  const double d_const_jerk_stop = ft(t_const_jerk, j, a0, v0, 0.0);
  double velocity = 0;
  // case target velocity is within constant jerk stop
  if (l < d_const_jerk_stop) {
    double lower = 0;
    double upper = t_const_jerk;
    const double t = binarySearch(j,a0,v0,l,0,t_const_jerk);
    const double velocity = vt(t, v.max_stop_jerk, a0, v0);
    return velocity;
    // case where target velocity is within constant max accel after constant
    // jerk stop
  } else {
    const double l1 = l - d_const_jerk_stop;
    const double v1 = vt(t_const_jerk, v.max_stop_jerk, a0, v0);
    const double t = binarySearch(0,a_max,v1,l1,t_const_jerk,100);
    const double velocity = vt(t, v.max_stop_jerk, v.a_ego, v.v_ego);
    return velocity;
  }
  return -1;
}

/**
 * @param: v: ego velocity config
 * @param: ttc: time to collision
 * @return safe motion
 **/
inline SafeMotion calculateSafeMotion(const Velocity &v, const double ttc) {
  SafeMotion sm;
  const double j_max = v.safety_ratio * v.max_stop_jerk;
  const double a_max = v.safety_ratio * v.max_stop_accel;
  const double t1 = v.delay_time;
  double t2 = a_max / j_max;
  double &v_safe = sm.safe_velocity;
  double &stop_dist = sm.stop_dist;
  if (ttc <= t1) {
    // delay
    v_safe = 0;
    stop_dist = 0;
  } else if (ttc <= t2 + t1) {
    // delay + const jerk
    t2 = ttc - t1;
    v_safe = -0.5 * j_max * t2 * t2;
    stop_dist = v_safe * t1 - j_max * t2 * t2 * t2 / 6;
  } else {
    const double t3 = ttc - t2 - t1;
    // delay + const jerk + const accel
    const double v2 = -0.5 * j_max * t2 * t2;
    v_safe = v2 - a_max * t3;
    stop_dist = v_safe * t1 - j_max * t2 * t2 * t2 / 6 + v2 * t3 -
                0.5 * a_max * t3 * t3;
  }
  stop_dist += v.safe_margin;
  return sm;
}

/**
 *
 * @param: longitudinal_distance: longitudinal distance to collision
 * @param: param: planner param
 * @return lateral distance
 **/
inline double
calculateLateralDistanceFromTTC(const double longitudinal_distance,
                                const PlannerParam &param) {
  const auto &v = param.v;
  const auto &p = param;
  double v_min = 1.0;
  const double lateral_buffer = 0.5; // TODO this should be in param
  const double min_distance = p.half_vehicle_width + lateral_buffer;
  const double max_distance = p.detection_area.max_lateral_distance;
  if (longitudinal_distance <= 0)
    return min_distance;
  if (v_min < param.v.min_allowed_velocity)
    v_min = param.v.min_allowed_velocity;
  // use min velocity if ego velocity is below min allowed
  const double v0 = (v.v_ego > v_min) ? v.v_ego : v_min;
  // here is a part where ego t(ttc) can be replaced by calculation of velocity
  // smoother or ?
  double t = longitudinal_distance / v0;
  double lateral_distance = t * param.pedestrian_vel + p.half_vehicle_width;
  return std::min(max_distance, std::max(min_distance, lateral_distance));
}

} // namespace occlusion_spot_utils
} // namespace behavior_velocity_planner

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

#include "gtest_practice/risk_predictive_braking.hpp"

#include <gtest/gtest.h>

#include <vector>

TEST(safeMotion, delay_jerk_acceleration) {
  namespace utils = behavior_velocity_planner::occlusion_spot_utils;
  using utils::calculateSafeMotion;
  /**
   * @brief check if calculation is correct in below parameter
   * delay =  0.5 [s]
   * a_max = -4.5 [m/s^2]
   * j_max = -3.0 [m/s^3]
   * case1 delay
   * case2 delay + jerk
   * case3 delay + jerk + acc
   */
  utils::Velocity v{1.0, -3.0, -4.5, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0};
  double ttc = 0.0;
  // case 1 delay
  {
    ttc = 0.5;
    utils::SafeMotion sm = utils::calculateSafeMotion(v, ttc);
    EXPECT_DOUBLE_EQ(sm.safe_velocity, 0.0);
    EXPECT_DOUBLE_EQ(sm.stop_dist, 0.0);
  }
  // case 2 delay + jerk
  {
    ttc = 1.5;
    utils::SafeMotion sm = utils::calculateSafeMotion(v, ttc);
    EXPECT_DOUBLE_EQ(sm.safe_velocity, 1.5);
    EXPECT_DOUBLE_EQ(sm.stop_dist, 1.25);
  }
  // case 3 delay + jerk + acc
  {
    ttc = 3.25;
    utils::SafeMotion sm = utils::calculateSafeMotion(v, ttc);
    EXPECT_DOUBLE_EQ(sm.safe_velocity, 9);
    EXPECT_DOUBLE_EQ(std::round(sm.stop_dist * 100.0) / 100.0, 13.92);
  }
}

TEST(detectionArea, calcLateralDistance) {
  namespace utils = behavior_velocity_planner::occlusion_spot_utils;
  using utils::calculateLateralDistanceFromTTC;
  /**
   * @brief check if calculation is correct in below parameter
   * lateral distance is calculated from
   * - ego velocity
   * - min distance(safety margin)
   * - max distance(ignore distance above this)
   * - pedestrian velocity
   * - min allowed velocity(not to stop)
   */
  utils::PlannerParam p;
  p.half_vehicle_width = 2.0;
  p.baselink_to_front = 3.0;
  p.pedestrian_vel = 1.5;
  p.detection_area.max_lateral_distance = 5.0;
  p.v.min_allowed_velocity = 1.5;
  p.v.v_ego = 5.0;
  const double detection_area_margin = 1.0;
  const double offset_from_ego_to_start = 0.0;
  {
    bool is_valid;
    for (size_t i = 0; i <= 15; i += 5) {
      // arc length in path point
      const double l = i * 1.0;
      const double s = l - offset_from_ego_to_start;
      const double d = utils::calculateLateralDistanceFromTTC(s, p);
      const double eps = 1e-3;
      // std::cout << "s: " << l << " v: " << p.v.v_ego << " d: " << d
      //           << std::endl;
      if (i == 0)
        EXPECT_NEAR(d, 2.5, eps);
      if (i == 5)
        EXPECT_NEAR(d, 3.5, eps);
      if (i == 10)
        EXPECT_NEAR(d, 5.0, eps);
      if (i == 15)
        EXPECT_NEAR(d, 5.0, eps);
    }
  }
}

TEST(smoothDeceleration, calculatePredictedVelocity) {
  namespace utils = behavior_velocity_planner::occlusion_spot_utils;
  using utils::calculateLateralDistanceFromTTC;
  /**
   * @brief check if calculation is correct in below parameter
   * lateral distance is calculated from
   * - ego velocity
   * - min distance(safety margin)
   * - max distance(ignore distance above this)
   * - pedestrian velocity
   * - min allowed velocity(not to stop)
   */
  utils::PlannerParam p;
  utils::Velocity v{1.0, -3.0, -4.5, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0};
  v.v_ego = 5.0;
  v.a_ego = 1.0;
  v.max_stop_jerk = -1.5;
  v.max_stop_accel = -2.5;
  {
    bool is_valid;
    for (size_t i = 0; i <= 15; i += 5) {
      // arc length in path point
      const double l = i * 1.0;
      const double vel = utils::calculatePredictedVelocity(v, l);
      const double eps = 1e-3;
      // std::cout << "s: " << l << " v: " << vel << std::endl;
    }
  }
}

TEST(bsearch, bsearch) {
  namespace utils = behavior_velocity_planner::occlusion_spot_utils;
  const double x = utils::binarySearch(1.0, -10, -50, 50);
}

//
//  Copyright 2021 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

// ros depend
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <string>

using std_msgs::msg::Float64MultiArray;

class Debugger {

public:
  enum DBGVAL : std::uint8_t {
    INPUT_DATA = 0,             // [0] raw data
    INPUT_PROCESSED = 1,        // [1] processed data
    RESPONSE_DATA = 2,          // [3] raw data
    RESPONSE_PROCESSED = 3,     // [4] response data
    STDDEV_MAX = 4,             // [6] stddev thresh
    DELAY_TIME = 5,             // [7] estimate
    DELAY_TIME_RAW = 6,         // [8] estimate raw
    DELAY_STDDEV = 7,           // [9] estimate delay processed
    CROSS_CORRELATION_PEAK = 8, // [10] correlation coefficient
  };

  Debugger(rclcpp::Node *node, const std::string &name) {
    pub_debug_ = node->create_publisher<Float64MultiArray>(
        "~/debug_values/" + name, rclcpp::QoS{1});
    debug_values_.data.resize(num_debug_values_, 0.0);
  }

  // add debug values here
  mutable Float64MultiArray debug_values_;
  void publishDebugValue() { pub_debug_->publish(debug_values_); }
  ~Debugger() {}

private:
  rclcpp::Publisher<Float64MultiArray>::SharedPtr pub_debug_;
  static constexpr std::uint8_t num_debug_values_ = 9;
};

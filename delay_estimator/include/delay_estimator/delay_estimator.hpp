#pragma once
#include <bits/stdc++.h>

#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "delay_estimator/data_processor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <generic_type_support/generic_type_support.hpp>

using namespace data_processor;
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

class DelayEstimator {
private:
  UpdateResult update_result_;
  Statistic staticstic_;
  EstimationResult estimation_result_;
  std::deque<double> weights_;
  std::deque<double> cross_correlation_;
  double max_current_stddev_ = {0.0};
  double smoothed_input_={0.0};
  double smoothed_response_={0.0};

public:
  DelayEstimator() = default;
  Data input_;
  Data response_;
  EstimationResult estimate(const Params &params, const double input,
                            const double response,
                            std::unique_ptr<Debugger> &debugger);
  DelayEstimator(const std::string &name);
  double getDelayTime();
};

using std_msgs::msg::Float64;
using GenericMessage = generic_type_support::GenericMessageSupport;
using GenericAccess = generic_type_support::GenericTypeAccess;

class DelayEstimatorNode : public rclcpp::Node {
private:
  // subscription
  rclcpp::Subscription<Float64>::SharedPtr sub_command_ptr_;
  rclcpp::Subscription<Float64>::SharedPtr sub_status_ptr_;

  // input
  std::shared_ptr<GenericMessage> type_name_input_;
  std::shared_ptr<GenericAccess> access_input_;
  rclcpp::GenericSubscription::SharedPtr sub_input_;
  // response
  std::shared_ptr<GenericMessage> type_name_response_;
  std::shared_ptr<GenericAccess> access_response_;
  rclcpp::GenericSubscription::SharedPtr sub_response_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void estimateDelay();

  Params params_;
  std::unique_ptr<DelayEstimator> delay_estimator_;
  std::unique_ptr<Debugger> debugger_;

  double input_value_ = {0};
  double response_value_ = {0};

  void callbackInput(const Float64::ConstSharedPtr msg);
  void callbackResponse(const Float64::ConstSharedPtr msg);
  void timerCallback();

public:
  explicit DelayEstimatorNode(const rclcpp::NodeOptions &node_options);
};

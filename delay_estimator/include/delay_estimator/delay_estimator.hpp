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
#include "delay_estimator/debugger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <generic_type_support/generic_type_support.hpp>

using namespace data_processor;
using namespace generic_type_support;

class DelayEstimator {
private:
  UpdateResult update_result_;
  Statistic staticstic_;
  EstimationResult estimation_result_;
  std::deque<double> weights_;
  std::deque<double> cross_correlation_;
  double max_current_stddev_ = 0;
  double smoothed_input_;
  double smoothed_response_;

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

class DelayEstimatorNode : public rclcpp::Node {
private:
  // subscription
  rclcpp::Subscription<Float64>::SharedPtr sub_command_ptr_;
  rclcpp::Subscription<Float64>::SharedPtr sub_status_ptr_;

  // input
  std::shared_ptr<GenericMessageSupport> type_name_input_;
  std::shared_ptr<GenericTypeAccess> access_input_;
  rclcpp::GenericSubscription::SharedPtr sub_input_;
  // response
  std::shared_ptr<GenericMessageSupport> type_name_response_;
  std::shared_ptr<GenericTypeAccess> access_response_;
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

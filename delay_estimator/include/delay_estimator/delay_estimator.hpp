#pragma once
#include <bits/stdc++.h>

#include <chrono>
#include <cmath>
#include <deque>
#include <numeric>
#include <utility>
#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "delay_estimator/data_processor.hpp"
#include "delay_estimator/debugger.hpp"

using namespace data_processor;

class DelayEstimator
{
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
  DelayEstimator()=default;
  Data input_;
  Data response_;
  EstimationResult estimate(const Params & params,
   const double input,const double response,std::unique_ptr<Debugger> &debugger);
  DelayEstimator(
    rclcpp::Node * node, const Params & params, const std::string & name);
  DelayEstimator(rclcpp::Node * node, const std::string & name);
  double getDelayTime();

  std::unique_ptr<Debugger> debugger_;
};


using std_msgs::msg::Float64;

class DelayEstimatorNode : public rclcpp::Node
{
private:
  // subscription
  rclcpp::Subscription<Float64>::SharedPtr sub_command_ptr_;
  rclcpp::Subscription<Float64>::SharedPtr sub_status_ptr_;

  // publication
  rclcpp::Publisher<Float64>::SharedPtr pub_estimated_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void estimateDelay();

  Params params_;
  std::unique_ptr<DelayEstimator> delay_estimator_;
  std::unique_ptr<Debugger> debugger_;

  double input_value_={0};
  double response_value_={0};

  void callbackInput(const Float64::ConstSharedPtr msg);
  void callbackResponse(const Float64::ConstSharedPtr msg);
  void timerCallback();

public:
  explicit DelayEstimatorNode(const rclcpp::NodeOptions & node_options);
};

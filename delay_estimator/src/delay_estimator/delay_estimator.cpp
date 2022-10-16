#include <bits/stdc++.h>
#include <delay_estimator/delay_estimator.hpp>

EstimationResult DelayEstimator::estimate(const Params &params,
                                          const double input,
                                          const double response,
                                          std::unique_ptr<Debugger> &debugger) {
  smoothed_input_ =
      lowpassFilter(input, smoothed_input_, params.filter.cutoff_hz_in,
                    params.data.sampling_hz);
  smoothed_response_ =
      lowpassFilter(response, smoothed_response_, params.filter.cutoff_hz_in,
                    params.data.sampling_hz);
  update_result_ = updateData(params, smoothed_input_, smoothed_response_,
                              input_, response_);
  if (update_result_ = UpdateResult::QUEQUE) {
    const auto &pd = params.data;
    const auto &pt = params.thresh;
    const size_t valid_max_delay_index =
        static_cast<size_t>(pd.total_data * pt.valid_delay_index_ratio);
    const size_t delay_index = getPeakCrossCorrelationCoefficientIndex(
        input_.data, response_.data, weights_, cross_correlation_,
        valid_max_delay_index);
    const double delay_per_index = 1.0 / params.data.sampling_hz;
    const double conf = cross_correlation_.at(delay_index);
    const double delay_time =
        static_cast<double>(delay_index) * delay_per_index;
    if (conf < params.thresh.valid_cross_correlation) {
      return EstimationResult::LOWCORRELATION;
    }
    const double fitlered_value =
        lowpassFilter(delay_time, staticstic_.p_value,
                      params.filter.cutoff_hz_out, params.data.sampling_hz);
    staticstic_.calcSequentialStddev(fitlered_value);
    // set debug values
    {
      using DBG = Debugger::DBGVAL;
      auto &dbgv = debugger->debug_values_;
      dbgv.data.at(DBG::INPUT_DATA) = input;
      dbgv.data.at(DBG::INPUT_PROCESSED) = smoothed_input_;
      dbgv.data.at(DBG::RESPONSE_DATA) = response;
      dbgv.data.at(DBG::RESPONSE_PROCESSED) = smoothed_response_;
      dbgv.data.at(DBG::STDDEV_MAX) = update_result_;
      dbgv.data.at(DBG::DELAY_TIME) = fitlered_value;
      dbgv.data.at(DBG::DELAY_TIME_RAW) = delay_time;
      dbgv.data.at(DBG::DELAY_STDDEV) = staticstic_.stddev;
      dbgv.data.at(DBG::CROSS_CORRELATION_PEAK) = conf;
      debugger->publishDebugValue();
    }
    return SUCCESS;
  }
  return NONE;
}

DelayEstimatorNode::DelayEstimatorNode(const rclcpp::NodeOptions &node_options)
    : Node("delay_estimator", node_options) {
  const double inf = std::numeric_limits<double>::max();

  const auto dp = [this](const std::string &str, auto def_val) {
    std::string name = str;
    return this->declare_parameter(name, def_val);
  };

  // thresh
  auto &pt = params_.thresh;
  {
    pt.valid_min_value = dp("thresh/valid_min_value", -inf);
    pt.valid_max_value = dp("thresh/valid_max_value", inf);
    pt.validation_data_stddev = dp("thresh/validation_data_stddev", 0.2);
    pt.valid_cross_correlation = dp("thresh/valid_cross_correlation", 0.8);
    pt.valid_delay_index_ratio = dp("thresh/valid_delay_index_ratio", 0.1);
  }

  // data size
  auto &pd = params_.data;
  {
    pd.sampling_hz = dp("data/sampling_hz", 30.0);
    pd.estimation_hz = dp("data/estimation_hz", 10.0);
    pd.sampling_duration = dp("data/sampling_duration", 10.0);
    pd.validation_ratio = dp("data/validation_ratio", 0.2);
  }

  // fitler
  auto &pf = params_.filter;
  {
    pf.use_lowpass_filter = dp("filter/use_lowpass_filter", true);
    pf.cutoff_hz_in = dp("filter/cutoff_hz_input", 0.5);
    pf.cutoff_hz_out = dp("filter/cutoff_hz_output", 0.1);
  }

  // debug info
  auto &pde = params_.debug;
  {
    pde.is_showing_debug_info = dp("is_showing_debug_info", true);
    pde.name = dp("name", "delay_time");
    debugger_ = std::make_unique<Debugger>(this, pde.name);
  }

  // input subscriber
  {
    const std::string type_name = declare_parameter<std::string>("input_type");
    const std::string access = declare_parameter<std::string>("input_access");
    const std::string topic = declare_parameter<std::string>("input_topic");
    type_name_input_ = std::make_shared<GenericMessageSupport>(type_name);
    access_input_ = std::make_shared<GenericTypeAccess>(access);
    const auto callback =
        [this](const std::shared_ptr<rclcpp::SerializedMessage> serialized) {
          const auto yaml = type_name_input_->DeserializeYAML(*serialized);
          const auto node = access_input_->Get(yaml);
          input_value_ = node.as<float>();
        };
    sub_input_ =
        create_generic_subscription(topic, type_name, rclcpp::QoS(1), callback);
  }

  // response subscriber
  // {
  //   const std::string type_name =
  //       declare_parameter<std::string>("response_type");
  //   const std::string access =
  //       declare_parameter<std::string>("response_access");
  //   const std::string topic =
  //   declare_parameter<std::string>("response_topic"); type_name_response_ =
  //   std::make_shared<GenericMessageSupport>(type_name); access_response_ =
  //   type_name_response_->GetAccess(access); const auto callback =
  //       [this](const std::shared_ptr<rclcpp::SerializedMessage> serialized) {
  //         const auto yaml = type_name_response_->ConvertYAML(*serialized);
  //         const auto node = access_response_->Access(yaml);
  //         response_value_ = node.as<float>();
  //       };
  //   sub_response_ =
  //       create_generic_subscription(topic, type_name, rclcpp::QoS(1),
  //       callback);
  // }

  // estimation callback
  {
    const auto period_s = 1.0 / params_.data.estimation_hz;
    auto estimation_callback =
        std::bind(&DelayEstimatorNode::estimateDelay, this);
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_s));
    timer_ =
        std::make_shared<rclcpp::GenericTimer<decltype(estimation_callback)>>(
            this->get_clock(), period_ns, std::move(estimation_callback),
            this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_, nullptr);
  }
}

void DelayEstimatorNode::estimateDelay() {
  if (!input_value_ || !response_value_)
    return;
  delay_estimator_->estimate(params_, input_value_, response_value_, debugger_);
}

#include "rclcpp/rclcpp.hpp"
#include <generic_type_support/generic_type_support.hpp>
#include <gtest/gtest.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

using namespace generic_type_support;

class MinimalPublisher : public rclcpp::Node {
private:
  std::shared_ptr<GenericMessageSupport> type_name_input_;
  std::shared_ptr<GenericTypeAccess> access_input_;
  rclcpp::GenericSubscription::SharedPtr sub_input_;

public:
  MinimalPublisher() : Node("test") {
    const std::string type_name =
        declare_parameter<std::string>("std_msgs/msg/Header");
    const std::string access = declare_parameter<std::string>("stamp.sec");
    const std::string topic = declare_parameter<std::string>("/input");
    type_name_input_ = std::make_shared<GenericMessageSupport>(type_name);
    access_input_ = std::make_shared<GenericTypeAccess>(access);
    const auto callback =
        [this](const std::shared_ptr<rclcpp::SerializedMessage> serialized) {
          const auto yaml = type_name_input_->DeserializeYAML(*serialized);
          const auto node = access_input_->Get(yaml);
          std::cout << node.as<float>() << std::endl;
        };
    sub_input_ =
        create_generic_subscription(topic, type_name, rclcpp::QoS(1), callback);
  }
};

TEST(generic_type_support, test1) {
  MinimalPublisher mini;
  while (true) {
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::spin(std::make_shared<DelayEstimatorNode>(node_options));
  rclcpp::shutdown();
  return 0;
}

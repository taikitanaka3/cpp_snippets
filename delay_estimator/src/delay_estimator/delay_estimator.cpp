#include <bits/stdc++.h>
#include <delay_estimator/delay_estimator.hpp>

EstimationResult DelayEstimator::estimate(const Params &params,
                                          const double input,
                                          const double response,
                                          std::unique_ptr<Debugger> &debugger) {
  EstimationResult result = NONE;
  smoothed_input_ =
      lowpassFilter(input, smoothed_input_, params.filter.cutoff_hz_in,
                    params.data.sampling_hz);
  smoothed_response_ =
      lowpassFilter(response, smoothed_response_, params.filter.cutoff_hz_in,
                    params.data.sampling_hz);
  double current_stddev = 0;
  update_result_ = updateData(params, smoothed_input_, smoothed_response_,
                              input_, response_, current_stddev);
  double conf = 0;
  if (update_result_ == UpdateResult::QUEQUE) {
    const auto &pd = params.data;
    const auto &pt = params.thresh;
    const size_t valid_max_delay_index =
        static_cast<size_t>(pd.sampling_data_size * pt.valid_delay_index_ratio);
    const size_t delay_index = getPeakCrossCorrelationCoefficientIndex(
        input_.data, response_.data, weights_, cross_correlation_,
        valid_max_delay_index);
    const double delay_per_index = 1.0 / params.data.sampling_hz;
    conf = cross_correlation_.at(delay_index);
    const double delay_time =
        static_cast<double>(delay_index) * delay_per_index;
    if (conf < params.thresh.valid_cross_correlation) {
      result = EstimationResult::LOWCORRELATION;
    } else {
      const double fitlered_value =
          lowpassFilter(delay_time, staticstic_.p_value,
                        params.filter.cutoff_hz_out, params.data.sampling_hz);
      staticstic_.calcSequentialStddev(fitlered_value);
      result = SUCCESS;
    }
  }
  // set debug values
  {
    using DBG = Debugger::DBGVAL;
    auto &dbgv = debugger->debug_values_;
    dbgv.data.at(DBG::INPUT_DATA) = input;
    dbgv.data.at(DBG::INPUT_PROCESSED) = smoothed_input_;
    dbgv.data.at(DBG::RESPONSE_DATA) = response;
    dbgv.data.at(DBG::RESPONSE_PROCESSED) = smoothed_response_;
    dbgv.data.at(DBG::STDDEV_MAX) = current_stddev;
    dbgv.data.at(DBG::DELAY_TIME) = staticstic_.mean;
    dbgv.data.at(DBG::DELAY_TIME_RAW) = staticstic_.p_value;
    dbgv.data.at(DBG::DELAY_STDDEV) = staticstic_.stddev;
    dbgv.data.at(DBG::CROSS_CORRELATION_PEAK) = conf;
    debugger->publishDebugValue();
  }
  return result;
}

DelayEstimatorNode::DelayEstimatorNode(const rclcpp::NodeOptions &node_options)
    : Node("delay_estimator", node_options) {
  const double inf = std::numeric_limits<double>::max();
  // thresh
  auto &pt = params_.thresh;
  {
    pt.valid_min_value = declare_parameter<double>("thresh.valid_min_value");
    pt.valid_max_value = declare_parameter<double>("thresh.valid_max_value");
    pt.validation_data_stddev =
        declare_parameter<double>("thresh.validation_data_stddev");
    pt.valid_cross_correlation =
        declare_parameter<double>("thresh.valid_cross_correlation");
    pt.valid_delay_index_ratio =
        declare_parameter<double>("thresh.valid_delay_index_ratio");
  }

  // data size
  auto &pd = params_.data;
  {
    pd.sampling_hz = declare_parameter<double>("data.sampling_hz");
    pd.estimation_hz = declare_parameter<double>("data.estimation_hz");
    pd.sampling_duration = declare_parameter<double>("data.sampling_duration");
    pd.validation_duration =
        declare_parameter<double>("data.validation_duration");
    pd.sampling_data_size = pd.sampling_duration * pd.sampling_hz;
    pd.validation_data_size = pd.validation_duration * pd.sampling_hz;
  }

  // fitler
  auto &pf = params_.filter;
  {
    pf.use_lowpass_filter =
        declare_parameter<bool>("filter.use_lowpass_filter");
    pf.cutoff_hz_in = declare_parameter<double>("filter.cutoff_hz_input");
    pf.cutoff_hz_out = declare_parameter<double>("filter.cutoff_hz_output");
  }

  // debug info
  auto &pde = params_.debug;
  {
    pde.is_showing_debug_info =
        declare_parameter<bool>("debug.is_showing_debug_info");
    pde.name = declare_parameter<std::string>("node_name");
    debugger_ = std::make_unique<Debugger>(this, pde.name);
  }

  // input subscriber
  {
    const std::string type_name = declare_parameter<std::string>("input_type");
    const std::string access = declare_parameter<std::string>("input_access");
    const std::string topic = declare_parameter<std::string>("input_topic");
    type_name_input_ = std::make_shared<GenericMessage>(type_name);
    access_input_ = type_name_input_->GetAccess(access);
    const auto callback =
        [this](const std::shared_ptr<rclcpp::SerializedMessage> serialized) {
          const auto yaml = type_name_input_->ConvertYAML(*serialized);
          const auto node = access_input_->Access(yaml);
          input_value_ = node.as<float>();
        };
    sub_input_ =
        create_generic_subscription(topic, type_name, rclcpp::QoS(1), callback);
  }

  // response subscriber
  {
    const std::string type_name =
        declare_parameter<std::string>("response_type");
    const std::string access =
        declare_parameter<std::string>("response_access");
    const std::string topic = declare_parameter<std::string>("response_topic");
    type_name_response_ = std::make_shared<GenericMessage>(type_name);
    access_response_ = type_name_response_->GetAccess(access);
    const auto callback =
        [this](const std::shared_ptr<rclcpp::SerializedMessage> serialized) {
          const auto yaml = type_name_response_->ConvertYAML(*serialized);
          const auto node = access_response_->Access(yaml);
          response_value_ = node.as<float>();
        };
    sub_response_ =
        create_generic_subscription(topic, type_name, rclcpp::QoS(1), callback);
  }

  // estimation callback
  {
    const auto period_ns = rclcpp::Rate(params_.data.estimation_hz).period();
    timer_ = rclcpp::create_timer(
        this, get_clock(), period_ns,
        std::bind(&DelayEstimatorNode::estimateDelay, this));
  }
}

void DelayEstimatorNode::estimateDelay() {
  if (!input_value_ || !response_value_)
    return;
  delay_estimator_ = std::make_unique<DelayEstimator>();
  delay_estimator_->estimate(params_, input_value_, response_value_, debugger_);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::spin(std::make_shared<DelayEstimatorNode>(node_options));
  rclcpp::shutdown();
  return 0;
}

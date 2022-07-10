#include <bits/stdc++.h>
#include <delay_estimator/delay_estimator.hpp>


EstimationResult DelayEstimator::estimate(
  const Params & params, const double input,const double response, std::unique_ptr<Debugger> &debugger){
  smoothed_input_ = lowpassFilter(input, smoothed_input_ ,params.filter.cutoff_hz_in, params.data_size.sampling_hz);  
  smoothed_response_ = lowpassFilter(response, smoothed_response_ ,params.filter.cutoff_hz_in, params.data_size.sampling_hz);  
  update_result_ = updateData(params,smoothed_input_,smoothed_response_,input_,response_);
  if(update_result_ = UpdateResult::QUEQUE){
    const auto &pd = params.data_size;
    const auto &pt = params.thresh;
    const size_t valid_max_delay_index = static_cast<size_t>(pd.total_data_size*pt.valid_delay_index_ratio);
    const size_t delay_index= getPeakCrossCorrelationCoefficientIndex(
      input_.data,response_.data,weights_,cross_correlation_,valid_max_delay_index);
    const double delay_per_index = 1.0 / params.data_size.sampling_hz;
    const double conf = cross_correlation_.at(delay_index);
    const double delay_time = static_cast<double>(delay_index) * delay_per_index;
    if(conf < params.thresh.valid_peak_cross_correlation){
      return EstimationResult::LOWCORRELATION;
    }
    const double fitlered_value = lowpassFilter(delay_time, staticstic_.p_value ,params.filter.cutoff_hz_out, params.data_size.sampling_hz);
    staticstic_.calcSequentialStddev(fitlered_value);
    // set debug values
    {
      using DBG = Debugger::DBGVAL;
      auto & dbgv = debugger->debug_values_;
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



DelayEstimatorNode::DelayEstimatorNode(const rclcpp::NodeOptions & node_options)
: Node("delay_estimator", node_options){
  const double inf = std::numeric_limits<double>::max();

  // thresh
  params_.thresh.valid_min_value = declare_parameter<double>("thresh/valid_min_value", -inf);
  params_.thresh.valid_max_value = declare_parameter<double>("thresh/valid_max_value", inf);
  params_.thresh.validation_data_stddev = declare_parameter<double>("thresh/validation_data_stddev", 0.2);
  params_.thresh.valid_peak_cross_correlation =
    declare_parameter<double>("thresh/valid_peak_cross_correlation", 0.8);
  params_.thresh.valid_delay_index_ratio =
    declare_parameter<double>("thresh/valid_delay_index_ratio", 0.1);
  
  // data size
  params_.data_size.sampling_hz = declare_parameter<double>("data/sampling_hz", 30.0);
  params_.data_size.estimation_hz = declare_parameter<double>("data/estimation_hz", 10.0);
  params_.data_size.sampling_duration = declare_parameter<double>("data/sampling_duration", 10.0);
  params_.data_size.validation_ratio = declare_parameter<double>("data/validation_ratio", 0.2);

  // fitler
  params_.filter.cutoff_hz_in = declare_parameter<double>("filter/cutoff_hz_input", 0.5);
  params_.filter.cutoff_hz_out = declare_parameter<double>("filter/cutoff_hz_output", 0.1);
  params_.filter.use_lowpass_filter = declare_parameter<bool>("filter/use_lowpass_filter", 0.1);

  // debug info
  params_.is_showing_debug_info = declare_parameter<bool>("is_showing_debug_info", true);
  const std::string name = declare_parameter<std::string>("name", "");
  debugger_ = std::make_unique<Debugger>(this, name);

  // estimation callback
  {
    const auto period_s = 1.0 / params_.data_size.sampling_duration;
    auto estimation_callback = std::bind(&DelayEstimatorNode::estimateDelay, this);
    const auto period_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(estimation_callback)>>(
      this->get_clock(), period_ns, std::move(estimation_callback),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_, nullptr);
  }
}

void DelayEstimatorNode::estimateDelay(){
  delay_estimator_->estimate(params_,input_value_,response_value_,debugger_);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::spin(std::make_shared<DelayEstimatorNode>(node_options));
  rclcpp::shutdown();
  return 0;
}

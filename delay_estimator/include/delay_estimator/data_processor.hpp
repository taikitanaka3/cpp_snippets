#pragma once
#include <bits/stdc++.h>

namespace data_processor {

enum UpdateResult {
  INIT = 0,
  SKIP = 1,
  QUEQUE = 2,
};

enum EstimationResult {
  NONE = 0,
  LOWCORRELATION = 1,
  SUCCESS = 2,
};

struct Params {
  struct Threshold {
    double valid_min_value;
    double valid_max_value;
    double validation_data_stddev;       // [-]
    double valid_peak_cross_correlation; // [-]
    double valid_delay_index_ratio;      // [-]
  };
  struct DataSize {
    size_t total_data;
    size_t validation_data;
    double sampling_duration; // [s]
    double sampling_hz;
    double estimation_hz;
    double validation_ratio; // [s]
  };
  struct Filter {
    double cutoff_hz_in;
    double cutoff_hz_out;
    bool use_lowpass_filter;
  };
  Threshold thresh;
  DataSize data;
  Filter filter;
  bool is_showing_debug_info;
};

struct Data {
  Data() : p_value{0.0} {}
  double p_value = 0;
  std::deque<double> data = {};
  std::deque<double> validation = {};
};

inline double lowpassFilter(const double current_value, const double prev_value,
                            double cutoff, const double dt) {
  const double tau = 1 / (2 * M_PI * cutoff);
  const double a = tau / (dt + tau);
  return prev_value * a + (1 - a) * current_value;
}

template <class T> double getAverageFromVector(const T &arr, const T &w) {
  double sum = 0;
  double sum_w = 0;
  for (size_t i = 0; i < arr.size(); ++i) {
    sum += w[i] * arr[i];
    sum_w += w[i];
  }
  return sum / sum_w;
}

template <class T>
double getStddevFromVector(const T &arr, const double average, const T &w) {
  double var = 0;
  double w_sum = 0;
  for (size_t i = 0; i < arr.size(); i++) {
    var += w[i] * std::pow((arr[i] - average), 2);
    w_sum += w[i];
  }
  return std::sqrt(var / w_sum);
}

template <class T> double getStddevFromVector(const T &arr) {
  const double average = std::accumulate((arr).begin(), (arr).end(), 0.0) /
                         static_cast<double>(arr.size());
  double diff = 0;
  for (const auto &v : arr) {
    diff += std::pow(v - average, 2);
  }
  return std::sqrt(diff / static_cast<double>(arr.size()));
}

template <class T>
double getCovarianceFromVector(const T &x, const T &y, const double avg_x,
                               const double avg_y, const T &w) {
  double cov = 0;
  double sum_w = 0;
  for (size_t i = 0; i < x.size(); i++) {
    cov += w[i] * (x[i] - avg_x) * (y[i] - avg_y);
    sum_w += w[i];
  }
  return cov / sum_w;
}

template <class T>
double calcCrossCorrelationCoefficient(const T &x, const T &y, const T &w) {
  const double avg_x = getAverageFromVector(x, w);
  const double avg_y = getAverageFromVector(y, w);
  const double x_stddev = getStddevFromVector(x, avg_x, w);
  const double y_stddev = getStddevFromVector(y, avg_y, w);
  const double xy_cov = getCovarianceFromVector(w, x, avg_x, avg_y, w);
  return xy_cov / (x_stddev * y_stddev);
}

/**
 *
 * @param input : input signal
 * @param response : output signal
 * @param weight : weight for correlation
 * @param valid_delay_index_ratio : number of shift to compare rate
 * @return corr : size of (input+num_shift) , type T correlation value
 */
template <class T>
inline size_t
getPeakCrossCorrelationCoefficientIndex(const T &input, const T &response,
                                        const T &weight, T &cross_correlation,
                                        const size_t num_estimation) {
  if (input.empty() || response.empty()) {
    return 0;
  }
  cross_correlation.clear();
  /**
   * Correlation Coefficient Method
   * CorrCoeff = Cov(x1x2)/Stddev(x1)*Stddev(x2)
   */
  for (size_t tau = 0; tau < num_estimation; tau++) {
    const size_t peak_index = calcCrossCorrelationCoefficient(
        {input.rbegin() + tau, input.rend()},
        {response.rbegin(), response.rend() - tau}, weight);
    cross_correlation.emplace_back(peak_index);
  }
  const auto &iter =
      std::max_element(cross_correlation.begin(), cross_correlation.end());
  const size_t index = std::distance(cross_correlation.begin(), iter);
  return index;
}

struct Statistic {
  size_t cnt = 0;
  double mean = 0;
  double variance = 0;
  double stddev = 0;
  double p_value = 0;
  // O(1) speed stddev & mean
  double calcSequentialStddev(const double val) {
    double old_avg = mean;
    const double seq = static_cast<double>(cnt);
    mean = (seq * mean + val) / (seq + 1.0);
    variance = (seq * (variance + std::pow(old_avg, 2)) + std::pow(val, 2)) /
                   (seq + 1.0) -
               std::pow(mean, 2);
    cnt++;
    p_value = val;
    return std::sqrt(variance);
  }
};

inline UpdateResult updateData(const Params &params, const double input,
                               const double response, Data &input_data_list,
                               Data &response_data_list) {
  double max_stddev = 0;
  {
    input_data_list.validation.emplace_back(input);
    response_data_list.validation.emplace_back(response);
    const double input_stddev = getStddevFromVector(input_data_list.validation);
    const double response_stddev =
        getStddevFromVector(response_data_list.validation);
    max_stddev = std::max(input_stddev, response_stddev);
    if (input_data_list.validation.size() >= params.data.validation_data) {
      input_data_list.validation.pop_front();
      response_data_list.validation.pop_front();
    }
  }
  if (max_stddev < params.thresh.validation_data_stddev) {
    // continue processing if current data has low stddev
    return UpdateResult::SKIP;
  } else {
    input_data_list.data.emplace_back(input);
    response_data_list.data.emplace_back(response);
    if (input_data_list.data.size() < params.data.total_data) {
      // adding data
      return UpdateResult::INIT;
    } else {
      // replace oldest data with newest if exceeds data size
      input_data_list.data.pop_front();
      response_data_list.data.pop_front();
      return UpdateResult::QUEQUE;
    }
  }
}

} // namespace data_processor

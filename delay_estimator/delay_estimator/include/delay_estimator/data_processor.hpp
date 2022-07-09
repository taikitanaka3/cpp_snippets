#include <bits/stdc++.h>

namespace data_processor{


template<class T>
double getAverageFromVector(const T & arr, const T & w)
{
  double sum = 0;
  double sum_w = 0;
  for (size_t i = 0; i < arr.size(); ++i) {
    sum += w[i] * arr[i];
    sum_w += w[i];
  }
  return sum / sum_w;
}

template<class T>
double getStddevFromVector(const T & arr,const T average, const T & w)
{
  double var = 0;
  double w_sum = 0;
  for (size_t i = 0; i < arr.size(); i++) {
    var += w[i] * std::pow((arr[i] - average), 2);
    w_sum += w[i];
  }
  return std::sqrt(var / w_sum);
}

template<class T>
double getCovarianceFromVector(const T & x, const T & y, const T & avg_x, const T & avg_y, const T & w)
{
  double cov = 0;
  double sum_w = 0;
  for (size_t i = 0; i < x.size(); i++) {
    cov += w[i] * (x[i] - avg_x) * (y[i] - avg_y);
    sum_w += w[i];
  }
  return cov / sum_w;
}

template<class T>
double calcCrossCorrelationCoefficient(const T & x, const T & y, const T & w)
{
  const double avg_x = getAverageFromVector(x, w);
  const double avg_y = getAverageFromVector(y, w);
  const double x_stddev = getStddevFromVector(x, avg_x, w);
  const double y_stddev = getStddevFromVector(y, avg_y, w);
  const double xy_cov = getCovarianceFromVector(w, x, avg_x,avg_y, w);
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
 template<class T>
inline double calcCrossCorrelationCoefficient(
  const T & input, const T & response, const T & weight, T & corss_correlation,
  const double validation_ratio)
{
  if (input.empty()||response.empty()) {return 0;}
  int num_validation = static_cast<size_t>(input.size() * validation_ratio);
  corss_correlation.reserve(num_validation, 0.0);
  /**
   * Correlation Coefficient Method
   * CorrCoeff = Cov(x1x2)/Stddev(x1)*Stddev(x2)
   */
  for (int tau = 0; tau < num_validation; tau++) {
    corss_correlation.at(tau) = getCorrelationCoefficientFromVector(
        {input.rbegin() + tau, input.rbegin.end()},
        {response.rbegin(), response.rbegin.end() - tau},
        weight);
  }
  std::vector<int>::iterator iter = std::max_element(corss_correlation.begin(), corss_correlation.end());
  size_t index = std::distance(corss_correlation.begin(), iter);
  return index;
}

struct Params
{
  double estimation_hz;
  double sampling_duration;
  double validation_duration;
  int data_size;
  int total_data_size;
  int validation_size;
  double valid_peak_cross_correlation_threshold;
  double valid_delay_index_ratio;
  double estimation_delta_time;
  double sampling_hz;
  double cutoff_hz;
  bool is_showing_debug_info;
  bool use_interpolation;
  int num_interpolation;
};
struct Data
{
  Data(): p_value{0.0} {}
  double p_value = 0;
  std::deque<double> data = {};
  std::deque<double> validation = {};
};

struct Result
{
  int estimated_delay_index = 0;
  double time_delay = 0;
  double time_delay_stddev = 0;
  double time_delay_prev = 0;
  double time_delay_default = 0;
  bool is_valid_data = false;
};

struct MinMax
{
  double min;
  double max;
};

enum class ForgetResult : std::int8_t
{
  CURRENT = 0,
  NONE = 1,
  OLD = 2,
  EXCEPTION = -1,
};

inline bool processData(
  const Params & params, const double input,const double response,
  Data & input_data_list,Data & response_data_list)
{
  bool has_enough_input = true;
  input_data_list.data.emplace_back(input);
  response_data_list.data.emplace_back(response);
  return true;
}

/**
  * @brief : forget no feature data to avoid over fitting
  * @param input : input data
  * @param response : response data
  * @return : forget result 0: None 1:OLD 2:NEW
  **/
inline bool checkIsValidData(
  Data & data, Data & response, const Params & params, double & max_stddev,
  const double & ignore_thresh);

}
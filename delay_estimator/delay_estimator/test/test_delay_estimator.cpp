#include <bits/stdc++.h>
#include <gtest/gtest.h>
#include <delay_estimator/delay_estimator.hpp>
#include <delay_estimator/data_processor.hpp>

TEST(test, Coefficient) {
  using namespace data_processor;
  std::deque<double> input={0,0,0,1,1,1};
  std::deque<double> response={0,0,0,0,1,1};


}

void printV(const std::deque<double> &a){
    for(size_t i =0 ;i<a.size();i++){
      std::cout<<" (i, v): "<<" ("<<i<<"," << a.at(i)<<") ";
    }
    std::cout<<std::endl;
}

TEST(test, vector_cmp) {
  using namespace data_processor;
  std::deque<double> input={0,0,0,1,1,1};
  std::deque<double> response={0,0,0,0,1,1};
  std::deque<double> r_input = input;
  std::deque<double> r_response = response;
  double validation_ratio = 0.8;
  const size_t num_validation = static_cast<int>(input.size() * validation_ratio);
  // std::deque<double> corss_correlation;
  // corss_correlation.reserve(num_validation, 0.0);
  /**
   * Correlation Coefficient Method
   * CorrCoeff = Cov(x1x2)/Stddev(x1)*Stddev(x2)
   */
  printV({input.rbegin()+1, input.rend()});
  for (size_t tau = 0; tau < num_validation; tau++) {
    std::deque<double> cmp_input(input.size());
    std::deque<double> cmp_response(response.size());
    copy(r_input.begin() + tau, r_input.end(), cmp_input.begin());
    copy(r_response.begin(), r_response.end() - tau, cmp_response.begin());
    // for(size_t i =0 ;i<cmp_input.size();i++){
    //   std::cout<<" i : "<<i<<" value: " << r_input.at(i);
    // }
    // std::cout<<std::endl;
  }

}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

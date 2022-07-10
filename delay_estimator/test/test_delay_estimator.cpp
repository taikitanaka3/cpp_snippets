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
  std::deque<double> input={0,1,1,1,1,1};
  std::deque<double> response={0,0,1,1,1,1};
  std::deque<double> weight = {1,1,1,1,1,1};
  std::deque<double> cross_correlation = {};
  const size_t max_index = getPeakCrossCorrelationCoefficientIndex(input,response,weight,cross_correlation,3);
  EXPECT_EQ(max_index, 2);
}

TEST(test, process_data) {
  using namespace data_processor;
  std::deque<double> in={0,0,1,0,0,1,1,1,1,1};
  std::deque<double> re={0,0,0,1,0,0,1,1,1,1};
  std::deque<double> we={1,1,1,1,1,1,1,1,1,1};
  std::deque<double> cross_correlation = {};
  Data input_data;
  Data response_data;
  Params params;
  params.data.validation_data=4;
  params.data.total_data=6;
  params.thresh.validation_data_stddev=0.2;
  for (size_t i = 0; i < in.size(); i++)
  {
    const UpdateResult update = updateData(
      params,in.at(i),re.at(i),input_data,response_data);
    std::cout<<"update "<<update<<std::endl;
  }
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

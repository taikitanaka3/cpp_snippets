#include <bits/stdc++.h>
#include <gtest/gtest.h>
#include <lowpass/lowpass.hpp>
inline double lowpassFilter(
  const double current_value, const double prev_value, double cutoff, const double dt)
{
  const double tau = 1 / (2 * M_PI * cutoff);
  const double a = tau / (dt + tau);
  return prev_value * a + (1 - a) * current_value;
}

TEST(test, nominal) {
  const double dt =0.01;
  for(double cutoff =1;cutoff<60;cutoff+=1.0){
    const double tau = 1 / (2 * M_PI * cutoff);
    const double gain = tau / (dt + tau);
    std::cout<<"cutoff_hz: "<< cutoff<<" gain: "<<gain<< " tau: "<<tau<<std::endl;
  }
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

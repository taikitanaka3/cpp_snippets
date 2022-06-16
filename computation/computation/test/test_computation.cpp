#include <bits/stdc++.h>
#include <computation/computation.hpp>
#include <gtest/gtest.h>

using namespace std::chrono;

inline double myhypot(double a, double b) { return a * a + b * b; }

void test_hypot() {
  size_t N = 1000000000;

  {
    auto start = high_resolution_clock::now();
    double a = 0.0;
    for (size_t i = 0; i < N; ++i) {
      a = std::hypot(1.0, 2.0);
    }
    auto end = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end - start);
    std::cout << a << std::endl; // 704 [msec]
    std::cout << duration.count() << std::endl;
  }

  {
    auto start = high_resolution_clock::now();
    double a = 0.0;
    for (size_t i = 0; i < N; ++i) {
      a = myhypot(1.0, 2.0);
    }
    auto end = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end - start);
    std::cout << a << std::endl; // 694 [msec]
    std::cout << duration.count() << std::endl;
  }
}

TEST(test, test_hypot_) { test_hypot(); }

TEST(perf, hypot_increment_none_const) {
  long double a = 1.0;
  long double b = 1.0;
  long double c = 0;
  // 0ms
  for (long long int i = 0; i < 1000000000 /*number of test*/; i++) {
    c += std::hypot(a, b);
  }
}

TEST(perf, hypot_increment) {
  const long double a = 1.0e32;
  const long double b = 1.0e32;
  long double c = 0;
  // 0ms
  for (long long int i = 0; i < 1000000000 /*number of test*/; i++) {
    c += std::hypot(a, b);
  }
  std::cout << std::endl;
}

TEST(perf, use_hypot) {
  const long double a = 1.0e32;
  const long double b = 1.0e32;
  // 0ms
  for (long long int i = 0; i < 1000000000 /*number of test*/; i++) {
    const long double c = std::hypot(a, b);
  }
  std::cout << std::endl;
}

TEST(perf, sqrt_pow2) {
  const long double a = 1.0e32;
  const long double b = 1.0e32;
  // 0ms
  for (long long int i = 0; i < 1000000000 /*number of test*/; i++) {
    const long double c = std::sqrt(a * a + b * b);
  }
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

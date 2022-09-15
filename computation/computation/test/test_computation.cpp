#include <bits/stdc++.h>
#include <computation/computation.hpp>
#include <gtest/gtest.h>

using namespace std::chrono;

using namespace std;

template<typename T>
ostream& operator << (ostream& os, vector<T>& vec) {
	os << "{";
	for (int i = 0; i<vec.size(); i++) {
		os << vec[i] << (i + 1 == vec.size() ? "" : ", ");
	}
	os << "}";
	return os;
}

TEST(std__unique, with_erase) {
  // old
  {
    vector<vector<int>> lane_idss = {{1,2},{2,3},{3,4}};
    std::vector<int> unique_lane_ids;
    for(auto lane_ids: lane_idss){
      unique_lane_ids.emplace_back(lane_ids.front());
    }
    std::vector<int> lane_ids = {1,2,2,3,3,4,4};
    std::cerr << "all ids" <<lane_ids<< std::endl;
    /* remove adjacent duplicates */
    lane_ids.erase(std::unique(lane_ids.begin(), lane_ids.end()), lane_ids.end());
    std::cerr << "ids" <<lane_ids<< std::endl;
  }
  // new
  {
    std::vector<int> lane_ids = {1,2,2,3,3,4,4};
    std::cerr << "all ids" <<lane_ids<< std::endl;
    /* remove adjacent duplicates */
    lane_ids.erase(std::unique(lane_ids.begin(), lane_ids.end()), lane_ids.end());
    std::cerr << "ids" <<lane_ids<< std::endl;
  }
}

inline double myhypot(double a, double b) { return a * a + b * b; }

TEST(vector, val) {
  std::vector<int> c(10);
  std::cerr << "vec" << std::endl;
  for (const auto i : c) {
    std::cerr << i << std::endl;
  }
}

TEST(how, SET) {
  std::set<int> c;

  c.insert(1);
  c.insert(4);
  c.insert(3);
  c.insert(1);

  for (const auto i : c) {
    std::cerr << i << std::endl;
  }
}

TEST(how, unordered_set) {
  std::unordered_set<int> c;

  c.insert(1);
  c.insert(4);
  c.insert(3);
  c.insert(1);

  for (const auto i : c) {
    std::cerr << i << std::endl;
  }
}

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

TEST(perf, order) {
  const double a = 2;
  const double b = 3;
  const bool c = a < b + 1;
  std::cout << c << std::endl;
}

TEST(perf, numelic_limits) {
  const double a = std::numeric_limits<double>::max();
  const double b = std::numeric_limits<double>::max();
  const double c = a * b;
  std::cout << a << std::endl;
  std::cout << c << std::endl;
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

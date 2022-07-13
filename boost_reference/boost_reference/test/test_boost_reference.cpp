#include <bits/stdc++.h>
#include <boost/optional.hpp>
#include <boost_reference/boost_reference.hpp>
#include <gtest/gtest.h>

boost::optional<double> returnEmpty() { return {}; }

boost::optional<size_t> returnNone() { return boost::none; }

TEST(test, nominal) {
  {
    boost::optional<double> a;
    boost::optional<double> b = boost::none;
    EXPECT_EQ(a, b);
  }

  {
    auto a = returnEmpty();
    boost::optional<double> b = boost::none;
    EXPECT_EQ(a, b);
  }
}

TEST(test, compare) {
  // fail
  // {
  // auto seg_idx = returnNone();
  // const size_t idx = *seg_idx ? *seg_idx + 1 : 0;
  // std::cout<<idx<<std::endl;
  // }

  // ok
  {
    auto seg_idx = returnNone();
    const size_t idx = seg_idx ? *seg_idx + 1 : 0;
    std::cout << idx << std::endl;
  }
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

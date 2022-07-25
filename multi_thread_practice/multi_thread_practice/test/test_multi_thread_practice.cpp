#include <bits/stdc++.h>
#include <gtest/gtest.h>
#include <multi_thread_practice/multi_thread_practice.hpp>
TEST(test, nominal) {
  // std::unique_ptr mtp =std::make_unique<MultiThreadPractice>(this);
  // mtp->main();
}

#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

// std::coutへのアクセスを排他的にする
std::mutex print_mtx_;
void safe_print(int x) {
  std::lock_guard<std::mutex> lock(print_mtx_);
  std::cout << x << std::endl;
}

class X {
  std::mutex mtx_;
  std::vector<int> data_;

public:
  void add_values(int values) {
    for (size_t i = 0; i < values; i++) {
      // ロックを取得する(lock_guardのコンストラクタ)
      std::lock_guard<std::mutex> lock(mtx_);
      data_.push_back(i);
      // ロックを手放す(lock_guardのデストラクタ)
    }
  }
  void clear_values(int values) {
    for (size_t i = 0; i < values; i++) {
      // ロックを取得する(lock_guardのコンストラクタ)
      std::lock_guard<std::mutex> lock(mtx_);
      data_.clear();
      // ロックを手放す(lock_guardのデストラクタ)
    }
  }
  void add_values_error(int values) {
    for (size_t i = 0; i < values; i++) {
      // ロックを取得する(lock_guardのコンストラクタ)
      data_.push_back(i);
      // ロックを手放す(lock_guardのデストラクタ)
    }
  }

  void print() {
    std::lock_guard<std::mutex> lock(mtx_);
    for (int x : data_) {
      if (x % 1000 == 0)
        safe_print(x);
    }
  }
};
TEST(test, safe_access) {
  X x;
  const bool is_safe = true;
  std::thread t1([&x] { x.add_values(10000); });
  std::thread t2([&x] { x.add_values(10000); });
  std::thread t3([&x] { x.clear_values(10000); });

  t1.join();
  t2.join();
  t3.join();
  std::cout << "======test=====" << std::endl;
  x.print();
}
TEST(test, conflict) {
  std::vector<int> x;
  std::mutex mtx;

  std::thread t1([&x, &mtx] {
    for (size_t i = 0; i < 1000; i++) {
      std::lock_guard<std::mutex> lock(mtx);
      x.emplace_back(1);
    }
  });
  std::thread t2([&x, &mtx] {
    for (size_t i = 0; i < 1000; i++) {
      std::lock_guard<std::mutex> lock(mtx);
      x.emplace_back(2);
    }
  });
  std::thread t3([&x, &mtx] {
    for (size_t i = 0; i < 1000; i++) {
      std::lock_guard<std::mutex> lock(mtx);
      x.clear();
    }
  });
  std::thread t4([&x, &mtx] {
    for (int v : x) {
      std::cout << "x: " << v << std::endl;
    }
  });
  t1.join();
  t2.join();
  t3.join();
  t4.join();
}

TEST(test, invalid_access) {
  std::vector<int> x={99};
  std::mutex mtx;

  std::thread t1([&x, &mtx] {
    for (size_t i = 0; i < 1000; i++) {
      std::lock_guard<std::mutex> lock(mtx);
      x.emplace_back(1);
      std::cout<<"a1";
    }
  });
  std::thread t2([&x, &mtx] {
    for (size_t i = 0; i < 1000; i++) {
      std::lock_guard<std::mutex> lock(mtx);
      std::cout<<"a2";
      x.emplace_back(2);
    }
  });
  std::thread t3([&x, &mtx] {
    for (size_t i = 0; i < 10000; i++) {
      std::lock_guard<std::mutex> lock(mtx);
      std::cout<<"c";
      x.clear();
    }
  });
  std::thread t4([&x, &mtx] {
    for (size_t i = 0; i < 100; i++) {
      for (int v : x) {
        std::cout << "b x: " << v << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "a x: " << v << std::endl;
      }
    }
  });
  try {
    t1.join();
    t2.join();
    t3.join();
    t4.join();
  } catch (...) {
  }
}

TEST(test, double_free) {
  std::vector<int> x;
  std::mutex mtx;

  std::thread t1([&x, &mtx] {
    for (size_t i = 0; i < 1000; i++) {
      std::lock_guard<std::mutex> lock(mtx);
      x.emplace_back(1);
    }
  });
  std::thread t2([&x, &mtx] {
    for (size_t i = 0; i < 1000; i++) {
      // std::lock_guard<std::mutex> lock(mtx);
      x.emplace_back(2);
    }
  });
  std::thread t3([&x, &mtx] {
    for (size_t i = 0; i < 1000; i++) {
      // std::lock_guard<std::mutex> lock(mtx);
      x.clear();
    }
  });
  std::thread t4([&x, &mtx] {
    for (size_t i = 0; i < 100; i++) {
      for (int v : x) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "x: " << v << std::endl;
      }
    }
  });
  try {
    t1.join();
    t2.join();
    t3.join();
    t4.join();
  } catch (...) {
  }
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

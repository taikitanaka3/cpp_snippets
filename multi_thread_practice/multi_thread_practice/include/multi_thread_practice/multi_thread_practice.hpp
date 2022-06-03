#include <bits/stdc++.h>
#include <chrono>
#include <iostream>
#include <list>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

using std::cout;
using std::endl;
using std::list;
using std::string;
using std::vector;

struct TestThread{
std::mutex mtx_;
std::shared_ptr<std::thread> t1;
void run();

};

struct NumberGenerator {
  std::mutex mtx_;

  std::vector<int> values_ = {0};
  int counter_ = 0;
  void stockValue() {
    mtx_.lock();
    values_.emplace_back(counter_);
    mtx_.unlock();
    counter_++;
  }
  void removeValue() {
    mtx_.lock();
    values_.clear();
    mtx_.unlock();
  }
};

class MultiThreadPractice : public rclcpp::Node {
public:
  std::mutex mtx_;
  std::vector<int> data_;
  MultiThreadPractice(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

public:
  NumberGenerator gen_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr delete_timer_;
  rclcpp::TimerBase::SharedPtr main_timer_;

  void onTimer();
  void onDelete();
  void run();
  void applyValue() {
    std::lock_guard<std::mutex> lock(mtx_);
    data_ = gen_.values_;
  }
  void add_value(int value) {
    std::lock_guard<std::mutex> lock(mtx_);
    data_.push_back(value);
  }
  void remove_value() {
    std::lock_guard<std::mutex> lock(mtx_);
    data_.clear();
  }

  void print(std::string name = "") {
    for (int x : data_) {
      std::cout << name << x << std::endl;
    }
  }
  int main();
};

#include <bits/stdc++.h>
#include <multi_thread_practice/multi_thread_practice.hpp>

void MultiThreadPractice::onTimer() {
  std::cout << "==timer==" << std::endl;
  gen_.stockValue();
  print("timer: ");
}
void MultiThreadPractice::onDelete() {
  std::cout << "==delete==" << std::endl;
  print("remove: ");
  gen_.removeValue();
}
void MultiThreadPractice::run() {
  std::cout << "==run==" << std::endl;
  applyValue();
  print("apply: ");
}
MultiThreadPractice::MultiThreadPractice(const rclcpp::NodeOptions &options)
    : Node("multi_thread_practice", options) {
  try {
    std::thread timer_callback_thread([&] {
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(3000),
          std::bind(&MultiThreadPractice::onTimer, this));
    });
    std::thread delete_callback_thread([&] {
      delete_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(10000),
          std::bind(&MultiThreadPractice::onDelete, this));
    });
    timer_callback_thread.join();
    delete_callback_thread.join();
  } catch (std::exception &ex) {
    std::cerr << ex.what() << std::endl;
  }
  main_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(2000),
                              std::bind(&MultiThreadPractice::run, this));
}

int MultiThreadPractice::main() {

  for (int i = 0; i < 10; i++) {
    std::thread t1([&] { gen_.stockValue(); });
    std::thread t2([&] { applyValue(); });

    t1.join();
    t2.join();

    print("");
  }
  return 0;
}

void TestThread::run() {
  tt.t1 = std::shared_ptr<std::thread>();
  t1 = std::make_shared<std::thread>(func);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<MultiThreadPractice>());
  TestThread tt;
  tt.run();

  rclcpp::shutdown();
  return 0;
}

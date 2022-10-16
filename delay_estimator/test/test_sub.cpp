#include "rclcpp/rclcpp.hpp"
#include <generic_type_support/generic_type_support.hpp>
#include <gtest/gtest.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

using namespace generic_type_support;

class MinimalPublisher : public rclcpp::Node {
private:
  std::shared_ptr<GenericMessageSupport> type_name_input_;
  std::shared_ptr<GenericTypeAccess> access_input_;
  rclcpp::GenericSubscription::SharedPtr sub_input_;

public:
  MinimalPublisher() : Node("test") {
    const std::string type_name =
        declare_parameter<std::string>("std_msgs/msg/Header");
    const std::string access = declare_parameter<std::string>("stamp.sec");
    const std::string topic = declare_parameter<std::string>("/input");
    type_name_input_ = std::make_shared<GenericMessageSupport>(type_name);
    access_input_ = std::make_shared<GenericTypeAccess>(access);
    const auto callback =
        [this](const std::shared_ptr<rclcpp::SerializedMessage> serialized) {
          const auto yaml = type_name_input_->DeserializeYAML(*serialized);
          const auto node = access_input_->Get(yaml);
          std::cout << node.as<float>() << std::endl;
        };
    sub_input_ =
        create_generic_subscription(topic, type_name, rclcpp::QoS(1), callback);
  }
};

TEST(generic_type_support, test1) {
  MinimalPublisher mini;
  while (true) {
  }
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

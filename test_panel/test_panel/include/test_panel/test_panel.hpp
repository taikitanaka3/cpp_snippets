#pragma once
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QTimer>

#ifndef Q_MOC_RUN
#include <bits/stdc++.h>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#endif

class QLineEdit;
using std_msgs::msg::String;
class TestPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TestPanel(QWidget * parent = nullptr);
  void update();
  void onInitialize() override;

private:
  QLineEdit * test_label_= {nullptr};
  QPushButton * test_button_ = {nullptr};
  rclcpp::Publisher<String>::SharedPtr pub_test_;

protected:
  rclcpp::Node::SharedPtr rviz_ros_node_;
};

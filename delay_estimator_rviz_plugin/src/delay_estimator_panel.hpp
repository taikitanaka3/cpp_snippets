// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DELAY_ESTIMATOR_PANEL_HPP_
#define DELAY_ESTIMATOR_PANEL_HPP_

// Qt
#include <QApplication>
#include <QDesktopWidget>
#include <QDir>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QPushButton>
#include <QScreen>
#include <QSpinBox>
#include <QTimer>

#ifndef Q_MOC_RUN
// rviz
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_window.hpp>

// ros
#include <std_msgs/msg/header.hpp>

#include <memory>
#include <string>
#include <vector>
#endif

class QLineEdit;
using InputSubscriberType = std_msgs::msg::Header;
using ResponseSubscriberType = std_msgs::msg::Header;

class DelayEstimatorPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit DelayEstimatorPanel(QWidget *parent = nullptr);
  ~DelayEstimatorPanel() override;
  void update();
  void onInitialize() override;
  void createWallTimer();
  void onTimer();
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config &config) override;
  void onResponseTopic(ResponseSubscriberType::ConstSharedPtr msg) {
    response_ = std::make_shared<ResponseSubscriberType>(*msg);
  }

public Q_SLOTS:
  void onClickStart();
  void onClickStop();
  void onClickResetData();
  void onDurationChanged();

private:
  QPushButton *start_button_ptr_;
  QPushButton *stop_button_ptr_;
  QPushButton *reset_button_ptr_;
  QSpinBox *duration_;
  QTimer *timer_;
  std::shared_ptr<InputSubscriberType> input_;
  std::shared_ptr<ResponseSubscriberType> response_;

protected:
  rclcpp::Subscription<InputSubscriberType>::SharedPtr sub_input_;
  rclcpp::Subscription<ResponseSubscriberType>::SharedPtr sub_response_;
  rclcpp::Node::SharedPtr raw_node_;
};

#endif // DELAY_ESTIMATOR_PANEL_HPP_

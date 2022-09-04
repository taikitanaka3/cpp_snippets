//
//  Copyright 2022 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef BAG_TIME_MANAGER_PANEL_HPP_
#define BAG_TIME_MANAGER_PANEL_HPP_

#include <qt5/QtWidgets/QComboBox>
#include <qt5/QtWidgets/QPushButton>
#include <qt5/QtWidgets/QLabel>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rviz_common/panel.hpp>

// #include <rosbag2_interfaces/srv/pause.hop>
// #include <rosbag2_interfaces/srv/resume.hop>
// #include <rosbag2_interfaces/srv/set_rate.hop>


#include <memory>

namespace rviz_plugins
{
// using rosbag2_interfaces::srv::Pause;
// using rosbag2_interfaces::srv::Resume;
// using rosbag2_interfaces::srv::SetRate;
class BagTimeManagerPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit BagTimeManagerPanel(QWidget * parent = nullptr);
  void onInitialize() override;

protected Q_SLOTS:
  /// @brief callback for when the publishing rate is changed
  void onRateChanged();
  /// @brief callback for when the step button is clicked
  void onPauseClicked();

protected:
  // ROS
  rclcpp::Node::SharedPtr raw_node_;
  // rclcpp::Client<Pause>::SharedPtr client_engage_;
  // rclcpp::Client<Resume>::SharedPtr client_emergency_stop_;
  // rclcpp::Client<SetRate>::SharedPtr client_emergency_stop_;

  // GUI
  QPushButton * pause_button_;
  QLabel * rate_label_;
  QComboBox * step_unit_combo_;
private:
  enum STATE{
    PAUSE,
    RESUME
  };
  STATE current_state_{RESUME};
};

}  // namespace rviz_plugins

#endif  // BAG_TIME_MANAGER_PANEL_HPP_

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

#include "bag_time_manager_panel.hpp"

#include <qt5/QtWidgets/QGridLayout>
#include <qt5/QtWidgets/QHBoxLayout>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QWidget>
#include <rclcpp/duration.hpp>
#include <rviz_common/display_context.hpp>

#include <chrono>
#include <string>

namespace rviz_plugins
{
BagTimeManagerPanel::BagTimeManagerPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // pause / resume
  {
    pause_button_ = new QPushButton("Pause");
    pause_button_->setToolTip("Freeze/Start ROS time.");
    pause_button_->setStyleSheet("background-color: #00FF00;");
    pause_button_->setCheckable(true);
  }

  // label
  {
    rate_label_ = new QLabel("Rate:");
    rate_label_->setAlignment(Qt::AlignCenter);
  }

  // combo
  {
    step_unit_combo_ = new QComboBox();
    step_unit_combo_->addItems({"0.01", "0.1", "1", "2", "5", "10"});
  }

  auto * layout = new QHBoxLayout();
  layout->addWidget(pause_button_);
  layout->addWidget(rate_label_);
  layout->addWidget(step_unit_combo_);
  setLayout(layout);

  connect(pause_button_, SIGNAL(clicked()), this, SLOT(onPauseClicked()));
  connect(step_unit_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(onRateChanged()));
}

void BagTimeManagerPanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  // client_pause_ = raw_node_->create_client<Pause>(
  //   "/rosbag2_player/pause", rmw_qos_profile_services_default);
  // client_resume_ = raw_node_->create_client<Resume>(
  //   "/rosbag2_player/resume", rmw_qos_profile_services_default);
  // client_set_rate_ = raw_node_->create_client<SetRate>(
  //   "/rosbag2_player/set_rate", rmw_qos_profile_services_default);

}


void BagTimeManagerPanel::onPauseClicked(){
  if(current_state_ == STATE::PAUSE){
    // do resume
    current_state_ = STATE::RESUME;
    pause_button_->setText(QString::fromStdString("Resume"));
    pause_button_->setStyleSheet("background-color: #FF0000;");
  } else{
    // do pause
    current_state_ = STATE::PAUSE;
    pause_button_->setText(QString::fromStdString("Pause"));
    pause_button_->setStyleSheet("background-color: #00FF00;");
    //client_pause_->async_send_request(
    //  request, [this](rclcpp::Client<Pause>::SharedFuture result) {
    //    const auto & response = result.get();
    //    if (response->status.code == ResponseStatus::SUCCESS) {
    //      RCLCPP_INFO(raw_node_->get_logger(), "service succeeded");
    //    } else {
    //      RCLCPP_WARN(
    //        raw_node_->get_logger(), "service failed: %s", response->status.message.c_str());
    //    }
    //  });
  }

}

void BagTimeManagerPanel::onRateChanged()
{
  const auto rate = std::stod(step_unit_combo_->currentText().toStdString());
  std::cerr<<"rate : "<<rate<<std::endl;
}
}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::BagTimeManagerPanel, rviz_common::Panel)

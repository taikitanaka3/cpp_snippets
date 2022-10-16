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

#include "delay_estimator_panel.hpp"

#include <rclcpp/rclcpp.hpp>

#include <ctime>
#include <filesystem>
#include <iostream>

using std::placeholders::_1;
using std::placeholders::_2;

DelayEstimatorPanel::DelayEstimatorPanel(QWidget *parent)
    : rviz_common::Panel(parent) {

  auto *button_layout = new QVBoxLayout;
  {
    start_button_ptr_ = new QPushButton("Start");
    connect(start_button_ptr_, &QPushButton::clicked, this,
            &DelayEstimatorPanel::onClickStart);
    stop_button_ptr_ = new QPushButton("Stop");
    connect(stop_button_ptr_, &QPushButton::clicked, this,
            &DelayEstimatorPanel::onClickStop);
    reset_button_ptr_ = new QPushButton("Reset Data");
    connect(reset_button_ptr_, &QPushButton::clicked, this,
            &DelayEstimatorPanel::onClickResetData);
    button_layout->addWidget(start_button_ptr_);
    button_layout->addWidget(stop_button_ptr_);
    button_layout->addWidget(reset_button_ptr_);
  }
  auto *v_layout = new QVBoxLayout;
  // estimation button
  {
    auto *h_layout = new QHBoxLayout;
    duration_ = new QSpinBox();
    duration_->setRange(1, 10);
    duration_->setValue(10);
    duration_->setSingleStep(1);
    connect(duration_, SIGNAL(valueChanged(int)), this,
            SLOT(onDurationChanged));
    h_layout->addWidget(new QLabel("Duration: "));
    h_layout->addWidget(duration_);
    h_layout->addWidget(new QLabel(" [Hz]"));
    v_layout->addLayout(h_layout);
  }
  auto *h_layout = new QHBoxLayout;
  h_layout->addLayout(button_layout);
  h_layout->addLayout(v_layout);
  setLayout(h_layout);

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &DelayEstimatorPanel::onTimer);
}

void DelayEstimatorPanel::onInitialize() {
  raw_node_ =
      this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  sub_input_ = raw_node_->create_subscription<InputSubscriberType>(
      "/input", 1, [this](const InputSubscriberType::ConstSharedPtr msg) {
        input_ = std::make_shared<InputSubscriberType>(*msg);
      });
  sub_response_ = raw_node_->create_subscription<ResponseSubscriberType>(
      "/response", 1, [this](const ResponseSubscriberType::ConstSharedPtr msg) {
        response_ = std::make_shared<ResponseSubscriberType>(*msg);
      });
}

void DelayEstimatorPanel::onDurationChanged() {
  std::cerr << "change duration: " << duration_->value() << std::endl;
}
void DelayEstimatorPanel::onClickStart() { timer_->start(100); }
void DelayEstimatorPanel::onClickStop() { timer_->stop(); }
void DelayEstimatorPanel::onClickResetData() {}
void DelayEstimatorPanel::onTimer() {
  // if(input_&&response_)
  std::cerr << "estimating..." << std::endl;
}

void DelayEstimatorPanel::save(rviz_common::Config config) const {
  Panel::save(config);
}

void DelayEstimatorPanel::load(const rviz_common::Config &config) {
  Panel::load(config);
}

DelayEstimatorPanel::~DelayEstimatorPanel() = default;

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(DelayEstimatorPanel, rviz_common::Panel)

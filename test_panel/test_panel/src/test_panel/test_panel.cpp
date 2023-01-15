#include <test_panel/test_panel.hpp>

TestPanel::TestPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  test_label_ = new QLineEdit("test text");
  test_button_ = new QPushButton("test button");
  connect(
        test_button_, &QPushButton::clicked, this,
        []{std::cerr<<"button pressed: "<<__FILE__<<" at: "<<__LINE__<<std::endl;}
  );

  auto * layout = new QHBoxLayout(this);
  layout->addWidget(new QLabel("test:"));
  layout->addWidget(test_button_);
  layout->addWidget(test_label_);
  setLayout(layout);

  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &TestPanel::update);
  timer->start(60);
}

void TestPanel::onInitialize()
{
  rviz_ros_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  pub_test_ = rviz_ros_node_->create_publisher<String>("test", 1);
}

void TestPanel::update()
{
  RCLCPP_INFO_THROTTLE(rviz_ros_node_->get_logger(), *rviz_ros_node_->get_clock(), 5000, "--update--");
  auto message = String();
  message.data="test pub";
  pub_test_->publish(message);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(TestPanel, rviz_common::Panel)

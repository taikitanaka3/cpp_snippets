#!/bin/bash

cd $(dirname $0)
dir=$(pwd)

echo $dir

package_name=$1

echo "Create package_name directory..."
mkdir $package_name
cd $package_name

echo "Create package_name directory..."
mkdir $package_name
cd $package_name

echo "Create files."

# hpp file template
template_hpp=$(
    cat <<EOS
#pragma once
#include <QHBoxLayout>
#include <QLabel>
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
  QLineEdit * test_label_;
  rclcpp::Publisher<String>::SharedPtr pub_test_;

protected:
  rclcpp::Node::SharedPtr rviz_ros_node_;
};

EOS
)

# cpp file template
template_cpp=$(
    cat <<EOS
#include <$package_name/$package_name.hpp>

TestPanel::TestPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  test_label_ = new QLineEdit;
  test_label_->setReadOnly(true);

  auto * layout = new QHBoxLayout(this);
  layout->addWidget(new QLabel("test:"));
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
  std::cerr<<"file: "<<__FILE__<<"line: "<<__LINE__<<std::endl;
  auto message = String();
  message.data="test pub";
  pub_test_->publish(message);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(TestPanel, rviz_common::Panel)

EOS
)

# test file template
template_test=$(
    cat <<EOS
#include <bits/stdc++.h>
#include <gtest/gtest.h>
#include <$package_name/$package_name.hpp>
TEST(test, nominal) {
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


EOS
)

# cmake file template
template_cmake=$(
    cat <<EOS
cmake_minimum_required(VERSION 3.14)
project($package_name)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options()
endif()
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
ament_auto_add_library($package_name SHARED
  src/$package_name/$package_name.cpp
  include/$package_name/$package_name.hpp
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB_RECURSE test_files test/*.cpp)
  ament_add_gtest(test_$package_name \${test_files})
  target_link_libraries(test_$package_name
    $package_name
  )
endif()

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)
ament_auto_package(
  INSTALL_TO_SHARE
  plugins
)


EOS
)

# xml template
template_xml=$(
    cat <<EOS
<?xml version="1.0"?>
<package format="2">
  <name>$package_name</name>
  <version>0.1.0</version>
  <maintainer email="tanaka3@gmail.com">tanaka3</maintainer>
  <description>The gtest package</description>
  <license>Apache License 2.0</license>
  <depend>rclcpp</depend>
  <depend>rviz_common</depend>
  <depend>rviz_rendering</depend>
  <depend>libqt5-core</depend>
  <depend>libqt5-gui</depend>
  <depend>libqt5-widgets</depend>
  <depend>qtbase5-dev</depend>
  <buildtool_depend>ament_cmake_auto</buildtool_depend>
  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_lint_auto</test_depend>
  <export>
    <build_type>ament_cmake</build_type>
    <rviz plugin="${prefix}/plugins/plugin_description.xml"/>
  </export>
</package>

EOS
)

template_description=$(
    cat <<EOS
<library path="$package_name">
  <class name="$package_name"
         type="TestPanel"
         base_class_type="rviz_common::Panel">
         <description>$package_name</description>
  </class>
</library>
EOS
)

filelist=($package_name)

for file in ${filelist[@]}; do
    mkdir -p src/${file}
    mkdir -p include/${file}
    mkdir -p plugins
    mkdir test
    echo "$template_test" >test/test_${file}.cpp
    echo "$template_hpp" >include/${file}/${file}.hpp
    echo "$template_cpp" >src/${file}/${file}.cpp
    echo "$template_xml" >package.xml
    echo "$template_description" >plugins/plugin_description.xml
    echo "$template_cmake" >CMakeLists.txt
    echo "created file: ${file}.cpp"
done

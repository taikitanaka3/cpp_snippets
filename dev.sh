#!/bin/bash

VEHICLE=lexus
#VEHICLE=jpntaxi
SENSOR=aip_xx1
PKG=lanelet2_extension
PKG=map_loader
PKG=tier4_autoware_utils
PKG=autoware_rosbag_recorder
PKG=raw_vehicle_cmd_converter
PKG=autoware_auto_control_msgs
PKG=pointcloud_preprocessor
PKG=external_cmd_converter

#PKG=external_cmd_converter
PKG=pacmod_interface
PKG=tier4_autoware_utils
PKG=parameter_estimator
PKG=gtest_practice
PKG=multi_thread_practice
PKG=multi_polygon_practice
PKG=cv_practice
#PKG=computation
#PKG=lowpass
#PKG=behavior_path_planner
#PKG=lanelet2_extension
#PKG=interpolation
PKG=window_recorder
#PKG=rviz_marker
PKG=delay_estimator
#PKG=groot
#--packages-up-to-regex .*path_distance.*
PKG=boost_reference

#PKG=path_smoother
case $1 in
test)
    colcon test --packages-select "$PKG" --event-handlers console_direct+
    ;;
sim)
    kill -9 %1
    ros2 launch autoware_launch planning_simulator.launch.xml map_path:="$MAP" vehicle_model:="$VEHICLE" sensor_model:="$SENSOR"
    ;;
build)
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --symlink-install --continue-on-error
    ;;
start)
    ros2 node kill $(rosnode list | grep "$PKG")
    ;;
delete)
    sudo rm -r install/"$PKG" build/"$PKG"
    ;;
debug)
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install --packages-select "$PKG"
    ;;
*)
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --symlink-install --packages-up-to "$PKG"
    ;;
esac

#!/bin/bash

PKG=gtest_practice
PKG=multi_thread_practice
#PKG=multi_polygon_practice
#PKG=cv_practice
PKG=computation
PKG=boost_reference
#PKG=lanelet2_extension
PKG=interpolation
#PKG=window_recorder
#PKG=rviz_marker
PKG=delay_estimator
#PKG=interp
PKG=delay_estimator_rviz_plugin

#PKG=path_smoother
case $1 in
test)
    colcon test --packages-select "$PKG" --event-handlers console_direct+
    ;;
build_all)
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --symlink-install --continue-on-error
    ;;
delete)
    sudo rm -r install/"$PKG" build/"$PKG"
    ;;
debug)
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install --packages-select "$PKG"
    ;;
build_pkg)
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --symlink-install --packages-select "$PKG"
    ;;
*)
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --symlink-install --packages-up-to "$PKG"
    ;;
esac

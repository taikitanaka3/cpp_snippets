cmake_minimum_required(VERSION 3.14)
project(test_panel)

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
ament_auto_add_library(test_panel SHARED
  src/test_panel/test_panel.cpp
  include/test_panel/test_panel.hpp
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB_RECURSE test_files test/*.cpp)
  ament_add_gtest(test_test_panel ${test_files})
  target_link_libraries(test_test_panel
    test_panel
  )
endif()

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)
ament_auto_package(
  INSTALL_TO_SHARE
  plugins
)

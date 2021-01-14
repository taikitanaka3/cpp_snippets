cmake_minimum_required(VERSION 3.0.2)
project(kalman_filter)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kalman_filter
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(kalman_filter
  src/kalman_filter.cpp
)

target_link_libraries(kalman_filter
  ${catkin_LIBRARIES}
)
add_dependencies(kalman_filter
  ${catkin_EXPORTED_TARGETS}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS kalman_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()

install(
  DIRECTORY
    include
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

cmake_minimum_required(VERSION 3.0.2)
project(vehicle)

#add_compile_options(-std=c++14)

find_package(catkin REQUIRED
  roscpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  cpp_common
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vehicle
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vehicle
  src/vehicle.cpp
)


target_link_libraries(vehicle
  ${catkin_LIBRARIES}
)

add_dependencies(vehicle
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS vehicle
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()

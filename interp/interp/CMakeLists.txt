cmake_minimum_required(VERSION 3.5)
project(interp)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options()
endif()
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_auto_add_library(interp SHARED
  src/interp/interp.cpp
)

ament_auto_add_executable(interp_node
  src/interp/interp.cpp
)

target_include_directories(interp
  SYSTEM PUBLIC
    ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB_RECURSE test_files test/*.cpp)
  ament_add_gtest(test_interp ${test_files})
  target_link_libraries(test_interp
    interp
  )
endif()
ament_auto_package()

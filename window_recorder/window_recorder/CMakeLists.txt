cmake_minimum_required(VERSION 3.5)
project(window_recorder)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options()
endif()
find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs imgproc videoio)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Boost REQUIRED)
ament_auto_add_library(window_recorder SHARED
  src/window_recorder/window_recorder.cpp
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS})

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB_RECURSE test_files test/*.cpp)
  ament_add_gtest(test_window_recorder ${test_files})
  target_link_libraries(test_window_recorder
    window_recorder
  )
endif()
ament_auto_package()

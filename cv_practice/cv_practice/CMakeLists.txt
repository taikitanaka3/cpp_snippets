cmake_minimum_required(VERSION 3.5)
project(cv_practice)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options()
endif()
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs imgproc videoio)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

ament_auto_add_library(cv_practice SHARED
  src/cv_practice/cv_practice.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS})



# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB_RECURSE test_files test/*.cpp)
  include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
  )
  ament_add_gtest(test_cv_practice ${test_files})
  target_link_libraries(test_cv_practice
    cv_practice
    ${OpenCV_LIBS}
  )
endif()
ament_auto_package()

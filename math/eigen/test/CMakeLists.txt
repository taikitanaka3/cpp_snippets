
cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
# Contest eigen/test src
add_executable(src_a src/calc.cpp)
add_executable(src_b src/matrix_operation.cpp)
add_executable(src_c src/quaternion.cpp)
add_executable(src_d src/kalman_filter.cpp)
add_executable(src_e src/e.cpp)
add_executable(src_f src/integrate.cpp)

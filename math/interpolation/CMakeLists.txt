
cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)

# Contest interpolation src
include_directories(include)
add_executable(interpolate src/interp.cpp)
add_executable(csv_loader src/csv_loader.cpp)
add_executable(linear_interpolation src/linear_interpolation.cpp)
add_executable(src_c src/c.cpp)
add_executable(interpolate_2d src/d.cpp)
add_executable(src_e src/vvprinte.cpp)
add_executable(src_f src/f.cpp)

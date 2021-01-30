
cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
# Contest math/correlate src
add_executable(src_a src/direct.cpp)
add_executable(src_b src/1d_convoluation.cpp)
add_executable(src_c src/statistic.cpp)
add_executable(src_d src/lowpass.cpp)
add_executable(src_e src/sequentialStat.cpp)
add_executable(src_f src/f.cpp)

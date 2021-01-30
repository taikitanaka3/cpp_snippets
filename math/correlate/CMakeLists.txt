
cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
# Contest math/correlate src
add_executable(src_a src/direct.cpp)
add_executable(1d_convoluation src/1d_convoluation.cpp)
add_executable(statistic src/statistic.cpp)
add_executable(lowpass src/lowpass.cpp)
add_executable(sequentialStat src/sequentialStat.cpp)
add_executable(src_f src/f.cpp)

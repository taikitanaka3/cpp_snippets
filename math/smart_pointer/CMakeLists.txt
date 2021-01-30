
cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)
# Contest smart_pointer src
add_executable(a_rawpointer src/a_rawpointer.cpp)
add_executable(b_auto_poiner src/b_auto_pointer.cpp)
add_executable(src_c src/c_unique_pointer.cpp)
add_executable(src_e src/e.cpp)
add_executable(src_f src/friend.cpp)

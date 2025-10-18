cmake_minimum_required(VERSION 3.10)
project(rfm69pi)
set(CMAKE_CXX_STANDARD 17)

add_executable(program main.cpp rfm69.cpp)
target_link_libraries(program pthread)

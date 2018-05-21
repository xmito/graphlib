cmake_minimum_required(VERSION 3.0)
project(main)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -pedantic -Wold-style-cast -g")

include_directories(../include)
include_directories(../src)

add_executable(main main.cpp)

cmake_minimum_required(VERSION 3.0)
project(graphlib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -pedantic -Wold-style-cast")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(graphlib INTERFACE)
target_include_directories(graphlib INTERFACE ${CMAKE_SOURCE_DIR}/algorithms
											  ${CMAKE_SOURCE_DIR}/data_structures
											  ${CMAKE_SOURCE_DIR}/common)

add_subdirectory(demos)
add_subdirectory(tests)

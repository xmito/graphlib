cmake_minimum_required(VERSION 3.0)
project(testing)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -pedantic -Wold-style-cast")

set (PROJECT_LINK_LIBS libgraphlib.a)
link_directories(../build)

include_directories(../include)

add_executable(main main.cpp)
target_link_libraries(main ${PROJECT_LINK_LIBS})


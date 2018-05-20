cmake_minimum_required(VERSION 3.0)
project(testing)

set (PROJECT_LINK_LIBS libgraphlib.a)
link_directories(../pv264_project/build)

include_directories(../pv264_project/include)

add_executable(main main.cpp)
target_link_libraries(main ${PROJECT_LINK_LIBS})


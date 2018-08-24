add_executable(dijkstra ${CMAKE_CURRENT_LIST_DIR}/main.cpp)
target_include_directories(dijkstra PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(dijkstra graphlib)

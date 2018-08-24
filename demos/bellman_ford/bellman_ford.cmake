add_executable(bellman_ford ${CMAKE_CURRENT_LIST_DIR}/main.cpp)
target_include_directories(bellman_ford PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(bellman_ford graphlib)

add_executable(dag ${CMAKE_CURRENT_LIST_DIR}/main.cpp)
target_include_directories(dag PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(dag graphlib)

cmake_minimum_required(VERSION 3.0)
project(graphlib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -pedantic -Wold-style-cast")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_CLANG_TIDY
  clang-tidy;
  -header-filter=${CMAKE_CURRENT_SOURCE_DIR}
)

# Find boost libraries and set Boost_INCLUDE_DIRS and Boost_LIBRARIES
find_package(Boost REQUIRED)

# Include boost header files
include_directories(${Boost_INCLUDE_DIRS})


add_library(graphlib INTERFACE)
target_include_directories(graphlib INTERFACE ${CMAKE_SOURCE_DIR}/algorithms
											  ${CMAKE_SOURCE_DIR}/data_structures
											  ${CMAKE_SOURCE_DIR}/common)

# Link against boost
target_link_libraries(graphlib INTERFACE ${Boost_LIBRARIES})

add_subdirectory(demos)
add_subdirectory(tests)
add_custom_target(
	run-clang-tidy
	COMMAND clang-tidy -p ${CMAKE_BINARY_DIR} ../tests/*.cpp ../demos
	WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	COMMENT "Running clang-tidy"
)

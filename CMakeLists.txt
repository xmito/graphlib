cmake_minimum_required(VERSION 3.0)
project(graphlib)

enable_testing()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -pedantic -Wold-style-cast")

set(TEST_DIR "./test")
set(INCLUDE_DIR "./include")
set(SRC_DIR "./src")

set(TEST_SUFFIX "Test")
set(CPP_END ".cpp")

include_directories(${INCLUDE_DIR})
include_directories(${SRC_DIR})

file(GLOB SOURCES "${SRC_DIR}/*.cpp")
add_library(graphlib STATIC ${SOURCES})

#include_directories(".")
#add_subdirectory(CircularList)
#add_subdirectory(FibonacciHeap)
#add_subdirectory(BinaryHeap)
#add_subdirectory(BellmanFord)

#add_library(graphlib STATIC CircularList FibonacciHeap BinaryHeap BellmanFord)

set(ALGORITHMS	"AStar"
				"BellmanFord"
				"Bfs"
				"Dag"
				"Dfs"
				"Dijkstra"
				"FloydWarshall"
				"IDDfs"
				"Johnson")

foreach(ALGORITHM ${ALGORITHMS})
	add_executable(${ALGORITHM}${TEST_SUFFIX} ${SRC_DIR}/${ALGORITHM}${CPP_END}
		${TEST_DIR}/${ALGORITHM}${TEST_SUFFIX}${CPP_END} ./src/${ALGORITHM}${CPP_END})
	add_test(${ALGORITHM}${TEST_SUFFIX} ${ALGORITHM}${TEST_SUFFIX})
endforeach(ALGORITHM ${SOURCES})

set(DATA_STRUCTURES "BinaryHeap"
					"CircularList"
					"FibonacciHeap"
					"ListDiGraph"
					"ListGraph"
					"Matrix")

foreach(DATA_STRUCTURE ${DATA_STRUCTURES})
	add_executable(${DATA_STRUCTURE}${TEST_SUFFIX} ${TEST_DIR}/${DATA_STRUCTURE}${TEST_SUFFIX}${CPP_END})
	add_test(${DATA_STRUCTURE}${TEST_SUFFIX} ${DATA_STRUCTURE}${TEST_SUFFIX})
endforeach(DATA_STRUCTURE ${DATA_STRUCTURES})

install(TARGETS graphlib DESTINATION /usr/lib)
install(FILES ${SOURCES} DESTINATION include)

add_executable(main main.cpp)
target_link_libraries(main graphlib)


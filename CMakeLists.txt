cmake_minimum_required(VERSION 3.0)
project(graphlib)

enable_testing()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -pedantic -Wold-style-cast")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(TEST_DIR "./test")
set(INCLUDE_DIR "./include")
set(SRC_DIR "./src")

set(TEST_SUFFIX "Test")
set(CPP_END ".cpp")

include_directories(${INCLUDE_DIR})
include_directories(${SRC_DIR})

file(GLOB LIB_SOURCES ${SRC_DIR}/*.cpp)
file(GLOB LIB_HEADERS ${INCLUDE_DIR}/*.h)

add_library(graphlib STATIC ${LIB_SOURCES} ${LIB_HEADERS})

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
	add_executable(${ALGORITHM}${TEST_SUFFIX}  ${TEST_DIR}/${ALGORITHM}${TEST_SUFFIX}${CPP_END} )
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

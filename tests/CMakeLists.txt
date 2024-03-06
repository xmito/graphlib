enable_testing()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -pedantic -Wold-style-cast")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(TEST_SUFFIX "Test")
set(CPP_SUFFIX ".cpp")
set(TESTS "AStar"
		  "BellmanFord"
		  "Bfs"
		  "DAGShortestPath"
		  "Dfs"
		  "Dijkstra"
		  "FloydWarshall"
		  "IDDfs"
		  "Johnson"
		  "BinaryHeap"
		  "CircularList"
		  "FibonacciHeap"
		  "ListDiGraph"
		  "ListGraph"
		  "Matrix")

foreach(TEST ${TESTS})
	add_executable(${TEST}${TEST_SUFFIX} ${CMAKE_CURRENT_LIST_DIR}/${TEST}${TEST_SUFFIX}${CPP_SUFFIX} )
	target_include_directories(${TEST}${TEST_SUFFIX} PRIVATE ${CMAKE_SOURCE_DIR})
	target_link_libraries(${TEST}${TEST_SUFFIX} graphlib)
	add_test(${TEST}${TEST_SUFFIX} ${TEST}${TEST_SUFFIX})
endforeach(TEST ${TESTS})

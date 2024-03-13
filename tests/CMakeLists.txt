enable_testing()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -pedantic -Wold-style-cast")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CPP_SUFFIX ".cpp")

set(TESTS "AStarTest"
		  "BellmanFordTest"
		  "BfsTest"
		  "DAGShortestPathTest"
		  "DfsTest"
		  "DijkstraTest"
		  "FloydWarshallTest"
		  "IDDfsTest"
		  "JohnsonTest"
		  "BinaryHeapTest"
		  "CircularListTest"
		  "FibonacciHeapTest"
		  "ListDiGraphTest"
		  "ListGraphTest"
		  "MatrixTest")

foreach(TEST ${TESTS})
	add_executable(${TEST} ${CMAKE_CURRENT_LIST_DIR}/${TEST}${CPP_SUFFIX} )
	target_include_directories(${TEST} PRIVATE ${CMAKE_SOURCE_DIR})
	target_link_libraries(${TEST} graphlib)
	add_test(${TEST} ${TEST})
endforeach(TEST ${TESTS})

add_custom_target(tests
	DEPENDS ${TESTS} 
    COMMENT "Building tests"
)
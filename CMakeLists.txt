cmake_minimum_required(VERSION 3.0)
project(graphlib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -pedantic -Wold-style-cast -g")

enable_testing()

add_executable(MatrixTest MatrixTest.cpp)
add_test(MatrixTest MatrixTest)

add_executable(BinaryHeapTest BinaryHeapTest.cpp)
add_test(MatrixTest MatrixTest)

add_executable(CircularListTest CircularListTest.cpp)
add_test(CircularListTest CircularListTest)

add_executable(FibonacciHeapTest FibonacciHeapTest.cpp)
add_test(FibonacciHeapTest FibonacciHeapTest)

add_executable(ListDiGraphTest ListDiGraphTest.cpp)
add_test(ListDiGraphTest ListDiGraphTest)

add_executable(ListGraphTest ListGraphTest.cpp)
add_test(ListGraphTest ListGraphTest)


add_executable(BfsTest BfsTest.cpp)
add_test(BfsTest BfsTest)

add_executable(DfsTest DfsTest.cpp)
add_test(DfsTest DfsTest)

add_executable(IDDfsTest IDDfsTest.cpp)
add_test(IDDfsTest IDDfsTest)

add_executable(DagTest DagTest.cpp)
add_test(DagTest DagTest)

add_executable(DijkstraTest DijkstraTest.cpp)
add_test(DijkstraTest DijkstraTest)

add_executable(BellmanFordTest BellmanFordTest.cpp)
add_test(BellmanFordTest BellmanFordTest)

add_executable(FloydWarshallTest FloydWarshallTest.cpp)
add_test(FloydWarshallTest FloydWarshallTest)

add_executable(JohnsonTest JohnsonTest.cpp)
add_test(JohnsonTest JohnsonTest)



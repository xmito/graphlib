cmake_minimum_required(VERSION 3.0)
project(graphlib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -pedantic -Wold-style-cast -g")


add_executable(MatrixTest MatrixTest.cpp)
add_executable(BinaryHeapTest BinaryHeapTest.cpp)
add_executable(CircularListTest CircularListTest.cpp)
add_executable(FibonacciHeapTest FibonacciHeapTest.cpp)
add_executable(ListDiGraphTest ListDiGraphTest.cpp)
add_executable(ListGraphTest ListGraphTest.cpp)

add_executable(BfsTest BfsTest.cpp)
add_executable(DfsTest DfsTest.cpp)
add_executable(IDDfsTest IDDfsTest.cpp)
add_executable(Dag DagTest.cpp)
add_executable(DijkstraTest DijkstraTest.cpp)
add_executable(BellmanFordTest BellmanFordTest.cpp)
add_executable(FloydWarshallTest FloydWarshallTest.cpp)
add_executable(JohnsonTest JohnsonTest.cpp)


#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "graph_traits.h"
#include "ListDiGraph.h"
#include "Johnson.h"


TEST_CASE("Johnson on ListDiGraph") {
	using namespace graphlib;
	using Graph = ListDiGraph<PathNodeData, WeightedEdgeData>;
	using node_handle = typename graph_traits<Graph>::node_handle;
	Graph graph;
	node_handle handles[5];
	for (auto &handle : handles)
		handle = graph.addNode();

	SECTION("Test on graph from cormen book, page 703") {
		graph.addEdge(handles[0], handles[1], 3);
		graph.addEdge(handles[0], handles[2], 8);
		graph.addEdge(handles[0], handles[4], -4);
		graph.addEdge(handles[1], handles[3], 1);
		graph.addEdge(handles[1], handles[4], 7);
		graph.addEdge(handles[2], handles[1], 4);
		graph.addEdge(handles[3], handles[2], -5);
		graph.addEdge(handles[3], handles[0], 2);
		graph.addEdge(handles[4], handles[3], 6);

		Matrix matrix = johnson(graph);
		REQUIRE_FALSE(matrix.empty());
		for (int i = 0; i < 5; ++i)
			REQUIRE(matrix[i][i] == 0);
		REQUIRE(matrix[0][1] == 1);
		REQUIRE(matrix[0][2] == -3);
		REQUIRE(matrix[0][3] == 2);
		REQUIRE(matrix[0][4] == -4);

		REQUIRE(matrix[1][0] == 3);
		REQUIRE(matrix[1][2] == -4);
		REQUIRE(matrix[1][3] == 1);
		REQUIRE(matrix[1][4] == -1);

		REQUIRE(matrix[2][0] == 7);
		REQUIRE(matrix[2][1] == 4);
		REQUIRE(matrix[2][3] == 5);
		REQUIRE(matrix[2][4] == 3);

		REQUIRE(matrix[3][0] == 2);
		REQUIRE(matrix[3][1] == -1);
		REQUIRE(matrix[3][2] == -5);
		REQUIRE(matrix[3][4] == -2);

		REQUIRE(matrix[4][0] == 8);
		REQUIRE(matrix[4][1] == 5);
		REQUIRE(matrix[4][2] == 1);
		REQUIRE(matrix[4][3] == 6);
	}
}



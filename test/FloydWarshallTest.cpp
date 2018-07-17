#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "ListDiGraph.h"
#include "ListGraph.h"
#include "FloydWarshall.h"

TEST_CASE("Floyd-Warshall on small graphs") {
	using namespace graphlib;

	using graph_type = ListDiGraph<PathNodeData, WeightedEdgeData>;
	using node_handle = typename graph_type::node_handle;
	using distance_type = typename graph_traits<graph_type>::distance_type;
	ListDiGraph<PathNodeData, WeightedEdgeData> graph;

	SECTION("small graph") {
		node_handle handles[5];
		for (auto &handle : handles)
			handle = graph.addNode();

		graph.addEdge(handles[0], handles[2], 6);
		graph.addEdge(handles[0], handles[3], 3);
		graph.addEdge(handles[1], handles[0], 3);
		graph.addEdge(handles[2], handles[3], 2);
		graph.addEdge(handles[3], handles[2], 1);
		graph.addEdge(handles[3], handles[1], 1);
		graph.addEdge(handles[4], handles[1], 4);
		graph.addEdge(handles[4], handles[3], 2);

		Matrix<int64_t> res = floydWarshall(graph);

		REQUIRE(res[0][0] == 0);
		REQUIRE(res[0][1] == 4);
		REQUIRE(res[0][2] == 4);
		REQUIRE(res[0][3] == 3);
		REQUIRE(res[0][4] == std::numeric_limits<distance_type>::max());

		REQUIRE(res[1][0] == 3);
		REQUIRE(res[1][1] == 0);
		REQUIRE(res[1][2] == 7);
		REQUIRE(res[1][3] == 6);
		REQUIRE(res[1][4] == std::numeric_limits<distance_type>::max());

		REQUIRE(res[2][0] == 6);
		REQUIRE(res[2][1] == 3);
		REQUIRE(res[2][2] == 0);
		REQUIRE(res[2][3] == 2);
		REQUIRE(res[2][4] == std::numeric_limits<distance_type>::max());

		REQUIRE(res[3][0] == 4);
		REQUIRE(res[3][1] == 1);
		REQUIRE(res[3][2] == 1);
		REQUIRE(res[3][3] == 0);
		REQUIRE(res[3][4] == std::numeric_limits<distance_type>::max());

		REQUIRE(res[4][0] == 6);
		REQUIRE(res[4][1] == 3);
		REQUIRE(res[4][2] == 3);
		REQUIRE(res[4][3] == 2);
		REQUIRE(res[4][4] == 0);
	}

	SECTION("Floyd-Warshall algorithm on a simple graph 2") {

		node_handle handles[8];
		for (auto &handle : handles)
			handle = graph.addNode();

		graph.addEdge(handles[0], handles[1], 9);
		graph.addEdge(handles[0], handles[2], 6);
		graph.addEdge(handles[1], handles[2], 6);
		graph.addEdge(handles[2], handles[6], 3);
		graph.addEdge(handles[3], handles[1], 6);
		graph.addEdge(handles[3], handles[7], 7);
		graph.addEdge(handles[4], handles[6], 9);
		graph.addEdge(handles[5], handles[6], 3);
		graph.addEdge(handles[5], handles[7], 1);
		graph.addEdge(handles[6], handles[5], 8);

		Matrix<int64_t> res = floydWarshall(graph);

		REQUIRE(res[0][1] == 9);
		REQUIRE(res[0][2] == 6);
		REQUIRE(res[0][5] == 17);
		REQUIRE(res[0][6] == 9);
		REQUIRE(res[0][7] == 18);
		REQUIRE(res[1][2] == 6);
		REQUIRE(res[1][5] == 17);
		REQUIRE(res[1][6] == 9);
		REQUIRE(res[1][7] == 18);
		REQUIRE(res[2][5] == 11);
		REQUIRE(res[2][6] == 3);
		REQUIRE(res[2][7] == 12);
		REQUIRE(res[3][1] == 6);
		REQUIRE(res[3][5] == 23);
		REQUIRE(res[3][6] == 15);
		REQUIRE(res[3][7] == 7);
		REQUIRE(res[4][5] == 17);
		REQUIRE(res[4][6] == 9);
		REQUIRE(res[4][7] == 18);
		REQUIRE(res[5][6] == 3);
		REQUIRE(res[5][7] == 1);
		REQUIRE(res[6][5] == 8);
		REQUIRE(res[6][7] == 9);
	}

	SECTION("small graph 3") {
		node_handle handles[8];
		for (auto &handle : handles)
			handle = graph.addNode();

		graph.addEdge(handles[0], handles[2], 4);
		graph.addEdge(handles[0], handles[3], 4);
		graph.addEdge(handles[2], handles[0], 1);
		graph.addEdge(handles[2], handles[4], 7);
		graph.addEdge(handles[2], handles[5], 4);
		graph.addEdge(handles[2], handles[6], 5);
		graph.addEdge(handles[5], handles[1], 7);
		graph.addEdge(handles[5], handles[3], 1);
		graph.addEdge(handles[5], handles[7], 4);

		Matrix<int64_t> res = floydWarshall(graph);

		REQUIRE(res[0][1] == 15);
		REQUIRE(res[0][2] == 4);
		REQUIRE(res[0][3] == 4);
		REQUIRE(res[0][4] == 11);
		REQUIRE(res[0][5] == 8);
		REQUIRE(res[0][6] == 9);
		REQUIRE(res[0][7] == 12);
		REQUIRE(res[2][0] == 1);
		REQUIRE(res[2][1] == 11);
		REQUIRE(res[2][3] == 5);
		REQUIRE(res[2][4] == 7);
		REQUIRE(res[2][5] == 4);
		REQUIRE(res[2][6] == 5);
		REQUIRE(res[2][7] == 8);
		REQUIRE(res[5][1] == 7);
		REQUIRE(res[5][3] == 1);
		REQUIRE(res[5][7] == 4);




	}
}

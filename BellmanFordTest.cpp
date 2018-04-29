#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "ListDiGraph.h"
#include "BellmanFord.h"

TEST_CASE("Test Bellman Ford on graphs") {
	using namespace graphlib;
	using node_handle = typename graph_traits<ListDiGraph<PathNodeData, WeightedEdgeData>>::node_handle;
	ListDiGraph<PathNodeData, WeightedEdgeData> graph;
	node_handle handles[5];
	for (int i = 0; i < 5; ++i)
		handles[i] = graph.addNode();
	SECTION("Simple graph test") {
		graph.addEdge(handles[0], handles[4], 2);
		graph.addEdge(handles[0], handles[2], 5);
		graph.addEdge(handles[0], handles[3], 1);
		graph.addEdge(handles[4], handles[2], 1);
		graph.addEdge(handles[1], handles[2], 1);
		graph.addEdge(handles[3], handles[1], 2);
		REQUIRE(bellmanFord(graph, handles[0]));
		REQUIRE(graph.getNode(handles[0]).dist_ == 0);
		REQUIRE(graph.getNode(handles[1]).dist_ == 3);
		REQUIRE(graph.getNode(handles[2]).dist_ == 3);
		REQUIRE(graph.getNode(handles[3]).dist_ == 1);
		REQUIRE(graph.getNode(handles[4]).dist_ == 2);

		REQUIRE(graph.getNode(handles[0]).pred_ == node_handle());
		REQUIRE(graph.getNode(handles[1]).pred_ == handles[3]);
		REQUIRE(graph.getNode(handles[2]).pred_ == handles[4]);
		REQUIRE(graph.getNode(handles[3]).pred_ == handles[0]);
		REQUIRE(graph.getNode(handles[4]).pred_ == handles[0]);
	}
	SECTION("Graph with negative edges") {
		graph.addEdge(handles[0], handles[4], -2);
		graph.addEdge(handles[4], handles[2], -1);
		graph.addEdge(handles[2], handles[0], 4);
		graph.addEdge(handles[2], handles[1], 1);
		graph.addEdge(handles[1], handles[3], 1);
		graph.addEdge(handles[3], handles[0], 1);
		REQUIRE(bellmanFord(graph, handles[0]));
		REQUIRE(graph.getNode(handles[0]).dist_ == 0);
		REQUIRE(graph.getNode(handles[1]).dist_ == -2);
		REQUIRE(graph.getNode(handles[2]).dist_ == -3);
		REQUIRE(graph.getNode(handles[3]).dist_ == -1);
		REQUIRE(graph.getNode(handles[4]).dist_ == -2);

		REQUIRE(graph.getNode(handles[0]).pred_ == node_handle());
		REQUIRE(graph.getNode(handles[1]).pred_ == handles[2]);
		REQUIRE(graph.getNode(handles[2]).pred_ == handles[4]);
		REQUIRE(graph.getNode(handles[3]).pred_ == handles[1]);
		REQUIRE(graph.getNode(handles[4]).pred_ == handles[0]);
	}
	SECTION("Graph with negative cycle") {
		graph.addEdge(handles[0], handles[1], - 1);
		graph.addEdge(handles[1], handles[2], 2);
		graph.addEdge(handles[2], handles[0], -2);
		REQUIRE(bellmanFord(graph, handles[0]) == false);

		REQUIRE(graph.getNode(handles[0]).pred_ == handles[2]);
		REQUIRE(graph.getNode(handles[1]).pred_ == handles[0]);
		REQUIRE(graph.getNode(handles[2]).pred_ == handles[1]);
	}
}

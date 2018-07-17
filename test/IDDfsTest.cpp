#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "ListDiGraph.h"
#include "ListGraph.h"
#include "IDDfs.h"

TEST_CASE("Find node in ListDiGraph") {
	using namespace graphlib;
	using node_handle = typename graph_traits<ListDiGraph<TraversableNodeData, EdgeData>>::node_handle;

	ListDiGraph<TraversableNodeData, EdgeData> graph;
	node_handle handles[15];
	for (auto &handle : handles)
		handle = graph.addNode();
	for (int i = 0; i < 7; ++i) {
		graph.addEdge(handles[i], handles[2*i + 1]);
		graph.addEdge(handles[i], handles[2*i + 2]);
	}
	REQUIRE(iddfs(graph, handles[0], handles[14], 3));
	REQUIRE_FALSE(iddfs(graph, handles[0], handles[14], 2));
	REQUIRE(iddfs(graph, handles[1], handles[8], 2));
	REQUIRE_FALSE(iddfs(graph, handles[1], handles[8], 1));
	REQUIRE_FALSE(iddfs(graph, handles[1], handles[2], 10));
	REQUIRE_FALSE(iddfs(graph, handles[1], handles[11], 10));
}

TEST_CASE("Find node in ListGraph") {
	using namespace graphlib;
	using node_handle = typename graph_traits<ListGraph<TraversableNodeData, EdgeData>>::node_handle;

	ListGraph<TraversableNodeData, EdgeData> graph;
	node_handle handles[15];
	for (auto &handle : handles)
		handle = graph.addNode();
	for (int i = 0; i < 7; ++i) {
		graph.addEdge(handles[i], handles[2*i + 1]);
		graph.addEdge(handles[i], handles[2*i + 2]);
	}
	REQUIRE(iddfs(graph, handles[0], handles[14], 3));
	REQUIRE_FALSE(iddfs(graph, handles[0], handles[14], 2));
	REQUIRE(iddfs(graph, handles[1], handles[8], 2));
	REQUIRE_FALSE(iddfs(graph, handles[1], handles[8], 1));
	REQUIRE(iddfs(graph, handles[1], handles[2], 2));
	REQUIRE_FALSE(iddfs(graph, handles[1], handles[2], 1));
	REQUIRE(iddfs(graph, handles[10], handles[11], 6));
	REQUIRE_FALSE(iddfs(graph, handles[10], handles[11], 5));
}

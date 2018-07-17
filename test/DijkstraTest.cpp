#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "ListDiGraph.h"
#include "ListGraph.h"
#include "graph_traits.h"
#include "Dijkstra.h"
#include "FibonacciHeap.h"

TEST_CASE("Dijkstra on ListDiGraph") {
	using namespace graphlib;
	using Graph = ListDiGraph<PathNodeData, WeightedEdgeData>;
	using node_handle = typename graph_traits<Graph>::node_handle;
	Graph graph;
	node_handle handles[6];
	for (auto &handle : handles)
		handle = graph.addNode();

	SECTION("Dijkstra using BinHeap") {
		graph.addEdge(handles[0], handles[4], 2);
		graph.addEdge(handles[0], handles[2], 5);
		graph.addEdge(handles[0], handles[3], 1);
		graph.addEdge(handles[4], handles[2], 1);
		graph.addEdge(handles[1], handles[2], 1);
		graph.addEdge(handles[3], handles[1], 2);
		graph.addEdge(handles[4], handles[5], 0);
		graph.addEdge(handles[1], handles[5], 1);
		graph.addEdge(handles[3], handles[5], 0);
		graph.addEdge(handles[5], handles[5], 1);
		dijkstra(graph, handles[0]);
		REQUIRE(graph.getNode(handles[0]).dist_ == 0);
		REQUIRE(graph.getNode(handles[1]).dist_ == 3);
		REQUIRE(graph.getNode(handles[2]).dist_ == 3);
		REQUIRE(graph.getNode(handles[3]).dist_ == 1);
		REQUIRE(graph.getNode(handles[4]).dist_ == 2);
		REQUIRE(graph.getNode(handles[5]).dist_ == 1);

		REQUIRE(graph.getNode(handles[0]).pred_ == node_handle());
		REQUIRE(graph.getNode(handles[1]).pred_ == handles[3]);
		REQUIRE(graph.getNode(handles[2]).pred_ == handles[4]);
		REQUIRE(graph.getNode(handles[3]).pred_ == handles[0]);
		REQUIRE(graph.getNode(handles[4]).pred_ == handles[0]);
		REQUIRE(graph.getNode(handles[5]).pred_ == handles[3]);
	}

	SECTION("Dijkstra using FibonacciHeap") {
		graph.addEdge(handles[0], handles[4], 2);
		graph.addEdge(handles[0], handles[2], 5);
		graph.addEdge(handles[0], handles[3], 1);
		graph.addEdge(handles[4], handles[2], 1);
		graph.addEdge(handles[1], handles[2], 1);
		graph.addEdge(handles[3], handles[1], 2);
		graph.addEdge(handles[4], handles[5], 0);
		graph.addEdge(handles[1], handles[5], 1);
		graph.addEdge(handles[3], handles[5], 0);
		graph.addEdge(handles[5], handles[5], 1);
		dijkstra<Graph, FibonacciHeap<Graph>>(graph, handles[0]);
		REQUIRE(graph.getNode(handles[0]).dist_ == 0);
		REQUIRE(graph.getNode(handles[1]).dist_ == 3);
		REQUIRE(graph.getNode(handles[2]).dist_ == 3);
		REQUIRE(graph.getNode(handles[3]).dist_ == 1);
		REQUIRE(graph.getNode(handles[4]).dist_ == 2);
		REQUIRE(graph.getNode(handles[5]).dist_ == 1);

		REQUIRE(graph.getNode(handles[0]).pred_ == node_handle());
		REQUIRE(graph.getNode(handles[1]).pred_ == handles[3]);
		REQUIRE(graph.getNode(handles[2]).pred_ == handles[4]);
		REQUIRE(graph.getNode(handles[3]).pred_ == handles[0]);
		REQUIRE(graph.getNode(handles[4]).pred_ == handles[0]);
		REQUIRE(graph.getNode(handles[5]).pred_ == handles[3]);
	}
	SECTION("Dijkstra on Cormen graph") {
		graph.removeNode(handles[5]);
		graph.addEdge(handles[0], handles[1], 4);
		graph.addEdge(handles[0], handles[2], 13);
		graph.addEdge(handles[0], handles[4], 0);
		graph.addEdge(handles[1], handles[3], 0);
		graph.addEdge(handles[1], handles[4], 10);
		graph.addEdge(handles[2], handles[1], 0);
		graph.addEdge(handles[3], handles[2], 0);
		graph.addEdge(handles[3], handles[0], 2);
		graph.addEdge(handles[4], handles[3], 2);
		dijkstra(graph, handles[0]);
		REQUIRE(graph.getNode(handles[0]).dist_ == 0);
		REQUIRE(graph.getNode(handles[1]).dist_ == 2);
		REQUIRE(graph.getNode(handles[2]).dist_ == 2);
		REQUIRE(graph.getNode(handles[3]).dist_ == 2);
		REQUIRE(graph.getNode(handles[4]).dist_ == 0);
	}
}

TEST_CASE("Dijkstra on ListGraph") {
	using namespace graphlib;
	using Graph = ListGraph<PathNodeData, WeightedEdgeData>;
	using node_handle = typename graph_traits<Graph>::node_handle;

	Graph graph;
	node_handle handles[6];
	for (auto &handle : handles)
		handle = graph.addNode();

	SECTION("Dijkstra using BinHeap") {
		graph.addEdge(handles[0], handles[4], 2);
		graph.addEdge(handles[0], handles[2], 5);
		graph.addEdge(handles[0], handles[3], 1);
		graph.addEdge(handles[4], handles[2], 1);
		graph.addEdge(handles[1], handles[2], 1);
		graph.addEdge(handles[3], handles[1], 2);
		graph.addEdge(handles[4], handles[5], 0);
		graph.addEdge(handles[1], handles[5], 1);
		graph.addEdge(handles[3], handles[5], 0);
		graph.addEdge(handles[5], handles[5], 1);
		dijkstra(graph, handles[0]);
		REQUIRE(graph.getNode(handles[0]).dist_ == 0);
		REQUIRE(graph.getNode(handles[1]).dist_ == 2);
		REQUIRE(graph.getNode(handles[2]).dist_ == 2);
		REQUIRE(graph.getNode(handles[3]).dist_ == 1);
		REQUIRE(graph.getNode(handles[4]).dist_ == 1);
		REQUIRE(graph.getNode(handles[5]).dist_ == 1);

		REQUIRE(graph.getNode(handles[0]).pred_ == node_handle());
		REQUIRE(graph.getNode(handles[1]).pred_ == handles[5]);
		REQUIRE(graph.getNode(handles[2]).pred_ == handles[4]);
		REQUIRE(graph.getNode(handles[3]).pred_ == handles[0]);
		REQUIRE(graph.getNode(handles[4]).pred_ == handles[5]);
		REQUIRE(graph.getNode(handles[5]).pred_ == handles[3]);
	}
	SECTION("Dijkstra using FibonacciHeap") {
		graph.addEdge(handles[0], handles[4], 2);
		graph.addEdge(handles[0], handles[2], 5);
		graph.addEdge(handles[0], handles[3], 1);
		graph.addEdge(handles[4], handles[2], 1);
		graph.addEdge(handles[1], handles[2], 1);
		graph.addEdge(handles[3], handles[1], 2);
		graph.addEdge(handles[4], handles[5], 0);
		graph.addEdge(handles[1], handles[5], 1);
		graph.addEdge(handles[3], handles[5], 0);
		graph.addEdge(handles[5], handles[5], 1);
		dijkstra<Graph, FibonacciHeap<Graph>>(graph, handles[0]);
		REQUIRE(graph.getNode(handles[0]).dist_ == 0);
		REQUIRE(graph.getNode(handles[1]).dist_ == 2);
		REQUIRE(graph.getNode(handles[2]).dist_ == 2);
		REQUIRE(graph.getNode(handles[3]).dist_ == 1);
		REQUIRE(graph.getNode(handles[4]).dist_ == 1);
		REQUIRE(graph.getNode(handles[5]).dist_ == 1);

		REQUIRE(graph.getNode(handles[0]).pred_ == node_handle());
		REQUIRE(graph.getNode(handles[1]).pred_ == handles[5]);
		REQUIRE(graph.getNode(handles[2]).pred_ == handles[4]);
		REQUIRE(graph.getNode(handles[3]).pred_ == handles[0]);
		REQUIRE(graph.getNode(handles[4]).pred_ == handles[5]);
		REQUIRE(graph.getNode(handles[5]).pred_ == handles[3]);
	}
	SECTION("Dijkstra on Cormen graph") {
		graph.removeNode(handles[5]);
		graph.addEdge(handles[0], handles[1], 4);
		graph.addEdge(handles[0], handles[2], 13);
		graph.addEdge(handles[0], handles[4], 0);
		graph.addEdge(handles[1], handles[3], 0);
		graph.addEdge(handles[1], handles[4], 10);
		graph.addEdge(handles[2], handles[1], 0);
		graph.addEdge(handles[3], handles[2], 0);
		graph.addEdge(handles[3], handles[0], 2);
		graph.addEdge(handles[4], handles[3], 2);
		dijkstra(graph, handles[0]);
		REQUIRE(graph.getNode(handles[0]).dist_ == 0);
		REQUIRE(graph.getNode(handles[1]).dist_ == 2);
		REQUIRE(graph.getNode(handles[2]).dist_ == 2);
		REQUIRE(graph.getNode(handles[3]).dist_ == 2);
		REQUIRE(graph.getNode(handles[4]).dist_ == 0);
	}

}

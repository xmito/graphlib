#define CATCH_CONFIG_MAIN
#include <climits>
#include "catch.hpp"
#include "ListGraph.h"
#include "ListDiGraph.h"
#include "dag.h"

TEST_CASE( "Dag on graph1" ) {
	using namespace graphlib;
	using graph_type = ListDiGraph<PathNodeData, WeightedEdgeData>;
	using node_handle = typename graph_type::node_handle;
	using distance_type = typename graph_traits<graph_type>::distance_type;

	ListDiGraph<PathNodeData, WeightedEdgeData> graph;

	node_handle handles[6];
	for (size_t i = 0; i < 6; ++i) {
		handles[i] = graph.addNode();
	}
	graph.addEdge(handles[0], handles[1], 5);
	graph.addEdge(handles[0], handles[2], 3);
	graph.addEdge(handles[1], handles[3], 6);
	graph.addEdge(handles[1], handles[2], 2);
	graph.addEdge(handles[2], handles[4], 4);
	graph.addEdge(handles[2], handles[5], 2);
	graph.addEdge(handles[2], handles[3], 7);
	graph.addEdge(handles[3], handles[4], -1);
	graph.addEdge(handles[4], handles[5], -2);

	SECTION("Test with no backEdges, source being handle[0]") {
		node_handle source = handles[0];
		dag(graph, source);
		REQUIRE(graph.getNode(handles[0]).dist_ == 0);
		REQUIRE(graph.getNode(handles[1]).dist_ == 5);
		REQUIRE(graph.getNode(handles[2]).dist_ == 3);
		REQUIRE(graph.getNode(handles[3]).dist_ == 10);
		REQUIRE(graph.getNode(handles[4]).dist_ == 7);
		REQUIRE(graph.getNode(handles[5]).dist_ == 5);
	}

	SECTION("Test with no backEdges, source being handle[1]") {
	 
		node_handle source = handles[1];
		dag(graph, source);
		REQUIRE(graph.getNode(handles[0]).dist_ == std::numeric_limits<distance_type>::max());
		REQUIRE(graph.getNode(handles[1]).dist_ == 0);
		REQUIRE(graph.getNode(handles[2]).dist_ == 2);
		REQUIRE(graph.getNode(handles[3]).dist_ == 6);
		REQUIRE(graph.getNode(handles[4]).dist_ == 5);
		REQUIRE(graph.getNode(handles[5]).dist_ == 3);
			
	}

	SECTION("Test with a backEdge") {
	 
		node_handle source = handles[0];
		graph.addEdge(handles[1], handles[0], 42);
		dag(graph, source);

		REQUIRE(graph.getNode(handles[0]).dist_ == 0);
		REQUIRE(graph.getNode(handles[1]).dist_ == std::numeric_limits<distance_type>::max());
		REQUIRE(graph.getNode(handles[2]).dist_ == std::numeric_limits<distance_type>::max());
		REQUIRE(graph.getNode(handles[3]).dist_ == std::numeric_limits<distance_type>::max());
		REQUIRE(graph.getNode(handles[4]).dist_ == std::numeric_limits<distance_type>::max());
		REQUIRE(graph.getNode(handles[5]).dist_ == std::numeric_limits<distance_type>::max());
			
	}
}

TEST_CASE( "Dag on graph2" ) {
	using namespace graphlib;
	using graph_type = ListDiGraph<PathNodeData, WeightedEdgeData>;
	using node_handle = typename graph_type::node_handle;

	ListDiGraph<PathNodeData, WeightedEdgeData> graph;

	node_handle handles[8];
	for (size_t i = 0; i < 8; ++i) {
		handles[i] = graph.addNode();
	}
	graph.addEdge(handles[0], handles[1], 3);
	graph.addEdge(handles[0], handles[2], 6);
	graph.addEdge(handles[1], handles[2], 4);

	graph.addEdge(handles[1], handles[3], 4);
	graph.addEdge(handles[1], handles[4], 11);

	graph.addEdge(handles[2], handles[3], 8);
	graph.addEdge(handles[2], handles[6], 11);

	graph.addEdge(handles[3], handles[4], -4);
	graph.addEdge(handles[3], handles[5], 5);
	graph.addEdge(handles[3], handles[6], 2);

	graph.addEdge(handles[4], handles[7], 9);
	graph.addEdge(handles[5], handles[7], 1);
	graph.addEdge(handles[6], handles[7], 2);

	SECTION("Test with no backEdges, source being handle[0]") {
		node_handle source = handles[0];
		dag(graph, source);
		REQUIRE(graph.getNode(handles[0]).dist_ == 0);
		REQUIRE(graph.getNode(handles[1]).dist_ == 3);
		REQUIRE(graph.getNode(handles[2]).dist_ == 6);
		REQUIRE(graph.getNode(handles[3]).dist_ == 7);
		REQUIRE(graph.getNode(handles[4]).dist_ == 3);
		REQUIRE(graph.getNode(handles[5]).dist_ == 12);
		REQUIRE(graph.getNode(handles[6]).dist_ == 9);
		REQUIRE(graph.getNode(handles[7]).dist_ == 11);
	}
}

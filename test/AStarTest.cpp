#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "AStar.cpp"
#include "ListDiGraph.h"
#include "ListGraph.h"
#include "NodeData.h"

#include <vector>
#include <set>
#include <iostream>

using namespace graphlib;

template<typename Graph>
std::vector<typename graph_traits<Graph>::node_handle>
creatGridNodes(Graph &graph, size_t x, size_t y) {
	using node_handle = typename graph_traits<Graph>::node_handle;
	std::vector<node_handle> vec;
	vec.reserve(x*y);
	for (size_t i = 0; i < x*y; ++i)
		vec.push_back(graph.addNode(i % x, i / x));
	return vec;
}

template<typename Graph>
void creatGridEdges(Graph &graph,
                const std::vector<typename graph_traits<Graph>::node_handle> &handles,
                size_t y,
                bool diagonal = true,
                const std::set<typename graph_traits<Graph>::node_handle> &phandles = {}) {
	size_t x = handles.size() / y;
	for (size_t i = 0; i < y; ++i) {
		for (size_t j = 1; j < x; ++j) {
			if (phandles.find(handles[i*x + j - 1]) != phandles.end() ||
			        phandles.find(handles[i*x + j]) != phandles.end())
				continue;
			graph.addEdge(handles[i*x + j - 1], handles[i*x + j]);
		}
	}
	for (size_t i = 0; i < x; ++i) {
		for (size_t j = 1; j < y; ++j) {
			if (phandles.find(handles[(j - 1)*x + i]) != phandles.end() ||
			        phandles.find(handles[j*x + i]) != phandles.end())
				continue;
			graph.addEdge(handles[(j - 1)*x + i], handles[j*x + i]);
		}
	}
	if (diagonal) {
		size_t roff = x + 1;
		size_t loff = x - 1;
		for (size_t i = 0; i < (x*y - x); ++i) {
			if (phandles.find(handles[i]) != phandles.end())
				continue;
			if (i % x != x - 1 &&
			        phandles.find(handles[i + roff]) == phandles.end()) {
				graph.addEdge(handles[i], handles[i + roff]);
			}
			if (i % x != 0 &&
			        phandles.find(handles[i + loff]) == phandles.end()) {
				graph.addEdge(handles[i], handles[i + loff]);
			}
		}
	}
}

template<typename Graph>
std::vector<typename graph_traits<Graph>::node_handle>
creatGrid(Graph &graph,
          size_t x,
          size_t y,
          bool diagonal = true) {
	using node_handle = typename graph_traits<Graph>::node_handle;
	std::vector<node_handle> handles(creatGridNodes(graph, x, y));
	creatGridEdges(graph, handles, y, diagonal);
	return handles;
}

TEST_CASE("A* on ListGraph") {
	using namespace graphlib;
	using Graph = ListGraph<LocationNodeData<PlaneLocation>, WeightedEdgeData>;
	using node_handle = typename graph_traits<Graph>::node_handle;

	Graph graph;
	SECTION("manhattanDistance test 1") {
		/* o x o o o o
		 * o o o x o o
		 * o x x x x o
		 * S o x o x o
		 * o o o o x E */
		std::vector<node_handle> handles(creatGridNodes(graph, 6, 5));
		std::set<node_handle> phandles{handles[1], handles[9], handles[13], handles[14],
			        handles[15], handles[16], handles[20], handles[22], handles[28]};
		creatGridEdges(graph, handles, 5, false, phandles);

		aStar(graph, handles[18], handles[29], manhattanDistance<Graph>);
		node_handle last = handles[29];
		REQUIRE(graph.getNode(handles[29]).dist_ == 12);
		REQUIRE(graph.getNode(handles[29]).pred_ == handles[23]);
		REQUIRE(graph.getNode(handles[23]).pred_ == handles[17]);
		REQUIRE(graph.getNode(handles[17]).pred_ == handles[11]);
		REQUIRE(graph.getNode(handles[11]).pred_ == handles[5]);
		REQUIRE(graph.getNode(handles[5]).pred_ == handles[4]);
		REQUIRE(graph.getNode(handles[4]).pred_ == handles[3]);
		REQUIRE(graph.getNode(handles[3]).pred_ == handles[2]);
		REQUIRE(graph.getNode(handles[2]).pred_ == handles[8]);
		REQUIRE(graph.getNode(handles[8]).pred_ == handles[7]);
		REQUIRE(graph.getNode(handles[7]).pred_ == handles[6]);
		REQUIRE(graph.getNode(handles[6]).pred_ == handles[12]);
		REQUIRE(graph.getNode(handles[12]).pred_ == handles[18]);
	}
	SECTION("manhattanDistance test 2") {
		/* o o o o o o E
		 * o o x x x o o
		 * o o o o x o o
		 * o o o o x o o
		 * S x x x x o o
		 * o o o o o o o */
		std::vector<node_handle> handles(creatGridNodes(graph, 7, 6));
		std::set<node_handle> phandles{handles[9], handles[10], handles[11], handles[18],
			                          handles[25], handles[29], handles[30], handles[31],
			                          handles[32]};
		creatGridEdges(graph, handles, 6, false, phandles);
		aStar(graph, handles[28], handles[6], manhattanDistance<Graph>);
		REQUIRE(graph.getNode(handles[6]).dist_ == 10);
	}
	SECTION("euclideanDistance simple grid") {
		/* S o o o o o o o o o
		 * o o o o o o o o o o
		 * o o o o o o o o o o
		 * o o o o o o o o o o
		 * o o o o o o o o o o
		 * o o o o o o o o o E */
		std::vector<node_handle> handles(creatGrid(graph, 10, 6));
		aStar(graph, handles[0], handles[59], euclideanDistance<Graph>);
		REQUIRE(graph.getNode(handles[59]).dist_ == 9);
	}
	SECTION("euclideanDistance test 1") {
		/* o x o o o o
		 * o o o x o o
		 * o x x x x o
		 * S o x o x o
		 * o o o o x E */
		std::vector<node_handle> handles(creatGridNodes(graph, 6, 5));
		std::set<node_handle> phandles{handles[1], handles[9], handles[13], handles[14],
			        handles[15], handles[16], handles[20], handles[22], handles[28]};
		creatGridEdges(graph, handles, 5, true, phandles);

		aStar(graph, handles[18], handles[29], euclideanDistance<Graph>);
		node_handle last = handles[29];
		REQUIRE(graph.getNode(handles[29]).dist_ == 8);
		REQUIRE(graph.getNode(handles[29]).pred_ == handles[23]);
		REQUIRE(graph.getNode(handles[23]).pred_ == handles[17]);
		REQUIRE(graph.getNode(handles[17]).pred_ == handles[10]);
		REQUIRE(graph.getNode(handles[10]).pred_ == handles[3]);
		REQUIRE(graph.getNode(handles[3]).pred_ == handles[8]);
		REQUIRE(graph.getNode(handles[8]).pred_ == handles[7]);
		REQUIRE(graph.getNode(handles[7]).pred_ == handles[12]);
		REQUIRE(graph.getNode(handles[12]).pred_ == handles[18]);
	}
	SECTION("euclideanDistance test 2") {
		/* o o o o o o E
		 * o o x x x o o
		 * o o o o x o o
		 * o o o o x o o
		 * S x x x x o o
		 * o o o o o o o */
		std::vector<node_handle> handles(creatGridNodes(graph, 7, 6));
		std::set<node_handle> phandles{handles[9], handles[10], handles[11], handles[18],
			                          handles[25], handles[29], handles[30], handles[31],
			                          handles[32]};
		creatGridEdges(graph, handles, 6, true, phandles);
		aStar(graph, handles[28], handles[6], euclideanDistance<Graph>);
		REQUIRE(graph.getNode(handles[6]).dist_ == 8);
	}
	SECTION("diagonalDistance simple grid") {
		/* S o o o o o o o o o
		 * o o o o o o o o o o
		 * o o o o o o o o o o
		 * o o o o o o o o o o
		 * o o o o o o o o o o
		 * o o o o o o o o o E */
		std::vector<node_handle> handles(creatGrid(graph, 10, 6));
		aStar(graph, handles[0], handles[59], diagonalDistance<Graph>);
		REQUIRE(graph.getNode(handles[59]).dist_ == 9);
	}
	SECTION("diagonalDistance test 1") {
		/* o x o o o o
		 * o o o x o o
		 * o x x x x o
		 * S o x o x o
		 * o o o o x E */
		std::vector<node_handle> handles(creatGridNodes(graph, 6, 5));
		std::set<node_handle> phandles{handles[1], handles[9], handles[13], handles[14],
			        handles[15], handles[16], handles[20], handles[22], handles[28]};
		creatGridEdges(graph, handles, 5, true, phandles);

		aStar(graph, handles[18], handles[29], diagonalDistance<Graph>);
		node_handle last = handles[29];
		REQUIRE(graph.getNode(handles[29]).dist_ == 8);
		REQUIRE(graph.getNode(handles[29]).pred_ == handles[23]);
		REQUIRE(graph.getNode(handles[23]).pred_ == handles[17]);
		REQUIRE(graph.getNode(handles[17]).pred_ == handles[10]);
		REQUIRE(graph.getNode(handles[10]).pred_ == handles[3]);
		REQUIRE(graph.getNode(handles[3]).pred_ == handles[8]);
		REQUIRE(graph.getNode(handles[8]).pred_ == handles[7]);
		REQUIRE(graph.getNode(handles[7]).pred_ == handles[12]);
		REQUIRE(graph.getNode(handles[12]).pred_ == handles[18]);
	}
	SECTION("diagonalDistance test 2") {
		/* o o o o o o E
		 * o o x x x o o
		 * o o o o x o o
		 * o o o o x o o
		 * S x x x x o o
		 * o o o o o o o */
		std::vector<node_handle> handles(creatGridNodes(graph, 7, 6));
		std::set<node_handle> phandles{handles[9], handles[10], handles[11], handles[18],
			                          handles[25], handles[29], handles[30], handles[31],
			                          handles[32]};
		creatGridEdges(graph, handles, 6, true, phandles);
		aStar(graph, handles[28], handles[6], diagonalDistance<Graph>);
		REQUIRE(graph.getNode(handles[6]).dist_ == 8);
	}
}

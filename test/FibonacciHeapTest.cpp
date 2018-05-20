#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "FibonacciHeap.h"
#include "ListDiGraph.h"
#include "GraphTraits.h"

TEST_CASE( "FibHeap::FibHeap(*)/operator=" ) {
	using namespace graphlib;
	using Graph = ListDiGraph<PathNodeData, WeightedEdgeData>;
	using node_handle = typename graph_traits<Graph>::node_handle;
	Graph graph;
	LessDistance<Graph> comp(&graph);
	std::vector<node_handle> handles;
	for (int i = 0; i < 20; ++i) {
		if (i > 0)
			handles.push_back(graph.addNode(Color::BLACK, handles[i - 1], i));
		else
			handles.push_back(graph.addNode(Color::BLACK, node_handle(), 0));
	}
	SECTION("FibHeap(const Compare&)") {
		FibHeap<Graph> fh(comp);
		REQUIRE(fh.empty());
		REQUIRE(fh.size() == 0);
	}
	SECTION("FibHeap(std::initializer_list<value_type>, const Compare&)") {
		FibHeap<Graph> fh({handles[0], handles[1], handles[2], handles[3]}, comp);
		REQUIRE(fh.size() == 4);
		REQUIRE(fh.top() == handles[0]);
	}
	SECTION("FibHeap(InputIt first, InputIt last, const Compare&)") {
		FibHeap<Graph> fh(handles.begin(), handles.end(), comp);
		REQUIRE(fh.size() == 20);
		REQUIRE(fh.empty() == false);
		REQUIRE(fh.top() == handles[0]);
	}
	SECTION("FibHeap(const Graph& graph)") {
		FibHeap<Graph> fh(graph);
		REQUIRE(fh.size() == 20);
		REQUIRE(fh.empty() == false);
		REQUIRE(fh.top() == handles[0]);
	}
	SECTION("FibHeap(const Graph& graph, const Compare &comp)") {
		FibHeap<Graph> fh(graph, comp);
		REQUIRE(fh.size() == 20);
		REQUIRE(fh.empty() == false);
		REQUIRE(fh.top() == handles[0]);
	}
	SECTION("FibHeap(const FibHeap&)") {
		FibHeap<Graph> fh(graph);
		FibHeap<Graph> fh_cp(fh);
		REQUIRE(fh_cp.size() == 20);
		REQUIRE(fh_cp.empty() == false);
		REQUIRE(fh_cp.top() == handles[0]);
	}
	SECTION("operator=(const FibHeap&)") {
		FibHeap<Graph> fh(graph);
		FibHeap<Graph> fh_cp(graph.endNode(), graph.endNode(), comp);
		fh_cp = fh;
		REQUIRE(fh_cp.size() == 20);
		REQUIRE(fh_cp.empty() == false);
		REQUIRE(fh_cp.top() == handles[0]);
	}
	SECTION("FibHeap(FibHeap&&)") {
		FibHeap<Graph> fh(graph);
		FibHeap<Graph> fh_mv(std::move(fh));
		REQUIRE(fh.empty());
		REQUIRE(fh.size() == 0);
		REQUIRE(fh_mv.empty() == false);
		REQUIRE(fh_mv.size() == 20);
	}
	SECTION("operator=(FibHeap&&)") {
		FibHeap<Graph> fh(graph);
		FibHeap<Graph> fh_mv(comp);
		fh_mv = std::move(fh);
		REQUIRE(fh.empty());
		REQUIRE(fh.size() == 0);
		REQUIRE(fh_mv.empty() == false);
		REQUIRE(fh_mv.size() == 20);
	}
}

TEST_CASE("FibHeap getters") {
	using namespace graphlib;
	using Graph = ListDiGraph<PathNodeData, WeightedEdgeData>;
	using node_handle = typename graph_traits<Graph>::node_handle;
	Graph graph;
	LessDistance<Graph> comp(&graph);
	std::vector<node_handle> handles;
	for (int i = 0; i < 20; ++i) {
		if (i > 0)
			handles.push_back(graph.addNode(Color::BLACK, handles[i - 1], i));
		else
			handles.push_back(graph.addNode(Color::BLACK, node_handle(), 0));
	}
	FibHeap<Graph> fh(graph);
	SECTION("FibHeap::size()") {
		REQUIRE(fh.size() == 20);
	}
	SECTION("FibHeap::empty()") {
		FibHeap<Graph> fh_empty(graph.endNode(), graph.endNode(), comp);
		REQUIRE(fh_empty.empty());
		REQUIRE(fh.empty() == false);
	}
	SECTION("FibHeap::top()") {
		REQUIRE(fh.top() == handles[0]);
	}
}

TEST_CASE("FibHeap modifiers") {
	using namespace graphlib;
	using Graph = ListDiGraph<PathNodeData, WeightedEdgeData>;
	using node_handle = typename graph_traits<Graph>::node_handle;
	Graph graph;
	LessDistance<Graph> comp(&graph);
	std::vector<node_handle> handles;
	for (int i = 0; i < 20; ++i) {
		if (i > 0)
			handles.push_back(graph.addNode(Color::BLACK, handles[i - 1], i + 20));
		else
			handles.push_back(graph.addNode(Color::BLACK, node_handle(), 20));
	}
	SECTION( "FibHeap::push(const value_type&)") {
		FibHeap<Graph> fh(graph.endNode(), graph.endNode(), comp);
		for (int i = 19; i >= 0; --i) {
			fh.push(handles[i]);
			REQUIRE(fh.top() == handles[i]);
		}
	}

	SECTION("FibHeap::pop()") {
		FibHeap<Graph> fh(graph);
		for (int i = 0; i < 20; ++i) {
			REQUIRE(fh.top() == handles[i]);
			fh.pop();
		}
	}
	SECTION("FibHeap::decUpdate()") {
		FibHeap<Graph> fh(graph);
		for (int i = 1; i <= 20; ++i) {
			auto &data = graph.getNode(handles[20 - i]);
			data.dist_ = 20 - i;
			fh.decUpdate(handles[20 - i]);
			REQUIRE(fh.top() == handles[20 - i]);
		}
	}
}



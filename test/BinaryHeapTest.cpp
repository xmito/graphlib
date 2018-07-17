#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "BinaryHeap.h"
#include "ListDiGraph.h"
#include "graph_traits.h"
#include "Comparators.h"
#include <iostream>
#include <list>

TEST_CASE( "BinaryHeap constructors" ) {
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

	SECTION("BinaryHeap(const Compare&)") {
		BinaryHeap<Graph> bh_one(comp);
		REQUIRE(bh_one.empty());
		REQUIRE(bh_one.size() == 0);
	}
	SECTION("BinaryHeap(InputIt, InputIt, const Compare &)") {
		BinaryHeap<Graph> bh(handles.begin(), handles.end(), comp);
		REQUIRE(bh.size() == 20);
		REQUIRE_FALSE(bh.empty());
		REQUIRE(bh.top() == handles[0]);
	}
	SECTION("BinaryHeap::BinaryHeap(std::initializer_list<value_type>, const Compare&)") {
		BinaryHeap<Graph> bh({handles[0], handles[1], handles[2]}, comp);
		REQUIRE(bh.size() == 3);
		REQUIRE_FALSE(bh.empty());
		REQUIRE(bh.top() == handles[0]);
	}
	SECTION("BinaryHeap::BinaryHeap(BinaryHeap&&)") {
		BinaryHeap<Graph> bh(graph);
		BinaryHeap<Graph> bh_mv(std::move(bh));
		REQUIRE(bh_mv.size() == 20);
		REQUIRE_FALSE(bh_mv.empty());
		REQUIRE(bh_mv.top() == handles[0]);
		REQUIRE(bh.empty());
		REQUIRE(bh.size() == 0);
	}
	SECTION("BinaryHeap::operator=(BinaryHeap&&)") {
		BinaryHeap<Graph> bh_one(graph);
		BinaryHeap<Graph> bh_two(comp);
		bh_two = std::move(bh_one);
		REQUIRE(bh_one.empty());
		REQUIRE(bh_one.size() == 0);
		REQUIRE_FALSE(bh_two.empty());
		REQUIRE(bh_two.size() == 20);
		REQUIRE(bh_two.top() == handles[0]);
	}
}

TEST_CASE("Methods") {
	using namespace graphlib;
	using Graph = ListDiGraph<PathNodeData, WeightedEdgeData>;
	using node_handle = typename graph_traits<Graph>::node_handle;
	using heap_handle = typename BinaryHeap<Graph>::heap_handle;
	Graph graph;
	LessDistance<Graph> comp(&graph);
	std::vector<node_handle> handles;
	for (int i = 0; i < 20; ++i) {
		if (i > 0)
			handles.push_back(graph.addNode(Color::BLACK, handles[i - 1], i));
		else
			handles.push_back(graph.addNode(Color::BLACK, node_handle(), 0));
	}
	BinaryHeap<Graph> bh(comp);
	SECTION("BinaryHeap::push(const node_handle&)") {
		for (int i = 19; i >= 0; --i) {
			BinaryHeap<Graph>::Handle handle = bh.push(handles[i]);
			REQUIRE(bh.top() == handles[i]);
			REQUIRE(bh.size() == 20 - i);
			REQUIRE(bh.get(handle) == handles[i]);
			REQUIRE(bh.topHandle() == handle);
		}
	}
	SECTION("BinaryHeap::pop()") {
		bh.pop();
		REQUIRE(bh.empty());
		for (int i = 0; i < 20; ++i)
			bh.push(handles[i]);
		for (int i = 0; i < 20; ++i) {
			REQUIRE(bh.top() == handles[i]);
			bh.pop();
		}
	}
	SECTION("BinaryHeap::swap(BinaryHeap&)") {
		BinaryHeap<Graph> bh_one(comp);
		BinaryHeap<Graph> bh_two(comp);
		for (int i = 0; i < 9; ++i) {
			bh_one.push(handles[i]);
			bh_two.push(handles[i + 1]);
		}
		bh_one.swap(bh_two);
		for (int i = 0; i < 9; ++i) {
			REQUIRE(bh_one.top() == handles[i + 1]);
			REQUIRE(bh_two.top() == handles[i]);
			bh_one.pop();
			bh_two.pop();
		}
	}
	SECTION("BinaryHeap::decUpdate(const node_handle &)") {
		BinaryHeap<Graph> bh(graph);
		auto &data = graph.getNode(handles[0]);
		data.dist_ = 30;
		auto bh_handle = bh.decUpdate(handles[0]);
		REQUIRE(bh.get(bh_handle) == handles[0]);
		REQUIRE(bh.top() == handles[1]);

		auto &data_two = graph.getNode(handles[10]);
		data_two.dist_ = 100;
		auto bh_handle_two = bh.decUpdate(handles[10]);
		REQUIRE(bh.get(bh_handle_two) == handles[10]);
		REQUIRE(bh.topHandle() != bh_handle_two);
	}
	SECTION("BinaryHeap::decUpdate(const heap_handle &)") {
		BinaryHeap<Graph> bh(graph);
		node_handle top = bh.top();
		auto &data = graph.getNode(top);
		data.dist_ = 100;
		bh.decUpdate(bh.topHandle());
		REQUIRE(bh.top() == handles[1]);
	}
	SECTION("BinaryHeap::operator==/operator!=") {
		REQUIRE(bh == bh);
		BinaryHeap<Graph> bh_two(comp);
		bh.push(handles[0]);
		REQUIRE(bh != bh_two);
		REQUIRE_FALSE((bh == bh_two));
	}
	SECTION("BinaryHeap::topHandle()") {
		BinaryHeap<Graph> bh(comp);
		for (int i = 10; i >= 0; --i)
			bh.push(handles[i]);
		auto handle = bh.topHandle();
		REQUIRE(bh.get(handle) == handles[0]);
	}
	SECTION("BinaryHeap::top()") {
		BinaryHeap<Graph> bh(graph);
		REQUIRE(bh.top() == handles[0]);
	}
	SECTION("BinaryHeap::get()") {
		heap_handle hh = bh.push(handles[0]);
		node_handle nh = bh.get(hh);
		REQUIRE(nh == handles[0]);
	}
	SECTION("BinaryHeap::getHandle()") {
		heap_handle hh = bh.push(handles[0]);
		REQUIRE(bh.getHandle(handles[0]) == hh);
	}
}

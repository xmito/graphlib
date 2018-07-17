#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "ListGraph.h"
#include "ListDiGraph.h"
#include "Dfs.h"
#include <random>


TEST_CASE("Traversal tests on ListDiGraph") {
	using namespace graphlib;
	using node_handle = typename ListDiGraph<TraversableNodeData, EdgeData>::node_handle;
	ListDiGraph<TraversableNodeData, EdgeData> graph;
	node_handle handles[30];
	for (auto &handle : handles)
		handle = graph.addNode();

	SECTION("Simple traversal test") {
		for (int i = 1; i < 30; ++i)
			graph.addEdge(handles[i - 1], handles[i]);
		graph.addEdge(handles[29], handles[0]);
		dfs(graph);
		for (int i = 1; i < 30; ++i) {
			auto &data = graph.getNode(handles[i]);
			REQUIRE(data.color_ == Color::BLACK);
			REQUIRE(data.pred_ == handles[i - 1]);
		}
		auto &data = graph.getNode(handles[0]);
		REQUIRE(data.color_ == Color::BLACK);
		REQUIRE(data.pred_ == node_handle());
	}

	SECTION("Test with random set of edges") {
		std::random_device rdev;
		std::default_random_engine eng(rdev());
		std::uniform_int_distribution<int> dist(0, 29);
		for (int i = 0; i < 60; ++i)
			graph.addEdge(handles[dist(eng)], handles[dist(eng)]);
		for (int i = 1; i < 30; ++i)
			graph.addEdge(handles[0], handles[i]);
		dfs(graph);
		for (int i = 1; i < 30; ++i) {
			auto &data = graph.getNode(handles[i]);
			REQUIRE(data.color_ == Color::BLACK);
			node_handle nh = data.pred_;
			while(nh != handles[0]) {
				auto &pred_data = graph.getNode(nh);
				nh = pred_data.pred_;
			}
		}
		auto &data = graph.getNode(handles[0]);
		REQUIRE(data.color_ == Color::BLACK);
		REQUIRE(data.pred_ == node_handle());
	}
}

TEST_CASE("Traversal tests on ListGraph") {
	using namespace graphlib;
	using node_handle = typename graph_traits<ListGraph<TraversableNodeData, EdgeData>>::node_handle;
	ListGraph<TraversableNodeData, EdgeData> graph;
	node_handle handles[30];
	for (auto &handle : handles)
		handle = graph.addNode();
	SECTION("Simple traversal test") {
		for (int i = 1; i < 30; ++i)
			graph.addEdge(handles[i - 1], handles[i]);
		graph.addEdge(handles[29], handles[0]);
		dfs(graph);
		for (int i = 1; i < 30; ++i) {
			auto &data = graph.getNode(handles[i]);
			REQUIRE(data.color_ == Color::BLACK);
			REQUIRE(data.pred_ == handles[i - 1]);
		}
		auto &data = graph.getNode(handles[0]);
		REQUIRE(data.color_ == Color::BLACK);
		REQUIRE(data.pred_ == node_handle());
	}
	SECTION("Test with random set of edges") {
		std::random_device rdev;
		std::default_random_engine eng(rdev());
		std::uniform_int_distribution<int> dist(0, 29);
		for (int i = 0; i < 60; ++i)
			graph.addEdge(handles[dist(eng)], handles[dist(eng)]);
		for (int i = 1; i < 30; ++i)
			graph.addEdge(handles[0], handles[i]);
		dfs(graph);
		for (int i = 1; i < 30; ++i) {
			auto &data = graph.getNode(handles[i]);
			REQUIRE(data.color_ == Color::BLACK);
			node_handle nh = data.pred_;
			while(nh != handles[0]) {
				auto &pred_data = graph.getNode(nh);
				nh = pred_data.pred_;
			}
		}
		auto &data = graph.getNode(handles[0]);
		REQUIRE(data.color_ == Color::BLACK);
		REQUIRE(data.pred_ == node_handle());
	}
}

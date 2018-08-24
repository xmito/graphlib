#define CATCH_CONFIG_MAIN
#include "Bfs.h"
#include "ListDiGraph.h"
#include "ListGraph.h"
#include "catch.hpp"
#include <random>
#include <syslog.h>

TEST_CASE("Traversal tests on ListDiGraph") {
    using namespace graphlib;
    using node_handle = typename ListDiGraph<PathNodeData, EdgeData>::node_handle;
    ListDiGraph<PathNodeData, EdgeData> graph;
    node_handle handles[30];
    for (auto &handle : handles)
        handle = graph.addNode();

    SECTION("Simple traversal test") {
        for (int i = 1; i < 30; ++i)
            graph.addEdge(handles[i - 1], handles[i]);
        graph.addEdge(handles[29], handles[0]);
        bfs(graph, handles[0]);
        for (int i = 1; i < 30; ++i) {
            auto &data = graph.getNode(handles[i]);
            REQUIRE(data.color_ == Color::BLACK);
            REQUIRE(data.pred_ == handles[i - 1]);
            REQUIRE(data.dist_ == i);
        }
        auto &data = graph.getNode(handles[0]);
        REQUIRE(data.color_ == Color::BLACK);
        REQUIRE(data.pred_ == node_handle());
        REQUIRE(data.dist_ == 0);
    }

    SECTION("Test with random set of edges") {
        std::random_device rdev;
        std::random_device::result_type seed = rdev();
        syslog(LOG_INFO | LOG_USER,
               "BFSTEST -> Traversal tests on ListDiGraph, seed: %u", seed);
        std::default_random_engine eng(seed);
        std::uniform_int_distribution<int> dist(0, 29);
        for (int i = 0; i < 60; ++i)
            graph.addEdge(handles[dist(eng)], handles[dist(eng)]);
        for (int i = 1; i < 30; ++i)
            graph.addEdge(handles[0], handles[i]);
        bfs(graph, handles[0]);
        for (int i = 1; i < 30; ++i) {
            auto &data = graph.getNode(handles[i]);
            REQUIRE(data.color_ == Color::BLACK);
            node_handle nh = data.pred_;
            while (nh != handles[0]) {
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
    using node_handle =
        typename graph_traits<ListGraph<PathNodeData, EdgeData>>::node_handle;
    ListGraph<PathNodeData, EdgeData> graph;
    node_handle handles[30];
    for (auto &handle : handles)
        handle = graph.addNode();
    SECTION("Simple traversal test") {
        for (int i = 1; i < 30; ++i)
            graph.addEdge(handles[i - 1], handles[i]);
        graph.addEdge(handles[29], handles[0]);
        bfs(graph, handles[0]);
        int dist = 0;
        for (int i = 1; i < 30; ++i) {
            auto &data = graph.getNode(handles[i]);
            REQUIRE(data.color_ == Color::BLACK);
            if (i > 15 && i < 29) {
                REQUIRE(data.pred_ == handles[i + 1]);
                REQUIRE(data.dist_ == --dist);
            } else if (i == 29) {
                REQUIRE(data.pred_ == handles[0]);
                REQUIRE(data.dist_ == --dist);
            } else {
                REQUIRE(data.pred_ == handles[i - 1]);
                REQUIRE(data.dist_ == ++dist);
            }
        }
        auto &data = graph.getNode(handles[0]);
        REQUIRE(data.color_ == Color::BLACK);
        REQUIRE(data.pred_ == node_handle());
        REQUIRE(data.dist_ == 0);
    }
    SECTION("Test with random set of edges") {
        std::random_device rdev;
        std::random_device::result_type seed = rdev();
        syslog(LOG_INFO | LOG_USER,
               "DFSTEST -> Traversal test on ListGraph, seed: %u", seed);
        std::default_random_engine eng(seed);
        std::uniform_int_distribution<int> dist(0, 29);
        for (int i = 0; i < 60; ++i)
            graph.addEdge(handles[dist(eng)], handles[dist(eng)]);
        for (int i = 1; i < 30; ++i)
            graph.addEdge(handles[0], handles[i]);
        bfs(graph, handles[0]);
        for (int i = 1; i < 30; ++i) {
            auto &data = graph.getNode(handles[i]);
            REQUIRE(data.color_ == Color::BLACK);
            node_handle nh = data.pred_;
            while (nh != handles[0]) {
                auto &pred_data = graph.getNode(nh);
                nh = pred_data.pred_;
            }
        }
        auto &data = graph.getNode(handles[0]);
        REQUIRE(data.color_ == Color::BLACK);
        REQUIRE(data.pred_ == node_handle());
    }
}

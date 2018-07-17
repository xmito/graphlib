#define CATCH_CONFIG_MAIN
#include "FibonacciHeap.h"
#include "ListDiGraph.h"
#include "catch.hpp"
#include "graph_traits.h"

TEST_CASE("FibonacciHeap::FibonacciHeap(*)/operator=")
{
    using namespace graphlib;
    using Graph = ListDiGraph<PathNodeData, WeightedEdgeData>;
    using node_handle = typename graph_traits<Graph>::node_handle;
    Graph graph;
    LessDistance<Graph> comp(&graph);
    std::vector<node_handle> handles;
    for (int i = 0; i < 20; ++i)
    {
        if (i > 0)
            handles.push_back(graph.addNode(Color::BLACK, handles[i - 1], i));
        else
            handles.push_back(graph.addNode(Color::BLACK, node_handle(), 0));
    }
    SECTION("FibonacciHeap(const Compare&)")
    {
        FibonacciHeap<Graph> fh(comp);
        REQUIRE(fh.empty());
    }
    SECTION("FibonacciHeap(std::initializer_list<value_type>, const Compare&)")
    {
        FibonacciHeap<Graph> fh({handles[0], handles[1], handles[2], handles[3]}, comp);
        REQUIRE(fh.size() == 4);
        REQUIRE(fh.top() == handles[0]);
    }
    SECTION("FibonacciHeap(InputIt first, InputIt last, const Compare&)")
    {
        FibonacciHeap<Graph> fh(handles.begin(), handles.end(), comp);
        REQUIRE(fh.size() == 20);
        REQUIRE_FALSE(fh.empty());
        REQUIRE(fh.top() == handles[0]);
    }
    SECTION("FibonacciHeap(const Graph& graph)")
    {
        FibonacciHeap<Graph> fh(graph);
        REQUIRE(fh.size() == 20);
        REQUIRE_FALSE(fh.empty());
        REQUIRE(fh.top() == handles[0]);
    }
    SECTION("FibonacciHeap(const Graph& graph, const Compare &comp)")
    {
        FibonacciHeap<Graph> fh(graph, comp);
        REQUIRE(fh.size() == 20);
        REQUIRE_FALSE(fh.empty());
        REQUIRE(fh.top() == handles[0]);
    }
    SECTION("FibonacciHeap(const FibonacciHeap&)")
    {
        FibonacciHeap<Graph> fh(graph);
        FibonacciHeap<Graph> fh_cp(fh);
        REQUIRE(fh_cp.size() == 20);
        REQUIRE_FALSE(fh_cp.empty());
        REQUIRE(fh_cp.top() == handles[0]);
    }
    SECTION("operator=(const FibonacciHeap&)")
    {
        FibonacciHeap<Graph> fh(graph);
        FibonacciHeap<Graph> fh_cp(graph.endNode(), graph.endNode(), comp);
        fh_cp = fh;
        REQUIRE(fh_cp.size() == 20);
        REQUIRE_FALSE(fh_cp.empty());
        REQUIRE(fh_cp.top() == handles[0]);
    }
    SECTION("FibonacciHeap(FibonacciHeap&&)")
    {
        FibonacciHeap<Graph> fh(graph);
        FibonacciHeap<Graph> fh_mv(std::move(fh));
        REQUIRE(fh.empty());
        REQUIRE_FALSE(fh_mv.empty());
        REQUIRE(fh_mv.size() == 20);
    }
    SECTION("operator=(FibonacciHeap&&)")
    {
        FibonacciHeap<Graph> fh(graph);
        FibonacciHeap<Graph> fh_mv(comp);
        fh_mv = std::move(fh);
        REQUIRE(fh.empty());
        REQUIRE_FALSE(fh_mv.empty());
        REQUIRE(fh_mv.size() == 20);
    }
}

TEST_CASE("FibonacciHeap getters")
{
    using namespace graphlib;
    using Graph = ListDiGraph<PathNodeData, WeightedEdgeData>;
    using node_handle = typename graph_traits<Graph>::node_handle;
    Graph graph;
    LessDistance<Graph> comp(&graph);
    std::vector<node_handle> handles;
    for (int i = 0; i < 20; ++i)
    {
        if (i > 0)
            handles.push_back(graph.addNode(Color::BLACK, handles[i - 1], i));
        else
            handles.push_back(graph.addNode(Color::BLACK, node_handle(), 0));
    }
    FibonacciHeap<Graph> fh(graph);
    SECTION("FibonacciHeap::size()")
    {
        REQUIRE(fh.size() == 20);
    }
    SECTION("FibonacciHeap::empty()")
    {
        FibonacciHeap<Graph> fh_empty(graph.endNode(), graph.endNode(), comp);
        REQUIRE(fh_empty.empty());
        REQUIRE_FALSE(fh.empty());
    }
    SECTION("FibonacciHeap::top()")
    {
        REQUIRE(fh.top() == handles[0]);
    }
}

TEST_CASE("FibonacciHeap modifiers")
{
    using namespace graphlib;
    using Graph = ListDiGraph<PathNodeData, WeightedEdgeData>;
    using node_handle = typename graph_traits<Graph>::node_handle;
    Graph graph;
    LessDistance<Graph> comp(&graph);
    std::vector<node_handle> handles;
    for (int i = 0; i < 20; ++i)
    {
        if (i > 0)
            handles.push_back(graph.addNode(Color::BLACK, handles[i - 1], i + 20));
        else
            handles.push_back(graph.addNode(Color::BLACK, node_handle(), 20));
    }
    SECTION("FibonacciHeap::push(const value_type&)")
    {
        FibonacciHeap<Graph> fh(graph.endNode(), graph.endNode(), comp);
        for (int i = 19; i >= 0; --i)
        {
            fh.push(handles[i]);
            REQUIRE(fh.top() == handles[i]);
        }
    }

    SECTION("FibonacciHeap::pop()")
    {
        FibonacciHeap<Graph> fh(graph);
        for (int i = 0; i < 20; ++i)
        {
            REQUIRE(fh.top() == handles[i]);
            fh.pop();
        }
    }
    SECTION("FibonacciHeap::decUpdate()")
    {
        FibonacciHeap<Graph> fh(graph);
        for (int i = 1; i <= 20; ++i)
        {
            auto &data = graph.getNode(handles[20 - i]);
            data.dist_ = 20 - i;
            fh.decUpdate(handles[20 - i]);
            REQUIRE(fh.top() == handles[20 - i]);
        }
    }
}

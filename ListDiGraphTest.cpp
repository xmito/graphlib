#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "ListDiGraph.h"

TEST_CASE("ListDiGraph::ListDiGraph") {
	using namespace graphlib;
	SECTION("ListDiGraph::ListDiGraph()") {
		ListDiGraph<DefaultNodeData, EdgeData> gr_one;
		ListDiGraph<TraversableNodeData, EdgeData> gr_two;
		ListDiGraph<PathNodeData, EdgeData> gr_three;
		ListDiGraph<LocationNodeData<int>, EdgeData> gr_four;

		ListDiGraph<DefaultNodeData, WeightedEdgeData> gr_five;
		ListDiGraph<TraversableNodeData, WeightedEdgeData> gr_six;
		ListDiGraph<PathNodeData, WeightedEdgeData> gr_seven;
		ListDiGraph<LocationNodeData<int>, WeightedEdgeData> gr_eight;
	}
	SECTION("ListDiGraph::ListDiGraph(size_t)") {
		ListDiGraph<DefaultNodeData, EdgeData> gr_one(30);
		ListDiGraph<TraversableNodeData, EdgeData> gr_two(30);
		ListDiGraph<PathNodeData, EdgeData> gr_three(30);
		ListDiGraph<LocationNodeData<int>, EdgeData> gr_four(30);

		ListDiGraph<DefaultNodeData, WeightedEdgeData> gr_five(30);
		ListDiGraph<TraversableNodeData, WeightedEdgeData> gr_six(30);
		ListDiGraph<PathNodeData, WeightedEdgeData> gr_seven(30);
		ListDiGraph<LocationNodeData<int>, WeightedEdgeData> gr_eight(30);
	}
}

TEST_CASE("ListDiGraph::addNode") {
	using namespace graphlib;
	SECTION("DefaultNodeData") {
		ListDiGraph<DefaultNodeData, EdgeData> graph;
		auto handle = graph.addNode();
		REQUIRE(graph.hasNode(handle));
		auto begin = graph[handle].begin();
		auto end = graph[handle].end();
		REQUIRE(begin == end);
		REQUIRE(graph.inNodeDegree(handle) == 0);
		REQUIRE(graph.outNodeDegree(handle) == 0);
	}
	SECTION("TraversableNodeData") {
		ListDiGraph<TraversableNodeData, EdgeData> graph;
		auto handle_one = graph.addNode();
		REQUIRE(graph.hasNode(handle_one));
		auto begin_one = graph[handle_one].begin();
		auto end_one = graph[handle_one].end();
		REQUIRE(begin_one == end_one);
		REQUIRE(graph.inNodeDegree(handle_one) == 0);
		REQUIRE(graph.outNodeDegree(handle_one) == 0);
		TraversableNodeData& data_one = graph.getNode(handle_one);
		REQUIRE(data_one.color_ == Color::WHITE);
		REQUIRE(data_one.pred_ == static_cast<size_t>(-1));

		auto handle_two = graph.addNode(Color::GRAY, 2);
		REQUIRE(graph.hasNode(handle_two));
		auto begin_two = graph[handle_two].begin();
		auto end_two = graph[handle_two].end();
		REQUIRE(begin_two == end_two);
		REQUIRE(graph.inNodeDegree(handle_two) == 0);
		REQUIRE(graph.outNodeDegree(handle_two) == 0);
		TraversableNodeData& data_two = graph.getNode(handle_two);
		REQUIRE(data_two.color_ == Color::GRAY);
		REQUIRE(data_two.pred_ == 2);
	}
	SECTION("PathNodeData") {
		ListDiGraph<PathNodeData, EdgeData> graph;
		auto handle_one = graph.addNode();
		REQUIRE(graph.hasNode(handle_one));
		auto begin_one = graph[handle_one].begin();
		auto end_one = graph[handle_one].end();
		REQUIRE(begin_one == end_one);
		REQUIRE(graph.inNodeDegree(handle_one) == 0);
		REQUIRE(graph.outNodeDegree(handle_one) == 0);
		PathNodeData& data_one = graph.getNode(handle_one);
		REQUIRE(data_one.color_ == Color::WHITE);
		REQUIRE(data_one.pred_ == static_cast<size_t>(-1));
		REQUIRE(data_one.dist_ == 0);

		auto handle_two = graph.addNode(Color::GRAY, 0, 2);
		REQUIRE(graph.hasNode(handle_two));
		auto begin_two = graph[handle_two].begin();
		auto end_two = graph[handle_two].end();
		REQUIRE(begin_two == end_two);
		REQUIRE(graph.inNodeDegree(handle_two) == 0);
		REQUIRE(graph.outNodeDegree(handle_two) == 0);
		PathNodeData& data_two = graph.getNode(handle_two);
		REQUIRE(data_two.color_ == Color::GRAY);
		REQUIRE(data_two.pred_ == 0);
		REQUIRE(data_two.dist_ == 2);
	}
	SECTION("LocationNodeData<Data>") {
		struct Data {
			int first_;
			int second_;
			Data() : first_(0), second_(0) {}
			Data(int first, int second) : first_(first), second_(second) {}
		};
		ListDiGraph<LocationNodeData<Data>, EdgeData> graph;
		auto handle_one = graph.addNode();
		REQUIRE(graph.hasNode(handle_one));
		auto begin_one = graph[handle_one].begin();
		auto end_one = graph[handle_one].end();
		REQUIRE(begin_one == end_one);
		REQUIRE(graph.inNodeDegree(handle_one) == 0);
		REQUIRE(graph.outNodeDegree(handle_one) == 0);
		LocationNodeData<Data>& data_one = graph.getNode(handle_one);
		REQUIRE(data_one.color_ == Color::WHITE);
		REQUIRE(data_one.dist_ == 0);
		REQUIRE(data_one.loc_.first_ == 0);
		REQUIRE(data_one.loc_.second_ == 0);
		REQUIRE(data_one.pred_ == static_cast<size_t>(-1));

		auto handle_two = graph.addNode(10, 11);
		REQUIRE(graph.hasNode(handle_two));
		auto begin_two = graph[handle_two].begin();
		auto end_two = graph[handle_two].end();
		REQUIRE(begin_two == end_two);
		REQUIRE(graph.inNodeDegree(handle_two) == 0);
		REQUIRE(graph.outNodeDegree(handle_two) == 0);
		LocationNodeData<Data>& data_two = graph.getNode(handle_two);
		REQUIRE(data_two.color_ == Color::WHITE);
		REQUIRE(data_two.dist_ == 0);
		REQUIRE(data_two.pred_ == static_cast<size_t>(-1));
		REQUIRE(data_two.loc_.first_ == 10);
		REQUIRE(data_two.loc_.second_ == 11);
	}
}

TEST_CASE("ListDiGraph::addEdge") {
	using namespace graphlib;
	SECTION("ListDiGraph::addEdge() without weight") {
		ListDiGraph<TraversableNodeData, EdgeData> graph;
		ListDiGraph<TraversableNodeData, EdgeData>::node_handle node_handles[20];
		ListDiGraph<TraversableNodeData, EdgeData>::edge_handle edge_handles[19];
		for (int i = 0; i < 20; ++i) {
			node_handles[i] = graph.addNode();
			if (i > 0) {
				edge_handles[i - 1] = graph.addEdge(node_handles[i - 1], node_handles[i]);
				REQUIRE(graph.inNodeDegree(node_handles[i]) == 1);
				REQUIRE(graph.outNodeDegree(node_handles[i - 1]) == 1);
				REQUIRE(graph.hasEdge(edge_handles[i - 1]));
				REQUIRE(graph.getSource(edge_handles[i - 1]) == node_handles[i - 1]);
				REQUIRE(graph.getTarget(edge_handles[i - 1]) == node_handles[i]);
			}
		}
	}
	SECTION("ListDiGraph::addEdge() with weight") {
		ListDiGraph<TraversableNodeData, WeightedEdgeData> graph;
		ListDiGraph<TraversableNodeData, WeightedEdgeData>::node_handle node_handles[20];
		ListDiGraph<TraversableNodeData, WeightedEdgeData>::edge_handle edge_handles[19];
		for (int i = 0; i < 20; ++i) {
			node_handles[i] = graph.addNode();
			if (i > 0) {
				edge_handles[i - 1] = graph.addEdge(node_handles[i - 1], node_handles[i], 2);
				REQUIRE(graph.inNodeDegree(node_handles[i]) == 1);
				REQUIRE(graph.outNodeDegree(node_handles[i - 1]) == 1);
				REQUIRE(graph.hasEdge(edge_handles[i - 1]));
				REQUIRE(graph.getSource(edge_handles[i - 1]) == node_handles[i - 1]);
				REQUIRE(graph.getTarget(edge_handles[i - 1]) == node_handles[i]);
				auto &edata = graph.getEdge(edge_handles[i - 1]);
				REQUIRE(edata.weight_ == 2);
			}
		}
	}
}

TEST_CASE("ListDiGraph::removeNode(const NodeHandle&)") {
	using namespace graphlib;
	ListDiGraph<TraversableNodeData, EdgeData> graph;
	ListDiGraph<TraversableNodeData, EdgeData>::node_handle node_handles[4];
	for (int i = 0; i < 4; ++i) {
		node_handles[i] = graph.addNode();
		for (int j = i - 1; j >= 0; --j)
			graph.addEdge(node_handles[i], node_handles[j]);
	}

	graph.removeNode(node_handles[2]);
	REQUIRE(graph.nodeCount() == 3);
	auto& list = graph[node_handles[3]];
	auto bit = list.begin();
	REQUIRE(graph.inNodeDegree(graph.getTarget(*bit)) == 1);
	++bit;
	REQUIRE(graph.inNodeDegree(graph.getTarget(*bit)) == 2);
	REQUIRE(list.size() == 2);
	REQUIRE(graph.hasNode(node_handles[2]) == false);

	graph.removeNode(node_handles[1]);
	// Note that removed node is still there...
	REQUIRE(graph.inNodeDegree(node_handles[1]) == 1);
	REQUIRE(graph.outNodeDegree(node_handles[1]) == 0);
	// Test that inNodeDegree of first node dropped by one
	REQUIRE(graph.inNodeDegree(node_handles[0]) == 1);
	/* outNodeDegree returns 1 instead of 2, because it returns number
	 * of outgoing edges, that point to valid nodes. If outNodeDegree
	 * is called, it goes over all outgoing edges, removing those that
	 * point to invalid ones. Its complexity can reach at most O(V^2) */
	REQUIRE(graph.outNodeDegree(node_handles[3]) == 1);
}

TEST_CASE("ListDiGraph::removeEdge(const NodeHandle&)") {
	using namespace graphlib;
	ListDiGraph<TraversableNodeData, EdgeData> graph;
	ListDiGraph<TraversableNodeData, EdgeData>::node_handle node_handles[4];
	ListDiGraph<TraversableNodeData, EdgeData>::edge_handle edge_handles[6];
	int counter = 0;
	for (int i = 0; i < 4; ++i) {
		node_handles[i] = graph.addNode();
		for (int j = i - 1; j >= 0; --j)
			edge_handles[counter++] = graph.addEdge(node_handles[i], node_handles[j]);
	}
	graph.removeEdge(edge_handles[3]);
	REQUIRE(graph.outNodeDegree(node_handles[3]) == 2);
	REQUIRE(graph.inNodeDegree(node_handles[2]) == 0);

	graph.removeEdge(edge_handles[2]);
	REQUIRE(graph.inNodeDegree(node_handles[0]) == 2);
	REQUIRE(graph.outNodeDegree(node_handles[2]) == 1);

	graph.removeEdge(edge_handles[5]);
	REQUIRE(graph.inNodeDegree(node_handles[0]) == 1);
	REQUIRE(graph.outNodeDegree(node_handles[3]) == 1);
}

TEST_CASE("ListDiGraph lookup methods") {
	using namespace graphlib;
	ListDiGraph<TraversableNodeData, WeightedEdgeData> graph;
	ListDiGraph<TraversableNodeData, WeightedEdgeData>::node_handle node_handles[4];
	ListDiGraph<TraversableNodeData, WeightedEdgeData>::edge_handle edge_handles[6];
	int counter = 0;
	for (int i = 0; i < 4; ++i) {
		node_handles[i] = graph.addNode();
		for (int j = i - 1; j >= 0; --j)
			edge_handles[counter++] = graph.addEdge(node_handles[i], node_handles[j]);
	}
	SECTION("ListDiGraph::hasEdge()") {
		REQUIRE(graph.hasEdge(edge_handles[3]));
		REQUIRE(graph.hasEdge(edge_handles[5]));
		graph.removeEdge(edge_handles[3]);
		REQUIRE(graph.hasEdge(edge_handles[3]) == false);
		graph.removeEdge(edge_handles[0]);
		REQUIRE(graph.hasEdge(edge_handles[0]) == false);
	}
	SECTION("ListDiGraph::hasNode()") {
		for (int i = 0; i < 4; ++i)
			REQUIRE(graph.hasNode(node_handles[i]));
		graph.removeNode(node_handles[0]);
		REQUIRE(graph.hasNode(node_handles[0]) == false);
		graph.removeNode(node_handles[3]);
		REQUIRE(graph.hasNode(node_handles[3]) == 0);
	}
	SECTION("ListDiGraph::getEdge()") {
		auto &edata = graph.getEdge(edge_handles[0]);
		REQUIRE(edata.weight_ == 1);
		auto cedata = const_cast<const ListDiGraph<TraversableNodeData, WeightedEdgeData>&>(graph).getEdge(edge_handles[0]);
		REQUIRE(cedata.weight_ == 1);
	}
	SECTION("ListDiGraph::getNode()") {
		auto &ndata = graph.getNode(node_handles[3]);
		REQUIRE(ndata.color_ == Color::WHITE);
		REQUIRE(ndata.pred_ == static_cast<size_t>(-1));
		REQUIRE(graph.outNodeDegree(node_handles[3]) == 3);
		auto &cndata = const_cast<ListDiGraph<TraversableNodeData, WeightedEdgeData>&>(graph).getNode(node_handles[3]);
		REQUIRE(cndata.color_ == Color::WHITE);
		REQUIRE(cndata.pred_ == static_cast<size_t>(-1));
		REQUIRE(graph.outNodeDegree(node_handles[3]) == 3);
	}
	SECTION("ListDiGraph::getSource/getTarget()") {
		auto src_nh = graph.getSource(edge_handles[5]);
		REQUIRE(graph.inNodeDegree(src_nh) == 0);
		REQUIRE(graph.outNodeDegree(src_nh) == 3);
		auto tg_nh = graph.getTarget(edge_handles[5]);
		REQUIRE(graph.inNodeDegree(tg_nh) == 3);
		REQUIRE(graph.outNodeDegree(tg_nh) == 0);
	}
	SECTION("ListDiGraph::getSourceNode/getTargetNode()") {
		auto &src_node = graph.getSourceNode(edge_handles[5]);
		REQUIRE(src_node.color_ == Color::WHITE);
		REQUIRE(src_node.pred_ == static_cast<size_t>(-1));
		auto &csrc_node = const_cast<ListDiGraph<TraversableNodeData, WeightedEdgeData>&>(graph).getSourceNode(edge_handles[5]);

		auto &tg_node = graph.getTargetNode(edge_handles[5]);
		REQUIRE(tg_node.color_ == Color::WHITE);
		REQUIRE(tg_node.pred_ == static_cast<size_t>(-1));
		auto &ctg_node = const_cast<ListDiGraph<TraversableNodeData, WeightedEdgeData>&>(graph).getTargetNode(edge_handles[5]);
	}
	SECTION("ListDiGraph::operator[]()") {
		auto &list = graph[node_handles[3]];
		REQUIRE(list.size() == 3);
		int counter = 1;
		for (auto eh : list)
			REQUIRE(graph.inNodeDegree(graph.getTarget(eh)) == counter++);
		auto &clist = const_cast<const ListDiGraph<TraversableNodeData, WeightedEdgeData>&>(graph)[node_handles[3]];
	}
	SECTION("ListDiGraph::nodeCount()") {
		REQUIRE(graph.nodeCount() == 4);
		graph.removeNode(node_handles[3]);
		REQUIRE(graph.nodeCount() == 3);
		graph.addNode();
		REQUIRE(graph.nodeCount() == 4);
	}
	SECTION("ListDiGraph::edgeCount()") {
		REQUIRE(graph.edgeCount() == 6);
		graph.removeEdge(edge_handles[5]);
		REQUIRE(graph.edgeCount() == 5);
		graph.addEdge(node_handles[3], node_handles[0]);
		REQUIRE(graph.edgeCount() == 6);
	}
	SECTION("ListDiGraph::beginNode/endNode/cbeginNode/cendNode") {
		auto nbit = graph.beginNode();
		auto neit = graph.endNode();
		while (nbit != neit) {
			auto &nodedata = graph.getNode(*nbit);
			REQUIRE(nodedata.color_ == Color::WHITE);
			REQUIRE(nodedata.pred_ == static_cast<size_t>(-1));
			++nbit;
		}
		auto cnbit = const_cast<ListDiGraph<TraversableNodeData, WeightedEdgeData>&>(graph).beginNode();
		auto cneit = const_cast<ListDiGraph<TraversableNodeData, WeightedEdgeData>&>(graph).endNode();

		auto ccnbit = graph.cbeginNode();
		auto ccneit = graph.cendNode();
	}
	SECTION("ListDiGraph::beginEdge/endEdge/cbeginEdge/cendEdge") {
		auto ebit = graph.beginEdge();
		auto eeit = graph.endEdge();
		while (ebit != eeit) {
			auto &edgedata = graph.getEdge(*ebit);
			REQUIRE(edgedata.weight_ == 1);
			++ebit;
		}
		auto cebit = const_cast<ListDiGraph<TraversableNodeData, WeightedEdgeData>&>(graph).beginEdge();
		auto ceeit = const_cast<ListDiGraph<TraversableNodeData, WeightedEdgeData>&>(graph).endEdge();

		auto ccebit = graph.cbeginEdge();
		auto cceeit = graph.cendEdge();
	}
	SECTION("ListDiGraph::getWeight(const EdgeHandle&)") {
		auto weight = graph.getWeight(edge_handles[0]);
		REQUIRE(weight == 1);
	}
}

TEST_CASE("ListDiGraph::setWeight(const EdgeHandle&, weight_type)") {
	using namespace graphlib;
	ListDiGraph<TraversableNodeData, WeightedEdgeData> graph;

}

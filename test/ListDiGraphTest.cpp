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
		REQUIRE(data_one.pred_ == NodeHandle());

		auto handle_two = graph.addNode(Color::GRAY, handle_one);
		REQUIRE(graph.hasNode(handle_two));
		auto begin_two = graph[handle_two].begin();
		auto end_two = graph[handle_two].end();
		REQUIRE(begin_two == end_two);
		REQUIRE(graph.inNodeDegree(handle_two) == 0);
		REQUIRE(graph.outNodeDegree(handle_two) == 0);
		TraversableNodeData& data_two = graph.getNode(handle_two);
		REQUIRE(data_two.color_ == Color::GRAY);
		REQUIRE(data_two.pred_ == handle_one);
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
		REQUIRE(data_one.pred_ == NodeHandle());
		REQUIRE(data_one.dist_ == 0);

		auto handle_two = graph.addNode(Color::GRAY, handle_one, 2);
		REQUIRE(graph.hasNode(handle_two));
		auto begin_two = graph[handle_two].begin();
		auto end_two = graph[handle_two].end();
		REQUIRE(begin_two == end_two);
		REQUIRE(graph.inNodeDegree(handle_two) == 0);
		REQUIRE(graph.outNodeDegree(handle_two) == 0);
		PathNodeData& data_two = graph.getNode(handle_two);
		REQUIRE(data_two.color_ == Color::GRAY);
		REQUIRE(data_two.pred_ == handle_one);
		REQUIRE(data_two.dist_ == 2);
	}
	SECTION("LocationNodeData<Data>") {
		struct Data {
			int first_{0};
			int second_{0};
			Data() = default;
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
		REQUIRE(data_one.pred_ == NodeHandle());

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
		REQUIRE(data_two.pred_ == NodeHandle());
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

	graph.removeNode(node_handles[1]);
	// Note that removed node is still there...
	REQUIRE(graph.inNodeDegree(node_handles[1]) == 1);
	REQUIRE(graph.outNodeDegree(node_handles[1]) == 0);
	// Test that inNodeDegree of first node dropped by one
	REQUIRE(graph.inNodeDegree(node_handles[0]) == 1);
	/* outNodeDegree returns 1 instead of 2, because it returns number
	 * of outgoing edges, that point to valid nodes. */
	REQUIRE(graph.outNodeDegree(node_handles[3]) == 1);
}

TEST_CASE("ListDiGraph::removeEdge(const EdgeHandle&)") {
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
	using Graph = ListDiGraph<LocationNodeData<PlaneLocation>, WeightedEdgeData>;
	using node_handle = typename graph_traits<Graph>::node_handle;
	using edge_handle = typename graph_traits<Graph>::edge_handle;
	using location_type = typename graph_traits<Graph>::location_type;
	Graph graph;
	node_handle node_handles[4];
	edge_handle edge_handles[6];
	int counter = 0;
	for (int i = 0; i < 4; ++i) {
		node_handles[i] = graph.addNode();
		for (int j = i - 1; j >= 0; --j)
			edge_handles[counter++] = graph.addEdge(node_handles[i], node_handles[j]);
	}
	SECTION("ListDiGraph::hasEdge(const edge_handle &)") {
		for (auto &eh : edge_handles)
			REQUIRE(graph.hasEdge(eh));
	}
	SECTION("ListDiGraph::hasEdge(const node_handle &, const node_handle &)") {
		REQUIRE(graph.hasEdge(node_handles[1], node_handles[0]));
		REQUIRE(graph.hasEdge(node_handles[3], node_handles[2]));
		REQUIRE_FALSE(graph.hasEdge(node_handles[0], node_handles[1]));
		REQUIRE_FALSE(graph.hasEdge(node_handles[0], node_handles[0]));
	}
	SECTION("ListDiGraph::hasNode(const node_handle &)") {
		for (auto &nh : node_handles)
			REQUIRE(graph.hasNode(nh));
	}
	SECTION("ListDiGraph::getEdge(const edge_handle &)") {
		auto &edata = graph.getEdge(edge_handles[0]);
		REQUIRE(edata.weight_ == 1);
		auto cedata = const_cast<const Graph&>(graph).getEdge(edge_handles[0]);
		REQUIRE(cedata.weight_ == 1);
	}
	SECTION("ListDiGraph::getNode(const node_handle &)") {
		auto &ndata = graph.getNode(node_handles[3]);
		REQUIRE(ndata.color_ == Color::WHITE);
		REQUIRE(ndata.pred_ == NodeHandle());
		REQUIRE(graph.outNodeDegree(node_handles[3]) == 3);
		auto &cndata = const_cast<const Graph&>(graph).getNode(node_handles[3]);
		REQUIRE(cndata.color_ == Color::WHITE);
		REQUIRE(cndata.pred_ == NodeHandle());
		REQUIRE(graph.outNodeDegree(node_handles[3]) == 3);
	}
	SECTION("ListDiGraph::getSource/getTarget(const edge_handle &)") {
		auto src_nh = graph.getSource(edge_handles[5]);
		REQUIRE(graph.inNodeDegree(src_nh) == 0);
		REQUIRE(graph.outNodeDegree(src_nh) == 3);
		auto tg_nh = graph.getTarget(edge_handles[5]);
		REQUIRE(graph.inNodeDegree(tg_nh) == 3);
		REQUIRE(graph.outNodeDegree(tg_nh) == 0);
	}
	SECTION("ListDiGraph::getSourceNode/getTargetNode(const edge_handle &)") {
		auto &src_node = graph.getSourceNode(edge_handles[5]);
		REQUIRE(src_node.color_ == Color::WHITE);
		REQUIRE(src_node.pred_ == NodeHandle());
		auto &csrc_node = const_cast<const Graph&>(graph).getSourceNode(edge_handles[5]);

		auto &tg_node = graph.getTargetNode(edge_handles[5]);
		REQUIRE(tg_node.color_ == Color::WHITE);
		REQUIRE(tg_node.pred_ == NodeHandle());
		auto &ctg_node = const_cast<const Graph&>(graph).getTargetNode(edge_handles[5]);
	}
	SECTION("ListDiGraph::operator[](const node_handle &)") {
		auto &list = graph[node_handles[3]];
		REQUIRE(list.size() == 3);
		int counter = 1;
		for (auto eh : list)
			REQUIRE(graph.inNodeDegree(graph.getTarget(eh)) == counter++);
		auto &clist = const_cast<const Graph&>(graph)[node_handles[3]];
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
			REQUIRE(nodedata.pred_ == NodeHandle());
			++nbit;
		}
		auto cnbit = const_cast<const Graph&>(graph).beginNode();
		auto cneit = const_cast<const Graph&>(graph).endNode();

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
		auto cebit = const_cast<const Graph&>(graph).beginEdge();
		auto ceeit = const_cast<const Graph&>(graph).endEdge();

		auto ccebit = graph.cbeginEdge();
		auto cceeit = graph.cendEdge();
	}
	SECTION("ListDiGraph::nodes()") {
		int count = 0;
		for (auto &nh : graph.nodes()) {
			auto &data = graph.getNode(nh);
			REQUIRE(data.color_ == Color::WHITE);
			REQUIRE(data.pred_ == node_handle());
			++count;
		}
		REQUIRE(count == 4);
	}
	SECTION("ListDiGraph::edges()") {
		int count = 0;
		for (const auto &eh : graph.edges()) {
			auto &data = graph.getEdge(eh);
			REQUIRE(data.weight_ == 1);
			++count;
		}
		REQUIRE(count == 6);
	}
	SECTION("ListDiGraph::getWeight(const EdgeHandle&)") {
		auto weight = graph.getWeight(edge_handles[0]);
		REQUIRE(weight == 1);
	}
	SECTION("ListDiGraph::getNodeColor(const node_handle &)") {
		REQUIRE(graph.getNodeColor(node_handles[0]) == Color::WHITE);
		auto &data = graph.getNode(node_handles[0]);
		data.color_ = Color::BLACK;
		REQUIRE(graph.getNodeColor(node_handles[0]) == Color::BLACK);
	}
	SECTION("ListDiGraph::getNodePred(const node_handle &)") {
		REQUIRE(graph.getNodePred(node_handles[0]) == node_handle());
		auto &data = graph.getNode(node_handles[0]);
		data.pred_ = node_handles[1];
		REQUIRE(graph.getNodePred(node_handles[0]) == node_handles[1]);
	}
	SECTION("ListDiGraph::getNodeDist(const node_handle &)") {
		REQUIRE(graph.getNodeDist(node_handles[0]) == 0);
		auto &data = graph.getNode(node_handles[0]);
		data.dist_ = 1;
		REQUIRE(graph.getNodeDist(node_handles[0]) == 1);
	}
	SECTION("ListDiGraph::getNodeLoc(const node_handle &)") {
		REQUIRE(graph.getNodeLoc(node_handles[0]) == location_type(0, 0));
		auto &data = graph.getNode(node_handles[0]);
		data.loc_ = location_type(1, 1);
		REQUIRE(graph.getNodeLoc(node_handles[0]) == location_type(1, 1));
	}
	SECTION("ListDiGraph::getNodePrio(const node_handle &)") {
		REQUIRE(graph.getNodePrio(node_handles[0]) == 0);
		auto &data = graph.getNode(node_handles[0]);
		data.prio_ = 1;
		REQUIRE(graph.getNodePrio(node_handles[0]) == 1);
	}
}


TEST_CASE("ListDiGraph modifiers") {
	using namespace graphlib;
	using Graph = ListDiGraph<LocationNodeData<PlaneLocation>, WeightedEdgeData>;
	using node_handle = typename graph_traits<Graph>::node_handle;
	using edge_handle = typename graph_traits<Graph>::edge_handle;
	using location_type = typename graph_traits<Graph>::location_type;
	Graph graph;
	node_handle node_handles[4];
	edge_handle edge_handles[6];
	int counter = 0;
	for (int i = 0; i < 4; ++i) {
		node_handles[i] = graph.addNode();
		for (int j = i - 1; j >= 0; --j)
			edge_handles[counter++] = graph.addEdge(node_handles[i], node_handles[j]);
	}
	SECTION("ListDiGraph::setWeight(const edge_handle &, weight_type)") {
		auto &edata = graph.getEdge(edge_handles[0]);
		REQUIRE(edata.weight_ == 1);
		graph.setWeight(edge_handles[0], 2);
		REQUIRE(edata.weight_ == 2);
	}
	SECTION("ListDiGraph::modWeight(const EdgeHandle&, weight_type)") {
		auto cweight = graph.getWeight(edge_handles[0]);
		graph.modWeight(edge_handles[0], 3);
		REQUIRE(graph.getWeight(edge_handles[0]) == cweight + 3);
	}
	SECTION("ListDiGraph::setNodeColor(const node_handle &, Color)") {
		graph.setNodeColor(node_handles[0], Color::BLACK);
		REQUIRE(graph.getNodeColor(node_handles[0]) == Color::BLACK);
	}
	SECTION("ListDiGraph::setNodeDist(const node_handle &, distance_type)") {
		graph.setNodeDist(node_handles[0], 1);
		REQUIRE(graph.getNodeDist(node_handles[0]) == 1);
	}
	SECTION("ListDiGraph::setNodePred(const node_handle &, const node_handle&)") {
		graph.setNodePred(node_handles[0], node_handles[1]);
		REQUIRE(graph.getNodePred(node_handles[0]) == node_handles[1]);
	}
	SECTION("ListDiGraph::setNodeLoc(const node_handle &, const location_type &)") {
		graph.setNodeLoc(node_handles[0], location_type(1, 1));
		REQUIRE(graph.getNodeLoc(node_handles[0]) == location_type(1, 1));
	}
	SECTION("ListDiGraph::setNodePrio(const node_handle &, priority_type)") {
		graph.setNodePrio(node_handles[0], 2);
		REQUIRE(graph.getNodePrio(node_handles[0]) == 2);
	}
}

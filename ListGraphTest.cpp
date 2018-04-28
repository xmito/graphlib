#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "ListGraph.h"

TEST_CASE("ListGraph::ListGraph") {
	using namespace graphlib;
	SECTION("ListGraph::ListGraph()") {
		ListGraph<DefaultNodeData, EdgeData> gr_one;
		ListGraph<TraversableNodeData, EdgeData> gr_two;
		ListGraph<PathNodeData, EdgeData> gr_three;
		ListGraph<LocationNodeData<int>, EdgeData> gr_four;

		ListGraph<DefaultNodeData, WeightedEdgeData> gr_five;
		ListGraph<TraversableNodeData, WeightedEdgeData> gr_six;
		ListGraph<PathNodeData, WeightedEdgeData> gr_seven;
		ListGraph<LocationNodeData<int>, WeightedEdgeData> gr_eight;
	}
	SECTION("ListGraph::ListGraph(size_t)") {
		ListGraph<DefaultNodeData, EdgeData> gr_one(30);
		ListGraph<TraversableNodeData, EdgeData> gr_two(30);
		ListGraph<PathNodeData, EdgeData> gr_three(30);
		ListGraph<LocationNodeData<int>, EdgeData> gr_four(30);

		ListGraph<DefaultNodeData, WeightedEdgeData> gr_five(30);
		ListGraph<TraversableNodeData, WeightedEdgeData> gr_six(30);
		ListGraph<PathNodeData, WeightedEdgeData> gr_seven(30);
		ListGraph<LocationNodeData<int>, WeightedEdgeData> gr_eight(30);
	}
}

TEST_CASE("ListGraph::addNode") {
	using namespace graphlib;
	SECTION("DefaultNodeData") {
		ListGraph<DefaultNodeData, EdgeData> graph;
		auto handle = graph.addNode();
		REQUIRE(graph.hasNode(handle));
		auto begin = graph[handle].begin();
		auto end = graph[handle].end();
		REQUIRE(begin == end);
		REQUIRE(graph.degree(handle) == 0);
	}
	SECTION("TraversableNodeData") {
		ListGraph<TraversableNodeData, EdgeData> graph;
		auto handle_one = graph.addNode();
		REQUIRE(graph.hasNode(handle_one));
		auto begin_one = graph[handle_one].begin();
		auto end_one = graph[handle_one].end();
		REQUIRE(begin_one == end_one);
		REQUIRE(graph.degree(handle_one) == 0);
		TraversableNodeData& data_one = graph.getNode(handle_one);
		REQUIRE(data_one.color_ == Color::WHITE);
		REQUIRE(data_one.pred_ == NodeHandle());

		auto handle_two = graph.addNode(Color::GRAY, handle_one);
		REQUIRE(graph.hasNode(handle_two));
		auto begin_two = graph[handle_two].begin();
		auto end_two = graph[handle_two].end();
		REQUIRE(begin_two == end_two);
		REQUIRE(graph.degree(handle_two) == 0);
		TraversableNodeData& data_two = graph.getNode(handle_two);
		REQUIRE(data_two.color_ == Color::GRAY);
		REQUIRE(data_two.pred_ == handle_one);
	}
	SECTION("PathNodeData") {
		ListGraph<PathNodeData, EdgeData> graph;
		auto handle_one = graph.addNode();
		REQUIRE(graph.hasNode(handle_one));
		auto begin_one = graph[handle_one].begin();
		auto end_one = graph[handle_one].end();
		REQUIRE(begin_one == end_one);
		REQUIRE(graph.degree(handle_one) == 0);
		PathNodeData& data_one = graph.getNode(handle_one);
		REQUIRE(data_one.color_ == Color::WHITE);
		REQUIRE(data_one.pred_ == NodeHandle());
		REQUIRE(data_one.dist_ == 0);

		auto handle_two = graph.addNode(Color::GRAY, handle_one, 2);
		REQUIRE(graph.hasNode(handle_two));
		auto begin_two = graph[handle_two].begin();
		auto end_two = graph[handle_two].end();
		REQUIRE(begin_two == end_two);
		REQUIRE(graph.degree(handle_two) == 0);
		PathNodeData& data_two = graph.getNode(handle_two);
		REQUIRE(data_two.color_ == Color::GRAY);
		REQUIRE(data_two.pred_ == handle_one);
		REQUIRE(data_two.dist_ == 2);
	}
	SECTION("LocationNodeData<Data>") {
		struct Data {
			int first_;
			int second_;
			Data() : first_(0), second_(0) {}
			Data(int first, int second) : first_(first), second_(second) {}
		};
		ListGraph<LocationNodeData<Data>, EdgeData> graph;
		auto handle_one = graph.addNode();
		REQUIRE(graph.hasNode(handle_one));
		auto begin_one = graph[handle_one].begin();
		auto end_one = graph[handle_one].end();
		REQUIRE(begin_one == end_one);
		REQUIRE(graph.degree(handle_one) == 0);
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
		REQUIRE(graph.degree(handle_two) == 0);
		LocationNodeData<Data>& data_two = graph.getNode(handle_two);
		REQUIRE(data_two.color_ == Color::WHITE);
		REQUIRE(data_two.dist_ == 0);
		REQUIRE(data_two.pred_ == NodeHandle());
		REQUIRE(data_two.loc_.first_ == 10);
		REQUIRE(data_two.loc_.second_ == 11);
	}
}

TEST_CASE("ListGraph::addEdge") {
	using namespace graphlib;
	SECTION("ListGraph::addEdge() without weight") {
		ListGraph<TraversableNodeData, EdgeData> graph;
		ListGraph<TraversableNodeData, EdgeData>::node_handle node_handles[20];
		ListGraph<TraversableNodeData, EdgeData>::edge_handle edge_handles[19];
		for (int i = 0; i < 20; ++i) {
			node_handles[i] = graph.addNode();
			if (i > 0) {
				edge_handles[i - 1] = graph.addEdge(node_handles[i - 1], node_handles[i]);
				REQUIRE(graph.degree(node_handles[i]) == 1);
				if (i - 1 == 0)
					REQUIRE(graph.degree(node_handles[i - 1]) == 1);
				else
					REQUIRE(graph.degree(node_handles[i - 1]) == 2);
				REQUIRE(graph.hasEdge(edge_handles[i - 1]));
				REQUIRE(graph.getOther(edge_handles[i - 1], node_handles[i - 1]) == node_handles[i]);
				REQUIRE(graph.getOther(edge_handles[i - 1], node_handles[i]) == node_handles[i - 1]);
			}
		}
	}
	SECTION("ListGraph::addEdge() with weight") {
		ListGraph<TraversableNodeData, WeightedEdgeData> graph;
		ListGraph<TraversableNodeData, WeightedEdgeData>::node_handle node_handles[20];
		ListGraph<TraversableNodeData, WeightedEdgeData>::edge_handle edge_handles[19];
		for (int i = 0; i < 20; ++i) {
			node_handles[i] = graph.addNode();
			if (i > 0) {
				edge_handles[i - 1] = graph.addEdge(node_handles[i - 1], node_handles[i], 2);
				REQUIRE(graph.degree(node_handles[i]) == 1);
				if (i - 1 == 0)
					REQUIRE(graph.degree(node_handles[i - 1]) == 1);
				else
					REQUIRE(graph.degree(node_handles[i - 1]) == 2);
				REQUIRE(graph.hasEdge(edge_handles[i - 1]));
				REQUIRE(graph.getOther(edge_handles[i - 1], node_handles[i - 1]) == node_handles[i]);
				REQUIRE(graph.getOther(edge_handles[i - 1], node_handles[i]) == node_handles[i - 1]);
				auto &edata = graph.getEdge(edge_handles[i - 1]);
				REQUIRE(edata.weight_ == 2);
			}
		}
	}
}

TEST_CASE("ListGraph::removeNode(const NodeHandle&)") {
	using namespace graphlib;
	ListGraph<TraversableNodeData, EdgeData> graph;
	ListGraph<TraversableNodeData, EdgeData>::node_handle node_handles[4];
	for (int i = 0; i < 4; ++i) {
		node_handles[i] = graph.addNode();
		for (int j = i - 1; j >= 0; --j)
			graph.addEdge(node_handles[i], node_handles[j]);
	}
	graph.removeNode(node_handles[2]);

	REQUIRE(graph.nodeCount() == 3);
	REQUIRE(graph.degree(node_handles[3]) == 2);
	REQUIRE(graph.degree(node_handles[0]) == 2);
	REQUIRE(graph.degree(node_handles[1]) == 2);
	for (int i = 0; i < 4; ++i) {
		if (i == 2)
			continue;
		auto &list = graph[node_handles[i]];
		auto bit = list.begin();
		while (bit != list.end()) {
			auto nh = graph.getOther(*bit, node_handles[i]);
			REQUIRE(nh != node_handles[2]);
			++bit;
		}
	}

	graph.removeNode(node_handles[1]);
	REQUIRE(graph.degree(node_handles[0]) == 1);
	REQUIRE(graph.degree(node_handles[3]) == 1);
	for (int i = 0; i < 4; ++i) {
		if (i == 2 || i == 1)
			continue;
		auto &list = graph[node_handles[i]];
		REQUIRE(list.size() == 1);
		auto bit = list.begin();
		while (bit != list.end()) {
			auto nh = graph.getOther(*bit, node_handles[i]);
			REQUIRE(nh != node_handles[2]);
			REQUIRE(nh != node_handles[1]);
			++bit;
		}
	}
}

TEST_CASE("ListGraph::removeEdge(const EdgeHandle&)") {
	using namespace graphlib;
	ListGraph<TraversableNodeData, EdgeData> graph;
	ListGraph<TraversableNodeData, EdgeData>::node_handle node_handles[4];
	ListGraph<TraversableNodeData, EdgeData>::edge_handle edge_handles[6];
	int counter = 0;
	for (int i = 0; i < 4; ++i) {
		node_handles[i] = graph.addNode();
		for (int j = i - 1; j >= 0; --j)
			edge_handles[counter++] = graph.addEdge(node_handles[i], node_handles[j]);
	}
	graph.removeEdge(edge_handles[3]);
	REQUIRE(graph.degree(node_handles[2]) == 2);
	REQUIRE(graph.degree(node_handles[3]) == 2);

	graph.removeEdge(edge_handles[2]);
	REQUIRE(graph.degree(node_handles[2]) == 1);
	REQUIRE(graph.degree(node_handles[0]) == 2);

	graph.removeEdge(edge_handles[5]);
	REQUIRE(graph.degree(node_handles[0]) == 1);
	REQUIRE(graph.degree(node_handles[3]) == 1);
}

TEST_CASE("ListGraph lookup methods") {
	using namespace graphlib;
	ListGraph<TraversableNodeData, WeightedEdgeData> graph;
	ListGraph<TraversableNodeData, WeightedEdgeData>::node_handle node_handles[4];
	ListGraph<TraversableNodeData, WeightedEdgeData>::edge_handle edge_handles[6];
	int counter = 0;
	for (int i = 0; i < 4; ++i) {
		node_handles[i] = graph.addNode();
		for (int j = i - 1; j >= 0; --j)
			edge_handles[counter++] = graph.addEdge(node_handles[i], node_handles[j]);
	}
	SECTION("ListGraph::hasEdge()") {
		REQUIRE(graph.hasEdge(edge_handles[3]));
		REQUIRE(graph.hasEdge(edge_handles[5]));
		graph.removeNode(node_handles[3]);
		REQUIRE(graph.hasEdge(edge_handles[3]) == false);
		REQUIRE(graph.hasEdge(edge_handles[4]) == false);
		REQUIRE(graph.hasEdge(edge_handles[5]) == false);
	}
	SECTION("ListGraph::hasNode()") {
		for (int i = 0; i < 4; ++i)
			REQUIRE(graph.hasNode(node_handles[i]));
	}
	SECTION("ListGraph::getEdge()") {
		auto &edata = graph.getEdge(edge_handles[0]);
		REQUIRE(edata.weight_ == 1);
		auto cedata = const_cast<const ListGraph<TraversableNodeData, WeightedEdgeData>&>(graph).getEdge(edge_handles[0]);
		REQUIRE(cedata.weight_ == 1);
	}
	SECTION("ListGraph::getNode()") {
		auto &ndata = graph.getNode(node_handles[3]);
		REQUIRE(ndata.color_ == Color::WHITE);
		REQUIRE(ndata.pred_ == NodeHandle());
		REQUIRE(graph.degree(node_handles[3]) == 3);

		auto &cndata = const_cast<ListGraph<TraversableNodeData, WeightedEdgeData>&>(graph).getNode(node_handles[3]);
		REQUIRE(cndata.color_ == Color::WHITE);
		REQUIRE(cndata.pred_ == NodeHandle());
		REQUIRE(graph.degree(node_handles[3]) == 3);
	}
	SECTION("ListGraph::getOther/getBoth()") {
		auto onh = graph.getOther(edge_handles[5], node_handles[3]);
		REQUIRE(onh == node_handles[0]);
		REQUIRE(graph.degree(onh) == 3);

		auto [fst, snd] = graph.getBoth(edge_handles[3]);
		REQUIRE(fst == node_handles[3]);
		REQUIRE(snd == node_handles[2]);
		REQUIRE(graph.degree(fst) == 3);
		REQUIRE(graph.degree(snd) == 3);
	}
	SECTION("ListGraph::getOtherNode()") {
		auto odata = graph.getOtherNode(edge_handles[5], node_handles[3]);
		REQUIRE(odata.color_ == Color::WHITE);
		REQUIRE(odata.pred_ == NodeHandle());
		auto &codata = const_cast<ListGraph<TraversableNodeData, WeightedEdgeData>&>(graph).getOtherNode(edge_handles[5], node_handles[3]);
		REQUIRE(codata.color_ == Color::WHITE);
		REQUIRE(codata.pred_ == NodeHandle());
	}
	SECTION("ListGraph::operator[]()") {
		auto &list = graph[node_handles[3]];
		REQUIRE(list.size() == 3);
		for (auto eh : list) {
			auto [fst, snd] = graph.getBoth(eh);
			REQUIRE(graph.degree(fst) == 3);
			REQUIRE(graph.degree(snd) == 3);
		}
		auto &clist = const_cast<const ListGraph<TraversableNodeData, WeightedEdgeData>&>(graph)[node_handles[3]];
	}
	SECTION("ListGraph::nodeCount()") {
		REQUIRE(graph.nodeCount() == 4);
		graph.removeNode(node_handles[3]);
		REQUIRE(graph.nodeCount() == 3);
		graph.addNode();
		REQUIRE(graph.nodeCount() == 4);
	}
	SECTION("ListGraph::edgeCount()") {
		REQUIRE(graph.edgeCount() == 6);
		graph.removeEdge(edge_handles[5]);
		REQUIRE(graph.edgeCount() == 5);
		graph.addEdge(node_handles[3], node_handles[0]);
		REQUIRE(graph.edgeCount() == 6);
	}
	SECTION("ListGraph::beginNode/endNode/cbeginNode/cendNode") {
		auto nbit = graph.beginNode();
		auto neit = graph.endNode();
		while (nbit != neit) {
			auto &nodedata = graph.getNode(*nbit);
			REQUIRE(nodedata.color_ == Color::WHITE);
			REQUIRE(nodedata.pred_ == NodeHandle());
			++nbit;
		}
		auto cnbit = const_cast<ListGraph<TraversableNodeData, WeightedEdgeData>&>(graph).beginNode();
		auto cneit = const_cast<ListGraph<TraversableNodeData, WeightedEdgeData>&>(graph).endNode();

		auto ccnbit = graph.cbeginNode();
		auto ccneit = graph.cendNode();
	}
	SECTION("ListGraph::beginEdge/endEdge/cbeginEdge/cendEdge") {
		auto ebit = graph.beginEdge();
		auto eeit = graph.endEdge();
		while (ebit != eeit) {
			auto &edgedata = graph.getEdge(*ebit);
			REQUIRE(edgedata.weight_ == 1);
			++ebit;
		}
		auto cebit = const_cast<ListGraph<TraversableNodeData, WeightedEdgeData>&>(graph).beginEdge();
		auto ceeit = const_cast<ListGraph<TraversableNodeData, WeightedEdgeData>&>(graph).endEdge();

		auto ccebit = graph.cbeginEdge();
		auto cceeit = graph.cendEdge();
	}
	SECTION("ListGraph::getWeight(const EdgeHandle&)") {
		auto weight = graph.getWeight(edge_handles[0]);
		REQUIRE(weight == 1);
	}
}

TEST_CASE("ListGraph::setWeight(const EdgeHandle&, weight_type)") {
	using namespace graphlib;
	using node_handle = typename graph_traits<ListGraph<TraversableNodeData, WeightedEdgeData>>::node_handle;
	using edge_handle = typename graph_traits<ListGraph<TraversableNodeData, WeightedEdgeData>>::edge_handle;
	ListGraph<TraversableNodeData, WeightedEdgeData> graph;
	node_handle handles[2];
	handles[0] = graph.addNode();
	handles[1] = graph.addNode();
	edge_handle eh = graph.addEdge(handles[0], handles[1]);
	auto &edata = graph.getEdge(eh);
	REQUIRE(edata.weight_ == 1);
	graph.setWeight(eh, 2);
	REQUIRE(edata.weight_ == 2);
}

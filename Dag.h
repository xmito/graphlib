#ifndef DAG_H
#define DAG_H

#include <type_traits>
#include <forward_list>

#include "Dfs.h"
#include "Relax.h"
#include "InitializeSingleSource.h"
#include "GraphTraits.h"
#include "NodeData.h"

namespace graphlib {

template<typename Graph>
std::enable_if_t<Graph::directedTag &&
                 Graph::weightedTag &&
                 Graph::pathTag>
dag(Graph& graph, typename graph_traits<Graph>::node_handle &source) {

	using node_handle = typename graph_traits<Graph>::node_handle;
	using edge_handle = typename graph_traits<Graph>::edge_handle;

	std::pair<std::forward_list<node_handle>, bool> nodeList = dfs(graph);

	initializeSingleSource(graph, source);
	if (nodeList.second) return;

	for (auto it = nodeList.first.begin(); it != nodeList.first.end(); ++it) {
		auto &list = graph[*it];

		for (edge_handle edge : list) {
			auto &targetNode = graph.getTargetNode(edge);
			auto &sourceNode = graph.getSourceNode(edge);
			auto &edgeData = graph.getEdge(edge);
			if (targetNode.dist_ > sourceNode.dist_ + edgeData.weight_) relax(graph, edge);
		}
	}
}

} // namespace graphlib

#endif


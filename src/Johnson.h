#ifndef JOHNSON_H
#define JOHNSON_H
#include <vector>
#include <algorithm>
#include "BellmanFord.h"
#include "Dijkstra.h"
#include "graph_traits.h"
#include "Matrix.h"

namespace graphlib {

template<typename Graph,
		 typename = std::enable_if_t<Graph::directedTag &&
									 Graph::weightedTag &&
									 Graph::pathTag>>
Matrix<typename graph_traits<Graph>::distance_type> johnson(Graph &graph) {
	using node_handle = typename graph_traits<Graph>::node_handle;
	using edge_handle = typename graph_traits<Graph>::edge_handle;
	using distance_type = typename graph_traits<Graph>::distance_type;

	size_t nodes = graph.nodeCount();
	std::vector<edge_handle> ehandles;
	ehandles.reserve(nodes);
	node_handle source = graph.addNode();

	for (auto &nh : graph.nodes()) {
		if (nh == source)
			continue;
		ehandles.push_back(graph.addEdge(source, nh, 0));
	}

	if (!bellmanFord(graph, source))
		return Matrix<distance_type>();

	std::vector<distance_type> h;
	h.reserve(nodes);
	for (auto &nh : graph.nodes()) {
		if (nh == source)
			continue;
		h.push_back(graph.getNodeDist(nh));
	}

	/* Reweight edges from computed distances */
	for (auto &eh : graph.edges()) {
		auto &src_data = graph.getSourceNode(eh);
		auto &tg_data = graph.getTargetNode(eh);
		graph.modWeight(eh, src_data.dist_ - tg_data.dist_);
	}

	/* Remove additional edges and source node */
	graph.removeNode(source);

	/* Construct matrix for results */
	Matrix<distance_type> matrix(nodes, nodes);

	/* Compute dijkstra from each node */
	for (auto &unh : graph.nodes()) {
		dijkstra(graph, unh);
		for (auto &vnh : graph.nodes())
			matrix[unh.getId()][vnh.getId()] = graph.getNodeDist(vnh) +
			        h[vnh.getId()] - h[unh.getId()];
	}
	return matrix;
}

} // namespace graphlib

#endif

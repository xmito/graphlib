#ifndef JOHNSON_H
#define JOHNSON_H
#include <vector>
#include <algorithm>
#include "BellmanFord.h"
#include "Dijkstra.h"
#include "GraphTraits.h"
#include "Matrix.h"

namespace graphlib {

template<typename Graph>
std::enable_if_t<Graph::directedTag &&
                 Graph::weightedTag &&
                 Graph::pathTag, Matrix<typename graph_traits<Graph>::distance_type>>
johnson(Graph &graph) {
	using node_handle = typename graph_traits<Graph>::node_handle;
	using edge_handle = typename graph_traits<Graph>::edge_handle;
	using distance_type = typename graph_traits<Graph>::distance_type;

	size_t nodes = graph.nodeCount();
	std::vector<edge_handle> ehandles;
	ehandles.reserve(nodes);
	node_handle source = graph.addNode();

	for (auto nit = graph.beginNode(); nit != graph.endNode(); ++nit) {
		if (*nit == source)
			continue;
		ehandles.push_back(graph.addEdge(source, *nit, 0));
	}

	if (bellmanFord(graph, source) == false)
		return Matrix<distance_type>();
	else {
		std::vector<distance_type> h;
		h.reserve(nodes);
		for (auto nit = graph.beginNode(); nit != graph.endNode(); ++nit) {
			if (*nit == source)
				continue;
			auto &data = graph.getNode(*nit);
			h.push_back(data.dist_);
		}

		/* Reweight edges from computed distances */
		for (auto eit = graph.beginEdge(); eit != graph.endEdge(); ++eit) {
			auto &src_data = graph.getSourceNode(*eit);
			auto &tg_data = graph.getTargetNode(*eit);
			graph.modWeight(*eit, src_data.dist_ - tg_data.dist_);
		}

		/* Remove additional edges and source node */
		graph.removeNode(source);

		/* Construct matrix for results */
		Matrix<distance_type> matrix(nodes, nodes);

		/* Compute dijkstra from each node */
		for (auto unit = graph.beginNode(); unit != graph.endNode(); ++unit) {
			dijkstra(graph, *unit);
			for (auto vnit = graph.beginNode(); vnit != graph.endNode(); ++vnit) {
				auto & data = graph.getNode(*vnit);
				matrix[unit->getId()][vnit->getId()] = data.dist_ + h[vnit->getId()] - h[unit->getId()];
			}
		}
		return matrix;
	}
}

}

#endif

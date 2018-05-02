#ifndef FLOYDWARSHALL_H
#define FLOYDWARSHALL_H

#include <type_traits>
#include <climits>

#include "Matrix.h"
#include "NodeData.h"
#include "GraphTraits.h"

namespace graphlib {

/* Assumes directed weighted graphs with no negative weight cycles*/
template<typename Graph>
std::enable_if_t<Graph::directedTag &&
                 Graph::weightedTag &&
                 Graph::pathTag,
				 Matrix<typename graph_traits<Graph>::distance_type>>
floydWarshall(const Graph& graph) {

	using edge_handle = typename graph_traits<Graph>::edge_handle;
	using distance_type = typename graph_traits<Graph>::distance_type;
	using weight_type = typename graph_traits<Graph>::weight_type;

	Matrix resMatrix(graph.nodeCount(), graph.nodeCount(), std::numeric_limits<distance_type>::max());
	for (auto it = graph.beginNode(); it != graph.endNode(); ++it) {

		for (edge_handle edgeHandle : graph[*it]) {
			auto targetNode = graph.getTarget(edgeHandle);
			resMatrix[(*it).getId()][targetNode.getId()] = graph.getWeight(edgeHandle);
		}
		resMatrix[(*it).getId()][(*it).getId()] = 0;
	}

	for (auto it = graph.beginNode(); it != graph.endNode(); ++it) {
		auto k = (*it).getId();

		for (auto it1 = graph.beginNode(); it1 != graph.endNode(); ++it1) {
			auto i = (*it1).getId();

			for (auto it2 = graph.beginNode(); it2 != graph.endNode(); ++it2) {
				auto j = (*it2).getId();

				if (resMatrix[i][k] != std::numeric_limits<distance_type>::max() 
						&& resMatrix[k][j] != std::numeric_limits<distance_type>::max()) {
					if (resMatrix[i][j] > resMatrix[i][k] + resMatrix[k][j]) {
						resMatrix[i][j] = resMatrix[i][k] + resMatrix[k][j];
					}
				}
			}
		}
	}


	return resMatrix;
}

} // namespace graphlib

#endif

#ifndef DAG_H
#define DAG_H

#include <type_traits>
#include <forward_list>
#include <stack>
#include <vector>
#include <algorithm>

#include "InitializeSingleSource.h"
#include "Relax.h"
#include "GraphTraits.h"
#include "NodeData.h"
#include "Dfs.cpp"

namespace graphlib {

/* dag - function computes shortest paths from source node.
 * As a return value, function returns whether it finished
 * properly */
template<typename Graph>
std::enable_if_t<Graph::directedTag &&
                 Graph::weightedTag &&
                 Graph::pathTag, bool>
dag(Graph &graph,
    const typename graph_traits<Graph>::node_handle &source) {
	using node_handle = typename graph_traits<Graph>::node_handle;

	std::vector<node_handle> vec;

	initializeSingleSource(graph, source);
	if (topologicalSort(graph, source, vec))
		return false;
	for (auto nit = vec.begin(); nit != vec.end(); ++nit) {
		for (auto &eh : graph[*nit]) {
			relax(graph, eh);
		}
	}
	return true;
}

/* topologicalSort - function finds topological sort in directed
 * acyclic graph. Unlike regular topological sort with dfs, topologicalSort
 * pushes to a vector only those nodes, that are reachable from
 * source node. This prevents higher computation times, when graph
 * has a lot of components, that are unreachable from the source node.
 * Then, in the dag function, we are not performing unnecessary edge
 * relaxations. topologicalSort returns true, if it encounters some
 * back edge, otherwise false */
template<typename Graph>
std::enable_if_t<Graph::directedTag &&
                 Graph::traversableTag, bool>
topologicalSort(Graph &graph,
                const typename graph_traits<Graph>::node_handle &nh,
                std::vector<typename graph_traits<Graph>::node_handle> &vec) {

	return dfsVisit(graph, nh, &vec);
}

} // namespace graphlib

#endif


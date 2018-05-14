#ifndef DAG_H
#define DAG_H

#include <type_traits>
#include <forward_list>

#include "InitializeSingleSource.h"
#include "Relax.h"
#include "GraphTraits.h"
#include "NodeData.h"

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

	std::forward_list<node_handle> list;
	initializeSingleSource(graph, source);
	if (topologicalSort(graph, source, list))
		return false;
	for (auto nit = list.begin(); nit != list.end(); ++nit) {
		for (auto &eh : graph[*nit]) {
			relax(graph, eh);
		}
	}
	return true;
}

/* topologicalSort - function finds topological sort in directed
 * acyclic graph. Unlike regular topological sort with dfs, topologicalSort
 * pushes to forward list only those nodes, that are reachable from
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
                std::forward_list<typename graph_traits<Graph>::node_handle> &list) {
	using node_handle = typename graph_traits<Graph>::node_handle;
	using edge_handle = typename graph_traits<Graph>::edge_handle;
	using adj_iterator = typename graph_traits<Graph>::adj_iterator;
	using stack_pair = std::pair<node_handle, adj_iterator>;

	std::stack<stack_pair> stack;
	stack.emplace(nh, graph[nh].begin());
	while (!stack.empty()) {
		stack_pair &top = stack.top();
		if (top.second == graph[top.first].end()) {
			list.push_front(top.first);
			graph.setNodeColor(top.first, Color::BLACK);
			stack.pop();
		} else {
			graph.setNodeColor(top.first, Color::GRAY);
			edge_handle eh = *top.second;

			/* Get target node of edge and if its color_ is white,
			 * set its predecessor, color to Color::GRAY and emplace
			 * corresponding stack_pair on top of the stack */
			node_handle tg = graph.getTarget(eh);
			Color tg_color = graph.getNodeColor(tg);
			if (tg_color == Color::WHITE) {
				graph.setNodeColor(tg, Color::GRAY);
				graph.setNodePred(tg, top.first);
				stack.emplace(tg, graph[tg].begin());
			} else if (tg_color == Color::GRAY)
				return true;

			/* Increase top adj_iterator, so that next time loop
			 * encounters same node on top of the stack, it has
			 * iterator pointing to unexplored edge */
			++top.second;
		}
	}
	return false;
}

} // namespace graphlib

#endif


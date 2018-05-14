#ifndef DFS_H
#define DFS_H
#include <type_traits>
#include <stack>
#include "GraphTraits.h"
#include "NodeData.h"

namespace graphlib {

template<typename Graph>
std::enable_if_t<Graph::traversableTag>
dfs(Graph& graph) {
	for (auto &nh : graph.nodes())
		if (graph.getNodeColor(nh) == Color::WHITE)
			dfsVisit(graph, nh);
}

template<typename Graph>
void dfsVisit(Graph& graph,
         const typename graph_traits<Graph>::node_handle &nh) {
	using node_handle = typename graph_traits<Graph>::node_handle;
	using edge_handle = typename graph_traits<Graph>::edge_handle;
	using adj_iterator = typename graph_traits<Graph>::adj_iterator;
	using stack_pair = std::pair<node_handle, adj_iterator>;

	std::stack<stack_pair> stack;
	stack.emplace(nh, graph[nh].begin());
	while (!stack.empty()) {
		stack_pair &top = stack.top();
		if (top.second == graph[top.first].end()) {
			graph.setNodeColor(top.first, Color::BLACK);
			stack.pop();
		} else {
			graph.setNodeColor(top.first, Color::GRAY);
			edge_handle eh = *top.second;

			/* Get target node of edge and if its color_ is white,
			 * set its predecessor, color to Color::GRAY and emplace
			 * corresponding stack_pair on top of the stack */
			node_handle tg;
			if constexpr (Graph::directedTag)
			    tg = graph.getTarget(eh);
			else
			    tg = graph.getOther(eh, top.first);

			if (graph.getNodeColor(tg) == Color::WHITE) {
				graph.setNodeColor(tg, Color::GRAY);
				graph.setNodePred(tg, top.first);
				stack.emplace(tg, graph[tg].begin());
			}
			/* Increase top adj_iterator, so that next time loop
			 * encounters same node on top of the stack, it has
			 * iterator pointing to unexplored edge */
			++top.second;
		}
	}
}

}

#endif

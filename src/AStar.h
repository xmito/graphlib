#ifndef A_STAR_H
#define A_STAR_H
#include <cstdlib>
#include <limits>
#include <cmath>
#include <algorithm>
#include "GraphTraits.h"
#include "BinaryHeap.h"
#include "FibonacciHeap.h"
#include "Relax.h"
#include "InitializeSingleSource.h"
#include "NodeData.h"

#define MAGIC -0.585786438 /* sqrt(2) - 2 */

namespace graphlib {

template<typename Graph>
std::enable_if_t<Graph::heuristicpathTag, size_t>
manhattanDistance(const Graph &graph,
                  const typename graph_traits<Graph>::node_handle &nha,
                  const typename graph_traits<Graph>::node_handle &nhb) {
	auto &nha_loc = graph.getNodeLoc(nha);
	auto &nhb_loc = graph.getNodeLoc(nhb);
	return std::abs(nha_loc.x_ - nhb_loc.x_) + std::abs(nha_loc.y_ - nhb_loc.y_);
}

template<typename Graph>
std::enable_if_t<Graph::heuristicpathTag, double>
euclideanDistance(const Graph &graph,
                  const typename graph_traits<Graph>::node_handle &nha,
                  const typename graph_traits<Graph>::node_handle &nhb) {
	auto &nha_loc = graph.getNodeLoc(nha);
	auto &nhb_loc = graph.getNodeLoc(nhb);
	size_t dx = std::abs(nha_loc.x_ - nhb_loc.x_);
	size_t dy = std::abs(nha_loc.y_ - nhb_loc.y_);
	return std::sqrt(dx*dx + dy*dy);
}

template<typename Graph>
std::enable_if_t<Graph::heuristicpathTag, double>
diagonalDistance(const Graph &graph,
                 const typename graph_traits<Graph>::node_handle &nha,
                 const typename graph_traits<Graph>::node_handle &nhb) {
	auto &nha_loc = graph.getNodeLoc(nha);
	auto &nhb_loc = graph.getNodeLoc(nhb);
	size_t dx = std::abs(nha_loc.x_ - nhb_loc.x_);
	size_t dy = std::abs(nha_loc.y_ - nhb_loc.y_);
	return dx + dy + (MAGIC)*std::min(dx, dy);
}

template<typename Graph,
         typename Heuristic,
         typename PriorityQueue = BinHeap<Graph, LessHeuristic<Graph>>>
std::enable_if_t<Graph::heuristicpathTag &&
                 Graph::weightedTag>
aStar(Graph& graph,
      const typename graph_traits<Graph>::node_handle &source,
      const typename graph_traits<Graph>::node_handle &target,
      const Heuristic &heuristic) {
	using node_handle = typename graph_traits<Graph>::node_handle;
	using distance_type = typename graph_traits<Graph>::distance_type;

	initializeSingleSource(graph, source);
	LessHeuristic<Graph> comp(graph);
	PriorityQueue pq(comp);
	pq.push(source);
	while (!pq.empty()) {
		node_handle top = pq.top();
		if (top == target)
			break;
		pq.pop();
		for (auto &eh : graph[top]) {
			node_handle tg_edge;
			if constexpr (Graph::directedTag)
			    tg_edge = graph.getTarget(eh);
			else
			    tg_edge = graph.getOther(eh, top);
			distance_type dist = graph.getNodeDist(tg_edge);
			bool ret;
			if constexpr (Graph::directedTag)
			    ret = relax(graph, eh);
			else
			    ret = relax(graph, eh, top);
			graph.setNodePrio(tg_edge, graph.getNodeDist(tg_edge) +
			                  heuristic(graph, tg_edge, target));
			if (dist == std::numeric_limits<distance_type>::max())
				pq.push(tg_edge);
			else if (ret)
				pq.decUpdate(tg_edge);
		}
	}
}

} // namespace graphlib
#endif

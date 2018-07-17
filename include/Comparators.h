#ifndef COMPARATORS_H
#define COMPARATORS_H
#include <type_traits>
#include "graph_traits.h"

namespace graphlib {

template<typename Graph,
         typename = std::enable_if_t<Graph::pathTag>>
class LessDistance {
	using node_handle = typename graph_traits<Graph>::node_handle;
	const Graph *graph_;

public:
	LessDistance(const Graph *graph) : graph_(graph) {}
	bool operator()(const node_handle &nha, const node_handle &nhb) const {
		return graph_->getNodeDist(nha) < graph_->getNodeDist(nhb);
	}
};

template<typename Graph,
         typename = std::enable_if_t<Graph::heuristicpathTag>>
class LessHeuristic {
	using node_handle = typename graph_traits<Graph>::node_handle;
	const Graph *graph_;

public:
	LessHeuristic(const Graph &graph) : graph_(&graph) {}
	bool operator()(const node_handle &nha, const node_handle &nhb) {
		return graph_->getNodePrio(nha) < graph_->getNodePrio(nhb);
	}
};


} // namespace graphlib

#endif

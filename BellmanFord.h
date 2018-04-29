#ifndef BELLMAN_FORD_H
#define BELLMAN_FORD_H
#include <type_traits>
#include <climits>
#include "GraphTraits.h"
#include "NodeData.h"
#include "InitializeSingleSource.h"
#include "Relax.h"

namespace graphlib {

template<typename Graph>
std::enable_if_t<Graph::directedTag &&
                 Graph::weightedTag &&
                 Graph::pathTag, bool>
bellmanFord(Graph &graph,
            const typename graph_traits<Graph>::node_handle &source) {
	using weight_type = typename graph_traits<Graph>::weight_type;
	initializeSingleSource(graph, source);
	size_t nonodes = graph.nodeCount();
	while(--nonodes) {
		for (auto eit = graph.beginEdge(); eit != graph.endEdge(); ++eit) {
			relax(graph, *eit);
		}
	}
	for (auto eit = graph.beginEdge(); eit != graph.endEdge(); ++eit) {
		weight_type weight = graph.getWeight(*eit);
		auto &src_data = graph.getSourceNode(*eit);
		auto &tg_data = graph.getTargetNode(*eit);
		if (src_data.dist_ != LLONG_MAX && tg_data.dist_ > src_data.dist_ + weight)
			return false;
	}
	return true;
}

}
#endif

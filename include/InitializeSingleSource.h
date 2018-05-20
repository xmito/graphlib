#ifndef INITIALIZE_SINGLE_SOURCE_H
#define INITIALIZE_SINGLE_SOURCE_H
#include <type_traits>
#include  <limits>
#include "GraphTraits.h"
#include "NodeData.h"

namespace graphlib {

template<typename Graph>
std::enable_if_t<Graph::pathTag>
initializeSingleSource(Graph& graph,
                const typename graph_traits<Graph>::node_handle &source) {
	using node_handle = typename graph_traits<Graph>::node_handle;
	using distance_type = typename graph_traits<Graph>::distance_type;

	for (auto &nh : graph.nodes()) {
		graph.setNodeColor(nh, Color::WHITE);
		graph.setNodeDist(nh, std::numeric_limits<distance_type>::max());
		graph.setNodePred(nh, node_handle());
	}
	graph.setNodeDist(source, 0);
}

}

#endif

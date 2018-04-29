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
	for (auto it = graph.beginNode(); it != graph.endNode(); ++it) {
		auto &data = graph.getNode(*it);
		data.color_ = Color::WHITE;
		data.dist_ = std::numeric_limits<distance_type>::max();
		data.pred_ = node_handle();
	}
	auto &data = graph.getNode(source);
	data.dist_ = 0;
}

}

#endif

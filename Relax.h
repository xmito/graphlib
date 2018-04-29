#ifndef RELAX_H
#define RELAX_H
#include <type_traits>
#include <climits>
#include "GraphTraits.h"

namespace graphlib {

template<typename Graph>
std::enable_if_t<Graph::pathTag && Graph::directedTag>
relax(Graph &graph,
      const typename graph_traits<Graph>::edge_handle &eh) {
	using weight_type = typename graph_traits<Graph>::weight_type;
	auto src_nh = graph.getSource(eh);
	auto &src_data = graph.getSourceNode(eh);
	auto &tg_data = graph.getTargetNode(eh);
	weight_type weight = graph.getWeight(eh);
	if (src_data.dist_ != LLONG_MAX && tg_data.dist_ > src_data.dist_ + weight) {
		tg_data.dist_ = src_data.dist_ + weight;
		tg_data.pred_ = src_nh;
	}
}

}

#endif

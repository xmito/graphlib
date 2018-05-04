#ifndef COMPARATORS_H
#define COMPARATORS_H
#include <type_traits>
#include "GraphTraits.h"

namespace graphlib {

template<typename Graph,
         typename = std::enable_if_t<Graph::pathTag>>
class LessDistance {
	using node_handle = typename graph_traits<Graph>::node_handle;
	const Graph *graph_;

public:
	LessDistance(const Graph *graph) : graph_(graph) {}
	bool operator()(const node_handle &nha, const node_handle &nhb) const {
		auto &nha_data = graph_->getNode(nha);
		auto &nhb_data = graph_->getNode(nhb);
		return nha_data.dist_ < nhb_data.dist_;
	}
};

}

#endif

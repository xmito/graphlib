#ifndef RELAX_H
#define RELAX_H
#include <type_traits>
#include <climits>
#include <limits>
#include "GraphTraits.h"

namespace graphlib {

template<typename Graph>
std::enable_if_t<Graph::pathTag && Graph::directedTag>
relax(Graph &graph,
      const typename graph_traits<Graph>::edge_handle &eh) {
	using weight_type = typename graph_traits<Graph>::weight_type;
	using distance_type = typename graph_traits<Graph>::distance_type;
	auto src_nh = graph.getSource(eh);
	auto &src_data = graph.getSourceNode(eh);
	auto &tg_data = graph.getTargetNode(eh);
	weight_type weight = graph.getWeight(eh);
	if (src_data.dist_ != std::numeric_limits<distance_type>::max() &&
	        tg_data.dist_ > src_data.dist_ + weight) {
		tg_data.dist_ = src_data.dist_ + weight;
		tg_data.pred_ = src_nh;
	}
}

template<typename Graph>
std::enable_if_t<Graph::pathTag && !Graph::directedTag>
relax(Graph &graph,
      const typename graph_traits<Graph>::edge_handle &eh) {
	using weight_type = typename graph_traits<Graph>::weight_type;
	using distance_type = typename graph_traits<Graph>::distance_type;
	auto [fst, snd] = graph.getBoth(eh);
	auto &fst_data = graph.getNode(fst);
	auto &snd_data = graph.getNode(snd);
	weight_type weight = graph.getWeight(eh);
	/* Undirected edges are basically two directed edges */
	if (fst_data.dist_ != std::numeric_limits<distance_type>::max() &&
	        snd_data.dist_ > fst_data.dist_ + weight) {
		snd_data.dist_ = fst_data.dist_ + weight;
		snd_data.pred_ = fst;
	}
	if (snd_data.dist_ != std::numeric_limits<distance_type>::max() &&
	        fst_data.dist_ > snd_data.dist_ + weight) {
		fst_data.dist_ = snd_data.dist_ + weight;
		fst_data.pred_ = snd;
	}
}

}

#endif

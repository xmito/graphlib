#ifndef RELAX_H
#define RELAX_H
#include <type_traits>
#include <climits>
#include <limits>
#include "graph_traits.h"

namespace graphlib {

template<typename Graph>
std::enable_if_t<Graph::pathTag && Graph::directedTag, bool>
relax(Graph &graph,
      const typename graph_traits<Graph>::edge_handle &eh) {
	using weight_type = typename graph_traits<Graph>::weight_type;
	using distance_type = typename graph_traits<Graph>::distance_type;

	auto src_nh = graph.getSource(eh);
	auto tg_nh = graph.getTarget(eh);
	distance_type src_dist = graph.getNodeDist(src_nh);
	weight_type weight = graph.getWeight(eh);
	if (src_dist != std::numeric_limits<distance_type>::max() &&
	        graph.getNodeDist(tg_nh) > src_dist + weight) {
		graph.setNodeDist(tg_nh, src_dist + weight);
		graph.setNodePred(tg_nh, src_nh);
		return true;
	}
	return false;
}

/* relax for undirected graphs, which relaxes undirected edge
 * in both directions. This is useful for BellmanFord implementation */
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

/* relax for undirected graphs, predominantly used by dijkstra shortest
 * path algorithm, which requires relaxation of undirected edge only in
 * one direction */
template<typename Graph>
std::enable_if_t<Graph::pathTag &&
                 !Graph::directedTag, bool>
relax(Graph &graph,
      const typename graph_traits<Graph>::edge_handle &eh,
      const typename graph_traits<Graph>::node_handle &src) {
	using weight_type = typename graph_traits<Graph>::weight_type;
	using distance_type = typename graph_traits<Graph>::distance_type;
	using node_handle = typename graph_traits<Graph>::node_handle;

	node_handle tg = graph.getOther(eh, src);
	distance_type tg_dist = graph.getNodeDist(tg);
	distance_type src_dist = graph.getNodeDist(src);
	weight_type weight = graph.getWeight(eh);
	if (src_dist != std::numeric_limits<distance_type>::max() &&
	        tg_dist > src_dist + weight) {
		graph.setNodeDist(tg, src_dist + weight);
		graph.setNodePred(tg, src);
		return true;
	}
	return false;
}

} // namespace graphlib

#endif

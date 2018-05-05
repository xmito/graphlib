#ifndef BFS_H
#define BFS_H
#include <queue>
#include <type_traits>
#include "GraphTraits.h"
#include "NodeData.h"

namespace graphlib {

template<typename Graph>
std::enable_if_t<Graph::traversableTag>
bfs(Graph& graph,
    const typename graph_traits<Graph>::node_handle &snh) {
	using node_handle = typename graph_traits<Graph>::node_handle;

	std::queue<node_handle> que;
	auto &sdata = graph.getNode(snh);
	sdata.color_ = Color::GRAY;
	que.push(snh);
	while (!que.empty()) {
		node_handle nh = que.front();
		auto &nh_data = graph.getNode(nh);
		que.pop();
		for (auto eh : graph[nh]) {
			node_handle tg;
			if constexpr (Graph::directedTag)
			    tg = graph.getTarget(eh);
			else
			    tg = graph.getOther(eh, nh);
			auto &tg_data = graph.getNode(tg);
			if (tg_data.color_ == Color::WHITE) {
				tg_data.color_ = Color::GRAY;
				tg_data.pred_ = nh;
				if constexpr (Graph::pathTag)
				    tg_data.dist_ = nh_data.dist_ + 1;
				que.push(tg);
			}
		}
		nh_data.color_ = Color::BLACK;
	}
}

}


#endif

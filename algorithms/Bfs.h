#ifndef BFS_H
#define BFS_H
#include <queue>
#include <type_traits>
#include "graph_traits.h"
#include "NodeData.h"

namespace graphlib {

/**
 * @brief Breadth-first search
 *
 * @param graph Graph to run BFS on
 * @param snh Source node handle
 */
template <typename Graph, typename = std::enable_if_t<Graph::traversableTag>>
void bfs(Graph &graph, const typename graph_traits<Graph>::node_handle &snh) {
    using node_handle = typename graph_traits<Graph>::node_handle;
    std::queue<node_handle> que;

    graph.setNodeColor(snh, Color::GRAY);
    que.push(snh);
    while (!que.empty()) {
        node_handle nh = que.front();
        que.pop();
        for (auto eh : graph[nh]) {
            node_handle tg;
            if constexpr (Graph::directedTag)
                tg = graph.getTarget(eh);
            else
                tg = graph.getOther(eh, nh);
            if (graph.getNodeColor(tg) == Color::WHITE) {
                graph.setNodeColor(tg, Color::GRAY);
                graph.setNodePred(tg, nh);
                if constexpr (Graph::pathTag) {
                    auto dist = graph.getNodeDist(nh);
                    graph.setNodeDist(tg, dist + 1);
                }
                que.push(tg);
            }
        }
        graph.setNodeColor(nh, Color::BLACK);
    }
}

} // namespace graphlib

#endif

#ifndef DIJKSTRA_H
#define DIJKSTRA_H
#include <type_traits>
#include <limits>
#include "graph_traits.h"
#include "NodeData.h"
#include "InitializeSingleSource.h"
#include "Relax.h"
#include "BinaryHeap.h"
#include "Comparators.h"

namespace graphlib {

template <typename Graph, typename PriorityQueue = BinaryHeap<Graph>,
          typename = std::enable_if_t<Graph::weightedTag && Graph::pathTag>>
bool dijkstra(Graph &graph,
              const typename graph_traits<Graph>::node_handle &source) {
    using node_handle = typename graph_traits<Graph>::node_handle;
    initializeSingleSource(graph, source);
    PriorityQueue pq(graph);
    while (!pq.empty()) {
        node_handle top = pq.top();
        pq.pop();
        for (auto &eh : graph[top]) {
            if (graph.getWeight(eh) < 0)
                return false;
            if constexpr (Graph::directedTag) {
                if (relax(graph, eh))
                    pq.decUpdate(graph.getTarget(eh));
            } else {
                if (relax(graph, eh, top))
                    pq.decUpdate(graph.getOther(eh, top));
            }
        }
    }
    return true;
}

} // namespace graphlib

#endif

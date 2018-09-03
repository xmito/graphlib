#ifndef BELLMAN_FORD_H
#define BELLMAN_FORD_H
#include <type_traits>
#include <climits>
#include "graph_traits.h"
#include "NodeData.h"
#include "InitializeSingleSource.h"
#include "Relax.h"

namespace graphlib {

/**
 * @brief Bellman-Ford algorithm
 *
 * @param graph Graph used to run Bellman-Ford algorithm on
 * @param source Source node
 * @return false if Graph contains a negative cycle, true otherwise
 */
template <typename Graph>
std::enable_if_t<Graph::directedTag && Graph::weightedTag && Graph::pathTag,
                 bool>
bellmanFord(Graph &graph,
            const typename graph_traits<Graph>::node_handle &source) {
    initializeSingleSource(graph, source);
    size_t nonodes = graph.nodeCount();
    while (--nonodes)
        for (auto &eh : graph.edges())
            relax(graph, eh);
    for (auto &eh : graph.edges())
        if (relax(graph, eh))
            return false;
    return true;
}

/**
 * @brief Bellman-Ford algorithm
 *
 * @param graph Graph used to run Bellman-Ford algorithm on
 * @param source Source node
 * @return false if Graph contains a negative cycle, true otherwise
 */
template <typename Graph>
std::enable_if_t<!Graph::directedTag && Graph::weightedTag && Graph::pathTag,
                 bool>
bellmanFord(Graph &graph,
            const typename graph_traits<Graph>::node_handle &source) {
    initializeSingleSource(graph, source);
    size_t nonodes = graph.nodeCount();
    while (--nonodes) {
        for (auto &eh : graph.edges()) {
            if (graph.getWeight(eh) < 0)
                return false;
            relax(graph, eh);
        }
    }
    return true;
}

} // namespace graphlib
#endif

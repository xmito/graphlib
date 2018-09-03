#ifndef DAGSHORTESTPATH_H
#define DAGSHORTESTPATH_H

#include <type_traits>
#include <forward_list>
#include <stack>
#include <vector>
#include <algorithm>

#include "InitializeSingleSource.h"
#include "Relax.h"
#include "graph_traits.h"
#include "NodeData.h"
#include "Dfs.h"

namespace graphlib {

/** 
 * @brief Compute shortest paths from a source node in a directed acyclic graph.
 * @param graph Directed acyclic graph to perform DAGShortestPath on
 * @param source Source node handle
 * @return false if a back edge was encountered, true otherwise
 */
template <typename Graph,
          typename = std::enable_if_t<Graph::directedTag &&
                                      Graph::weightedTag && Graph::pathTag>>
bool DAGShortestPath(Graph &graph,
         const typename graph_traits<Graph>::node_handle &source) {

    using node_handle = typename graph_traits<Graph>::node_handle;
    std::vector<node_handle> vec;

    initializeSingleSource(graph, source);
    if (topologicalSort(graph, source, vec))
        return false;
    for (auto nit : vec) {
        for (auto &eh : graph[nit]) {
            relax(graph, eh);
        }
    }
    return true;
}

/** 
 * Function finds topological sort in directed acyclic graph.
 * Unlike regular topological sort with dfs, topologicalSort
 * pushes to a vector only those nodes, that are reachable from
 * source node. This prevents higher computation times, when graph
 * has a lot of components, that are unreachable from the source node.
 * Then, in the DAGShortestPath function, we are not performing unnecessary edge
 * relaxations
 *
 * @param graph Graph used to do calculations on
 * @param nh Source node handle 
 * @param vec Vector used for storing a topological order of graph
 * @return true if graph contains a back edge, false otherwise
 */
template <
    typename Graph,
    typename = std::enable_if_t<Graph::directedTag && Graph::traversableTag>>
bool topologicalSort(
    Graph &graph, const typename graph_traits<Graph>::node_handle &nh,
    std::vector<typename graph_traits<Graph>::node_handle> &vec) {
    return dfsVisit(graph, nh, &vec);
}

} // namespace graphlib

#endif

#ifndef COMPARATORS_H
#define COMPARATORS_H
#include <type_traits>
#include "graph_traits.h"

namespace graphlib {

/**
 * @brief LessDistance is a custom comparator class used to compare distance values associated with nodes
 * tparam Graph Graph type, that has getNodeDist method able to retrieve node distance associated with provided node handle
 */
template <typename Graph, typename = std::enable_if_t<Graph::pathTag>>
class LessDistance {
    using node_handle = typename graph_traits<Graph>::node_handle;
    const Graph *graph_;

  public:
    /**
     * @brief Constructs LessDistance instance from pointer to Graph
     * @param graph Pointer to constant Graph
     */
    explicit LessDistance(const Graph *graph) : graph_(graph) {}
    /**
     * @brief Returns true if distance associated with the first node handle is less than distance of the second one
     * @param nha Valid node handle
     * @param nhb Valid node handle
     * @return bool value
     */
    bool operator()(const node_handle &nha, const node_handle &nhb) const {
        return graph_->getNodeDist(nha) < graph_->getNodeDist(nhb);
    }
};

/**
 * @brief LessHeuristic is a custom comparator class used to compare two nodes according to heuristic priority
 * @tparam Graph Type of graph, that implements getNodePrio method able to retrieve heuristic priority of node
 */
template <typename Graph, typename = std::enable_if_t<Graph::heuristicpathTag>>
class LessHeuristic {
    using node_handle = typename graph_traits<Graph>::node_handle;
    const Graph *graph_;

  public:
    /**
     * @brief Constructs LessHeuristic instance from constant reference to graph
     * @param graph Constant reference to Graph
     */
    explicit LessHeuristic(const Graph &graph) : graph_(&graph) {}
    /**
     * @brief Returns true if heuristic priority associated with the first node handle is less than that of the second one
     * @param nha Valid node handle
     * @param nhb Valid node handle
     * @return bool value
     */
    bool operator()(const node_handle &nha, const node_handle &nhb) {
        return graph_->getNodePrio(nha) < graph_->getNodePrio(nhb);
    }
};

} // namespace graphlib

#endif

#ifndef ID_DFS_H
#define ID_DFS_H
#include <stack>
#include "graph_traits.h"
#include "NodeData.h"

namespace graphlib {

/**
 * @brief Iterative deepening Depth-first search algorithm
 * @param graph Graph to run IDDFS
 * @param root_nh Node from which to start search
 * @param goal_nh Node to search for
 * @return bool value, that signifies, whether the goal node specified by provided goal_nh node handle was found
 */
template <typename Graph, typename = std::enable_if_t<Graph::traversableTag>>
bool iddfs(Graph &graph,
           const typename graph_traits<Graph>::node_handle &root_nh,
           const typename graph_traits<Graph>::node_handle &goal_nh,
           size_t max_depth) {
    size_t depth = 0;
    while (depth <= max_depth) {
        bool found = depthLimitedSearch(graph, root_nh, goal_nh, depth);
        if (found)
            return true;
        ++depth;
    }
    return false;
}

/**
 * @brief depthLimitedSearch does a dfs search on directed graphs, which is limited in its depth. If the function reaches depth specified by provided depth function argument, it starts backtracking
 * @tparam Graph Type of graph
 * @param graph Graph instance on which to run
 * @param root_nh Root node handle, that specifies starting node
 * @param goal_nh Goal node handle we are searching for
 * @param depth Maximum depth to consider
 * @return bool value, that signifies presence of node with specified goal_nh node handle
 */
template <typename Graph>
std::enable_if_t<Graph::traversableTag && Graph::directedTag, bool>
depthLimitedSearch(Graph &graph, const typename graph_traits<Graph>::node_handle &root_nh,
                   const typename graph_traits<Graph>::node_handle &goal_nh, size_t depth) {

    using traits = graph_traits<Graph>;
    using node_handle = typename traits::node_handle;
    using edge_handle = typename traits::edge_handle;
    using adj_iterator = typename traits::adj_iterator;
    using stack_pair = std::pair<node_handle, adj_iterator>;

    std::stack<stack_pair> stack;
    stack.emplace(root_nh, graph[root_nh].begin());
    while (!stack.empty()) {
        stack_pair &top = stack.top();
        if (stack.size() - 1 == depth && top.first == goal_nh)
            return true;
        if ((stack.size() - 1 == depth && top.first != goal_nh) ||
            (top.second == graph[top.first].end())) {
            stack.pop();
        } else {
            edge_handle eh = *top.second;
            node_handle tg = graph.getTarget(eh);
            stack.emplace(tg, graph[tg].begin());
            ++top.second;
        }
    }
    return false;
}

/**
 * @brief depthLimitedSearch does a dfs search on undirected graphs, which is limited in its depth. If the function reaches depth specified by provided depth function argument, it starts backtracking
 * @tparam Graph Type of graph
 * @param graph Graph instance on which to run
 * @param root_nh Root node handle, that specifies starting node
 * @param goal_nh Goal node handle we are searching for
 * @param depth Maximum depth to consider
 * @return bool value, that signifies presence of node with specified goal_nh node handle
 */
template <typename Graph>
std::enable_if_t<Graph::traversableTag && !Graph::directedTag, bool>
depthLimitedSearch(Graph &graph, const typename graph_traits<Graph>::node_handle &root_nh,
                   const typename graph_traits<Graph>::node_handle &goal_nh, size_t depth) {

    using traits = graph_traits<Graph>;
    using node_handle = typename traits::node_handle;
    using adj_iterator = typename traits::adj_iterator;
    using stack_pair = std::pair<node_handle, adj_iterator>;

    std::stack<stack_pair> stack;
    stack.emplace(root_nh, graph[root_nh].begin());
    while (!stack.empty()) {
        stack_pair &top = stack.top();
        graph.setNodeColor(top.first, Color::GRAY);

        if (stack.size() - 1 == depth && top.first == goal_nh) {
            graph.setNodeColor(top.first, Color::WHITE);
            stack.pop();
            while (!stack.empty()) {
                stack_pair stop = stack.top();
                graph.setNodeColor(stop.first, Color::WHITE);
                stack.pop();
            }
            return true;
        }
        if ((stack.size() - 1 == depth && top.first != goal_nh) ||
            (top.second == graph[top.first].end())) {
            graph.setNodeColor(top.first, Color::WHITE);
            stack.pop();
        } else {
            node_handle tg = graph.getOther(*top.second, top.first);
            if (graph.getNodeColor(tg) == Color::WHITE)
                stack.emplace(tg, graph[tg].begin());
            ++top.second;
        }
    }
    return false;
}

} // namespace graphlib

#endif

#ifndef GRAPH_ALGORITHMS
#define GRAPH_ALGORITHMS
#include <type_traits>
#include "BinaryHeap.h"
#include "GraphTraits.h"
#include "ListGraph.h"
#include "ListDiGraph.h"
#include "Matrix.h"

/********* GRAPH ALGORITHMS ****/
/********************************
 ***** TRAVERSAL ALGORITHMS *****
 *******************************/
namespace graphlib {

template<typename Graph>
std::enable_if_t<Graph::traversableTag>
dfs(Graph& graph);

template<typename Graph>
std::enable_if_t<Graph::traversableTag>
bfs(Graph& graph,
    const typename graph_traits<Graph>::node_handle &source);

/* Iterative deepening DFS - function traverses graph to depth
 * max_depth and returns true, if it finds goal_nh */
template<typename Graph>
std::enable_if_t<Graph::traversableTag, bool>
iddfs(Graph& graph,
      const typename graph_traits<Graph>::node_handle &root_nh,
      const typename graph_traits<Graph>::node_handle &goal_nh,
      std::size_t max_depth);

template<typename Graph,
         typename Heuristic,
         typename PriorityQueue = BinHeap<Graph, LessHeuristic<Graph>>>
std::enable_if_t<Graph::heuristicpathTag && Graph::weightedTag>
aStar(Graph& graph,
      const typename graph_traits<Graph>::node_handle &source,
      const typename graph_traits<Graph>::node_handle &target,
      const Heuristic& heuristic);


/********************************
 *** SHORTEST-PATH ALGORITHMS ***
 *******************************/

/* Dijkstra implementation for both directed and undirected
 * graphs. When Graph is undirected, its edges are treated
 * as two directed and called relax function reflects this.
 * Returned bool value signifies, whether graph contained
 * negative edge, iow. whether dijkstra call was successful */
template<typename Graph,
         typename PriorityQueue = BinHeap<Graph>>
std::enable_if_t<Graph::weightedTag &&
                 Graph::pathTag, bool>
dijkstra(Graph& graph,
         const typename graph_traits<Graph>::node_handle &source);

/* Bellman-Ford algorithm for directed weighted graphs. Returns
 * true if provided Graph argument didn't contain any negative
 * weight cycle, iow. whether it successfully finished */
template<typename Graph>
std::enable_if_t<Graph::directedTag &&
                 Graph::weightedTag &&
                 Graph::pathTag, bool>
bellmanFord(Graph& graph,
            const typename graph_traits<Graph>::node_handle &source);

/* Bellman-Ford algorithm that works on undirected graphs
 * by detecting negative edges, that trivially comprise
 * negative loops. It returns true if provided Graph instance
 * didn't contain negative edge and finished successfully */
template<typename Graph>
std::enable_if_t<!Graph::directedTag &&
                  Graph::weightedTag &&
                  Graph::pathTag, bool>
bellmanFord(Graph& graph,
            const typename graph_traits<Graph>::node_handle &source);

/* DAG - find shortest paths from source node in directed
 * acyclic graph. The return value of type bool signifies
 * whether it finished successfully */
template<typename Graph>
std::enable_if_t<Graph::directedTag &&
                 Graph::weightedTag &&
                 Graph::pathTag, bool>
dag(Graph& graph,
    const typename graph_traits<Graph>::node_handle &source);

/* Floyd-Warshall - computes all-pairs shortest-paths matrix
 * for directed weighted graphs */
template<typename Graph>
std::enable_if_t<Graph::directedTag &&
                 Graph::weightedTag &&
                 Graph::pathTag, Matrix<typename graph_traits<Graph>::distance_type>>
floydWarshall(const Graph& graph);

/* Johnson - computes all-pairs shortest-paths matrix for
 * directed weighted graphs */
template<typename Graph>
std::enable_if_t<Graph::directedTag &&
                 Graph::weightedTag &&
                 Graph::pathTag, Matrix<typename graph_traits<Graph>::distance_type>>
johnson(Graph& graph);

}
#endif

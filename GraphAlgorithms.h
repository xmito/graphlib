#ifndef GRAPH_ALGORITHMS
#define GRAPH_ALGORITHMS
#include <type_traits>
#include "BinaryHeap.h"
#include "GraphTraits.h"
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
std::enable_if_t<Graph::traversableTag || Graph::pathTag>
bfs(Graph& graph,
    const typename graph_traits<Graph>::node_handle &source);

template<typename Graph>
std::enable_if_t<Graph::traversableTag, bool>
iddfs(Graph& graph,
      const typename graph_traits<Graph>::node_handle &root_nh,
      const typename graph_traits<Graph>::node_handle &goal_nh,
      std::size_t max_depth);

template<typename Graph,
		 typename Heuristic,
         typename PriorityQueue = BinHeap<typename graph_traits<Graph>::node_handle,
                                          CustomComparator<Graph>>>
void aStar(Graph& graph,
           const typename graph_traits<Graph>::node_handle &source,
           const typename graph_traits<Graph>::node_handle &target,
		   const Heuristic& heuristic);


/********************************
 *** SHORTEST-PATH ALGORITHMS ***
 *******************************/

/* Dijkstra for directed weighted graphs with no negative edges */
template<typename Graph,
         typename PriorityQueue = BinHeap<typename graph_traits<Graph>::node_handle,
                                          CustomComparator<Graph>>>
std::enable_if_t<Graph::directedTag &&
                 Graph::weighted &&
                 Graph::pathTag>
dijkstra(Graph& graph,
         const typename graph_traits<Graph>::node_handle &source);

// Bellman-Ford algorithm for directed weighted graphs
template<typename Graph>
std::enable_if_t<Graph::directedTag &&
                 Graph::weighted &&
                 Graph::pathTag, bool>
bellmanFord(Graph& graph,
            const typename graph_traits<Graph>::node_handle &source);

/* Bellman-Ford algorithm that works on undirected
 * graphs by detecting negative edges, that trivially
 * comprise negative loops*/
template<typename Graph>
std::enable_if_t<!Graph::directedTag &&
                  Graph::weighted &&
                  Graph::pathTag, bool>
bellmanFord(Graph& graph,
            const typename graph_traits<Graph>::node_handle &source);

// DAG - finds shortest path in directed weighted acyclic graph
template<typename Graph>
std::enable_if_t<Graph::directedTag &&
                 Graph::weighted &&
                 Graph::pathTag>
dag(Graph& graph);

/* Assumes directed weighted graphs with no negative weight cycles*/
template<typename Graph>
std::enable_if_t<Graph::directedTag &&
                 Graph::weighted &&
                 Graph::pathTag, Matrix>
floydWarshall(const Graph& graph);

template<typename Graph>
std::enable_if_t<Graph::directedTag &&
                 Graph::weighted &&
                 Graph::pathTag, Matrix>
johnson(Graph& graph);

}
#endif

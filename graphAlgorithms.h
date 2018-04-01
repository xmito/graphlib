#ifndef GRAPHALGORITHMS
#define GRAPHALGORITHMS
#include <type_traits>
#include "binaryHeap.h"

/********* GRAPH ALGORITHMS ****/
/********************************
 ***** TRAVERSAL ALGORITHMS *****
 *******************************/

template<typename Graph>
void dfs(Graph& graph);

template<typename Graph>
void bfs(Graph& graph, typename Graph::node_id source);

template<typename Graph>
bool iterativeDeepeningDFS(Graph& graph,
						typename Graph::node_id source,
						typename Graph::node_id target,
						unsigned int maxdepth);

template<typename Graph,
		 typename Heuristic,
		 typename PriorityQueue = BinHeap<typename Graph::node_id>>
void AStar(Graph& graph,
		   typename Graph::node_id source,
		   typename Graph::node_id target,
		   const Heuristic& heuristic);


/********************************
 *** SHORTEST-PATH ALGORITHMS ***
 *******************************/

/* Dijkstra for directed weighted graphs with no negative edges */
template<typename Graph,
		 typename PriorityQueue = BinHeap<typename Graph::node_id>>
std::enable_if_t<Graph::directedTag && Graph::weighted>
dijkstra(Graph& graph, typename Graph::node_id source);

// Bellman-Ford algorithm for directed weighted graphs
template<typename Graph>
std::enable_if_t<Graph::directedTag && Graph::weighted>
bellmanFord(Graph& graph);

/* Bellman-Ford algorithm that works on undirected
 * graphs by detecting negative edges, that trivially
 * comprise negative loops*/
template<typename Graph>
std::enable_if_t<!Graph::directedTag && Graph::weighted>
bellmanFord(Graph& graph);

// DAG - finds shortest path in directed weighted acyclic graph
template<typename Graph>
std::enable_if_t<Graph::directedTag && Graph::weighted>
dag(Graph& graph);

/* Assumes directed weighted graphs with no negative weight cycles*/
template<typename Graph,
		 typename Matrix>
std::enable_if_t<Graph::directedTag && Graph::weighted, Matrix>
floydWarshall(const Graph& graph);

template<typename Graph,
		 typename Matrix>
std::enable_if_t<Graph::directedTag && Graph::weighted, Matrix>
johnson(Graph& graph);
#endif

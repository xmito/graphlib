/* TODO
 *
 * Vertex comparison
 * graph comparison?
 *
 *
 *
 *
 *
 */*
#include <vector>
#include <list>
#include <algorithm>

enum class Color {WHITE, GRAY, BLACK};

template < typename Graph >
void DFS ( Graph graph, typename Graph::Node node );
template < typename Graph >
void BFS ( Graph graph, typename Graph::Node node );
template < typename Graph >
void IDDFS ( Graph graph, typename Graph::Node node );
//template < typename Graph >
//Dijkstra
//template < typename Graph >
//AStar
//template < typename Graph >
//Bellman-Ford
//template < typename Graph >
//DAG
//template < typename Graph >
//Floyd-Warshall
//template < typename Graph >
//Johnson


template < typename Identificator >
struct AdjacencyListGraph {

	struct Vertex {
		Identificator id_;
		std::list< std::pair< Vertex*, int > > adjList;

		void addEdge(Vertex *vertex, int dist) {
			for (auto& vertPair : adjList) {
				if (vertPair.first == vertex) {
					if (vertPair.second > dist) {
						vertPair.second = dist;
					}
					return;
				}
			}
			adjList.push_back(std::pair(vertex, dist));
		}

		Vertex() = default;
		Vertex(Identificator id) : id_(id) {}

	};
	std::vector< Vertex > vertices_;

	AdjacencyListGraph() = default;
	AdjacencyListGraph(const AdjacencyListGraph& graph)
		: vertices_(graph.vertices_) {}
	
	AdjacencyListGraph& operator=(const AdjacencyListGraph& graph) {
		if (this == &graph)
			return *this;
		vertices_ = graph.vertices_;
		return *this;
	}

	AdjacencyListGraph& operator=(AdjacencyListGraph&& graph) {
		if (this == &graph)
			return *this;
		vertices_ = std::move(graph.vertices_);
		return *this;
	}

	void addVertex( Vertex v ) {
		//auto findVertex = std::find(vertices_.begin(), vertices_.end(), v);
		//if (findVertex == vertices_.end())
		vertices_.push_back(v);
	}
};

struct MatrixGraph {
	struct Node {
		
	};
};

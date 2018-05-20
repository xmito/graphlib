#include <iostream>
#include "interface.hpp"

template < typename Identificator >
void printVertexInfo(typename AdjacencyListGraph<Identificator>::Vertex v) {
	std::cout << "Vertex " << v.id_ << std::endl;
	std::cout << "neighbours: " << std::endl;
	for (auto vert : v.adjList) {
		std::cout << "\tVertex: " << vert.first->id_ << " distance: " << vert.second << std::endl;
	}
}

int main() {
	AdjacencyListGraph<int> g;
	AdjacencyListGraph<int> g1;
	AdjacencyListGraph<int>::Vertex v1(1);
	AdjacencyListGraph<int>::Vertex v2(2);
	AdjacencyListGraph<int>::Vertex v3(3);

	v1.addEdge(&v2, 5);
	v1.addEdge(&v3, 42);
	printVertexInfo<int>(v1);

	v2.addEdge(&v1, 7);
	v2.addEdge(&v1, 3);
	printVertexInfo<int>(v2);

	g.addVertex(v1);
	printVertexInfo<int>(g.vertices_[0]);
}

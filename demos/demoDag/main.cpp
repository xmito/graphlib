#include <iostream>
#include "graphlib.h"

int main() {
	using namespace graphlib;
	using Graph = ListDiGraph<PathNodeData, WeightedEdgeData>;
	using node_handle = typename graph_traits<Graph>::node_handle;
	Graph graph;
	node_handle handles[6];
	for (int i = 0; i < 6; ++i)
		handles[i] = graph.addNode();

	graph.addEdge(handles[0], handles[1], 3);
	graph.addEdge(handles[0], handles[2], 8);
	graph.addEdge(handles[0], handles[4], 4);
	graph.addEdge(handles[2], handles[1], 4);
	graph.addEdge(handles[3], handles[2], 6);
	graph.addEdge(handles[4], handles[3], 6);
	graph.addEdge(handles[2], handles[5], 4);
	graph.addEdge(handles[3], handles[5], 1);

	exportGraph(graph, "./graph.dot");
	dag(graph, handles[0]);
	exportShortestPath(graph, handles[5], "./graphsh.dot");
	return 0;
}

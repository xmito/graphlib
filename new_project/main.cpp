#include <iostream>
#include <graphlib.h>

int main()
{
	graphlib::Matrix<int> mtx;
	std::cout << mtx.empty() << std::endl;

	using namespace graphlib;
	using graph_type = ListDiGraph<PathNodeData, WeightedEdgeData>;
	using node_handle = typename graph_type::node_handle;
	using distance_type = typename graph_traits<graph_type>::distance_type;

	ListDiGraph<PathNodeData, WeightedEdgeData> graph;

	node_handle handles[6];
	for (size_t i = 0; i < 6; ++i) {
		handles[i] = graph.addNode();
	}


	std::cout << "do sth" << std::endl;
	return 0;
}

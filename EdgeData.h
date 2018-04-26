#ifndef EDGE_DATA_H
#define EDGE_DATA_H

namespace graphlib {

struct EdgeData {
	static constexpr bool weighted = false;
};

struct WeightedEdgeData {
	static constexpr bool weighted = true;
	using weight_type = long int;
	long int weight_;

	WeightedEdgeData() : weight_(1) {}
	WeightedEdgeData(long int weight) : weight_(weight) {}
};

}

#endif

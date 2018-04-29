#ifndef EDGE_DATA_H
#define EDGE_DATA_H

namespace graphlib {

struct EdgeData {
	static constexpr bool weighted = false;
private:
	using weight_type = void;
	template<typename>
	friend struct edge_traits;
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

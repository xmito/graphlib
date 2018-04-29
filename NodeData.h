#ifndef NODE_DATA_H
#define NODE_DATA_H
#include "Handle.h"
#include <utility>

namespace graphlib {

enum class Color : char {WHITE, GRAY, BLACK};

struct DefaultNodeData {
	static constexpr bool distance = false;
	static constexpr bool predecessor = false;
	static constexpr bool color = false;
	static constexpr bool location = false;
private:
	using distance_type = void;
	template<typename>
	friend struct node_traits;
};

struct TraversableNodeData {
	static constexpr bool distance = false;
	static constexpr bool predecessor = true;
	static constexpr bool color = true;
	static constexpr bool location = false;

	Color color_;
	NodeHandle pred_;

	TraversableNodeData() : color_(Color::WHITE) {}
	TraversableNodeData(Color color, const NodeHandle& pred) : color_(color), pred_(pred) {}
private:
	using distance_type = void;
	template<typename>
	friend struct node_traits;
};

struct PathNodeData {
	static constexpr bool distance = true;
	static constexpr bool predecessor = true;
	static constexpr bool color = true;
	static constexpr bool location = false;
	using distance_type = long int;

	Color color_;
	NodeHandle pred_;
	long int dist_;

	PathNodeData() : color_(Color::WHITE), dist_(0) {}
	PathNodeData(Color color, const NodeHandle& pred, long int dist = 0) : color_(color), pred_(pred), dist_(dist) {}
};

template<typename Location>
struct LocationNodeData {
	static constexpr bool distance = true;
	static constexpr bool predecessor = true;
	static constexpr bool color = true;
	static constexpr bool location = true;
	using distance_type = long int;

	Color color_;
	NodeHandle pred_;
	long int dist_;
	Location loc_;

	LocationNodeData() : color_(Color::WHITE), dist_(0) {}
	template<typename... Args>
	LocationNodeData(Args&&... args) : color_(Color::WHITE), dist_(0), loc_(std::forward<Args>(args)...) {}
};

}
#endif

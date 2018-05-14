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
	static constexpr bool priority = false;
private:
	using distance_type = void;
	using location_type = void;
	using priority_type = void;
	template<typename>
	friend struct node_traits;
};

struct TraversableNodeData {
	static constexpr bool distance = false;
	static constexpr bool predecessor = true;
	static constexpr bool color = true;
	static constexpr bool location = false;
	static constexpr bool priority = false;

	Color color_;
	NodeHandle pred_;

	TraversableNodeData() : color_(Color::WHITE) {}
	TraversableNodeData(Color color, const NodeHandle& pred) :
	    color_(color), pred_(pred) {}
private:
	using distance_type = void;
	using location_type = void;
	using priority_type = void;
	template<typename>
	friend struct node_traits;
};

struct PathNodeData {
	static constexpr bool distance = true;
	static constexpr bool predecessor = true;
	static constexpr bool color = true;
	static constexpr bool location = false;
	static constexpr bool priority = false;
	using distance_type = long int;

	Color color_;
	NodeHandle pred_;
	long int dist_;

	PathNodeData() : color_(Color::WHITE), dist_(0) {}
	PathNodeData(Color color, const NodeHandle& pred, long int dist = 0) :
	    color_(color), pred_(pred), dist_(dist) {}
private:
	using location_type = void;
	using priority_type = void;
	template<typename>
	friend struct node_traits;
};

template<typename Location>
struct LocationNodeData {
	static constexpr bool distance = true;
	static constexpr bool predecessor = true;
	static constexpr bool color = true;
	static constexpr bool location = true;
	static constexpr bool priority = true;
	using distance_type = long int;
	using location_type = Location;
	using priority_type = double;

	distance_type dist_;
	priority_type prio_;
	Color color_;
	NodeHandle pred_;
	Location loc_;

	LocationNodeData() : dist_(0), prio_(0), color_(Color::WHITE) {}
	template<typename... Args>
	LocationNodeData(Args&&... args) : dist_(0), prio_(0),
	    color_(Color::WHITE), loc_(std::forward<Args>(args)...) {}
};

struct PlaneLocation {
	long int x_;
	long int y_;

	PlaneLocation() : x_(0), y_(0) {}
	PlaneLocation(long int x, long int y) : x_(x), y_(y) {}
	bool operator==(const PlaneLocation &loc) const {
		return x_ == loc.x_ && y_ == loc.y_;
	}
	bool operator!=(const PlaneLocation &loc) const {
		return !(*this == loc);
	}
};

}
#endif

#ifndef NODE_DATA_H
#define NODE_DATA_H
#include "Handle.h"
#include <utility>
#include <cstdint>

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

	Color color_{Color::WHITE};
	NodeHandle pred_;

	TraversableNodeData() = default;
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
	using distance_type = int64_t;

	Color color_{Color::WHITE};
	NodeHandle pred_;
	int64_t dist_{0};

	PathNodeData() = default;
	PathNodeData(Color color, const NodeHandle& pred, int64_t dist = 0) :
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
	using distance_type = int64_t;
	using location_type = Location;
	using priority_type = double;

	distance_type dist_{0};
	priority_type prio_{0};
	Color color_{Color::WHITE};
	NodeHandle pred_;
	Location loc_;

	LocationNodeData() = default;
	template<typename... Args>
	explicit LocationNodeData(Args&&... args) :
	    loc_(std::forward<Args>(args)...) {}
};

struct PlaneLocation {
	int64_t x_{0};
	int64_t y_{0};

	PlaneLocation() = default;
	PlaneLocation(int64_t x, int64_t y) : x_(x), y_(y) {}
	bool operator==(const PlaneLocation &loc) const {
		return x_ == loc.x_ && y_ == loc.y_;
	}
	bool operator!=(const PlaneLocation &loc) const {
		return !(*this == loc);
	}
};

} // namespace graphlib
#endif

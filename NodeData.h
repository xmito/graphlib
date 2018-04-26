#ifndef NODE_DATA_H
#define NODE_DATA_H

namespace graphlib {

enum class Color : char {WHITE, GRAY, BLACK};

struct DefaultNodeData {
	static constexpr bool distance = false;
	static constexpr bool predecessor = false;
	static constexpr bool color = false;
	static constexpr bool location = false;
};

struct TraversableNodeData {
	static constexpr bool distance = false;
	static constexpr bool predecessor = true;
	static constexpr bool color = true;
	static constexpr bool location = false;

	Color color_;
	size_t pred_;

	TraversableNodeData() : color_(Color::WHITE), pred_(static_cast<size_t>(-1)) {}
	TraversableNodeData(Color color, size_t pred) : color_(color), pred_(pred) {}
};

struct PathNodeData {
	static constexpr bool distance = true;
	static constexpr bool predecessor = true;
	static constexpr bool color = true;
	static constexpr bool location = false;

	Color color_;
	size_t pred_;
	long int dist_;

	PathNodeData() : color_(Color::WHITE), pred_(static_cast<size_t>(-1)), dist_(0) {}
	PathNodeData(Color color, size_t pred, long int dist = 0) : color_(color), pred_(pred), dist_(dist) {}
};

template<typename Location>
struct LocationNodeData {
	static constexpr bool distance = true;
	static constexpr bool predecessor = true;
	static constexpr bool color = true;
	static constexpr bool location = true;

	Color color_;
	size_t pred_;
	long int dist_;
	Location loc_;

	LocationNodeData() : color_(Color::WHITE), pred_(static_cast<size_t>(-1)), dist_(0), loc_() {}
	template<typename... Args>
	LocationNodeData(Args&&... args) : color_(Color::WHITE), pred_(static_cast<size_t>(-1)),
	    dist_(0), loc_(std::forward<Args>(args)...) {}
};

}
#endif

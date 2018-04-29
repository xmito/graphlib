#ifndef NODE_TRAITS_H
#define NODE_TRAITS_H

namespace graphlib {

template<typename Node>
struct node_traits {
	static constexpr bool distance = Node::distance;
	static constexpr bool predecessor = Node::predecessor;
	static constexpr bool color = Node::color;
	static constexpr bool location = Node::location;
	using distance_type = std::conditional_t<Node::distance, typename Node::distance_type, void>;
};

}
#endif

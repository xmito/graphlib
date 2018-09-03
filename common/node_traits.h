#ifndef NODE_TRAITS_H
#define NODE_TRAITS_H

namespace graphlib {

/**
 * @brief node_traits is a data structure, that helps to retrieve traits of node template type argument
 * @tparam Node Type of node
 */
template <typename Node>
struct node_traits {
    static constexpr bool distance = Node::distance;
    static constexpr bool predecessor = Node::predecessor;
    static constexpr bool color = Node::color;
    static constexpr bool location = Node::location;
    using distance_type = typename Node::distance_type;
    using location_type = typename Node::location_type;
    using priority_type = typename Node::priority_type;
};

} // namespace graphlib

#endif

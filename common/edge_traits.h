#ifndef EDGE_TRAITS_H
#define EDGE_TRAITS_H

namespace graphlib {

/**
 * @brief edge_traits is a data structure, that helps to retrieve traits of some edge template type argument
 * @tparam Edge Type of edge
 */
template <typename Edge>
struct edge_traits {
    static constexpr bool weighted = Edge::weighted;
    using weight_type = typename Edge::weight_type;
};

} // namespace graphlib

#endif

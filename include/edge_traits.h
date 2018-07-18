#ifndef EDGE_TRAITS_H
#define EDGE_TRAITS_H

namespace graphlib {

template <typename Edge>
struct edge_traits {
    static constexpr bool weighted = Edge::weighted;
    using weight_type = typename Edge::weight_type;
};

} // namespace graphlib

#endif

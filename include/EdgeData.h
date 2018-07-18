#ifndef EDGE_DATA_H
#define EDGE_DATA_H

#include <cstdint>

namespace graphlib {

struct EdgeData {
    static constexpr bool weighted = false;

  private:
    using weight_type = void;
    template <typename>
    friend struct edge_traits;
};

struct WeightedEdgeData {
    static constexpr bool weighted = true;
    using weight_type = int64_t;
    int64_t weight_{1};

    WeightedEdgeData() = default;
    explicit WeightedEdgeData(int64_t weight) : weight_(weight) {}
};

} // namespace graphlib

#endif

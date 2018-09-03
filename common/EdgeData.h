#ifndef EDGE_DATA_H
#define EDGE_DATA_H

#include <cstdint>

namespace graphlib {

/**
 * @brief EdgeData is data structure, that has no member variables. It suitable for graph types, that do not need to store any data with edges
 */
struct EdgeData {
    /** EdgeData structure is not weighted, it doesn't fulfill weightedTag when used in graph */
    static constexpr bool weighted = false;

  private:
    using weight_type = void;
    template <typename>
    friend struct edge_traits;
};

/**
 * @brief WeightedEdgeData is a data structure, that fulfill weightedTag. It means, that it has member variable to store edge weight
 */
struct WeightedEdgeData {
    /** EdgeData is weighted and therefore it fulfills weightedTag when used in graph */
    static constexpr bool weighted = true;
    using weight_type = int64_t;
    int64_t weight_{1};

    /**
     * @brief Default constructor. Constructs WeightedEdgeData instance with default values
     */
    WeightedEdgeData() = default;
    /**
     * @brief Constructs WeightedEdgeData structure with provided edge weight
     * @param weight Weight to store in the new instance
     */
    explicit WeightedEdgeData(int64_t weight) : weight_(weight) {}
};

} // namespace graphlib

#endif

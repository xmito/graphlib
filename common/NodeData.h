#ifndef NODE_DATA_H
#define NODE_DATA_H
#include "Handle.h"
#include <utility>
#include <cstdint>

namespace graphlib {

/**
 * @brief Enum Color defines three different states of nodes, when traversing graph. WHITE color signifies, that node has not been visited. GRAY color signifies, that node has been visited, but still not processed. BLACK color on the other hand marks nodes, that are processed.
 */
enum class Color : char {
    WHITE,
    GRAY,
    BLACK
};

/**
 * @brief DefaultNodeData is a data structure, that has no member variables. It is suitable for graph types, that do not need to store any data with nodes.
 */
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
    template <typename>
    friend struct node_traits;
};

/**
 * @brief TraversableNodeData is a data structure, that fulfills traversableTag. That is, it has member variables to store predecessor and color
 */
struct TraversableNodeData {
    static constexpr bool distance = false;
    static constexpr bool predecessor = true;
    static constexpr bool color = true;
    static constexpr bool location = false;
    static constexpr bool priority = false;

    Color color_{Color::WHITE};
    NodeHandle pred_;

    TraversableNodeData() = default;
    TraversableNodeData(Color color, const NodeHandle &pred)
        : color_(color), pred_(pred) {}

  private:
    using distance_type = void;
    using location_type = void;
    using priority_type = void;
    template <typename>
    friend struct node_traits;
};

/**
 * @brief PathNodeData is a data structure, that fulfills pathTag. That is, it has member variables to store predecessor, color and distance values
 */
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

    /**
     * @brief Default constructor. Constructs PathNodeData instance with default values
     */
    PathNodeData() = default;
    /**
     * @brief Constructs PathNodeData instance from color, predecessor and distance values
     * @param color Color to be stored in node data
     * @param pred Predecessor to be stored in node data
     * @param dist Distance to be stored in node data. Default function argument value is zero
     */
    PathNodeData(Color color, const NodeHandle &pred, int64_t dist = 0)
        : color_(color), pred_(pred), dist_(dist) {}

  private:
    using location_type = void;
    using priority_type = void;
    template <typename>
    friend struct node_traits;
};

/**
 * @brief LocationNodeData is a data structure, that fulfills heuristicpathTag. That is, it has all member variables as PathNodeData and priority and location used for heuristic algorithms
 */
template <typename Location>
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

    /**
     * @brief Default constructor. Constructs LocationNodeData instance with default values
     */
    LocationNodeData() = default;
    /**
     * @brief Constructs LocationNodeData instance from arguments, that are forwarded to location member variable constructor
     * @tparam Args Types of arguments forwarded to location member variable constructor
     * @param args Arguments passed to location member variable constructor
     */
    template <typename... Args>
    explicit LocationNodeData(Args &&...args)
        : loc_(std::forward<Args>(args)...) {}
};

/**
 * @brief PlaneLocation is a data structure, that stores two dimensional coordinates. This type of location can be used with AStar algorithm.
 */
struct PlaneLocation {
    int64_t x_{0};
    int64_t y_{0};

    /**
     * @brief Default constructor. Constructs PlaneLocation instance with default values
     */
    PlaneLocation() = default;
    /**
     * @brief Constructs PlaneLocation instance from two coordinates x and y
     * @param x Coordinate of the first dimension
     * @param y Coordinate of the second dimension
     */
    PlaneLocation(int64_t x, int64_t y) : x_(x), y_(y) {}
    /**
     * @brief Compares *this with other PlaneLocation instance for equality
     * @param loc PlaneLocation instance to compare to
     * @return bool value, that signifies equality
     */
    bool operator==(const PlaneLocation &loc) const {
        return x_ == loc.x_ && y_ == loc.y_;
    }
    /**
     * @brief Compares *this with other PlaneLocation instance for inequality
     * @param loc PlaneLocation instance to compare to
     * @return bool value, that signifies inequality
     */
    bool operator!=(const PlaneLocation &loc) const { return !(*this == loc); }
};

} // namespace graphlib
#endif

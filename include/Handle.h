#ifndef HANDLE_H
#define HANDLE_H

#include <cstddef>

namespace graphlib {

template <typename IdType>
struct Handle {
    using id_type = IdType;
    Handle() = default;
    bool operator==(const Handle &handle) const { return id_ == handle.id_; }
    bool operator!=(const Handle &handle) const { return id_ != handle.id_; }
    bool operator<(const Handle &handle) const { return *id_ < *handle.id_; }
    bool operator>(const Handle &handle) const { return *id_ > *handle.id_; }
    bool operator<=(const Handle &handle) const { return !(*this > handle); }
    bool operator>=(const Handle &handle) const { return !(*this < handle); }
    id_type getId() const { return *id_; }

  protected:
    explicit Handle(id_type *id) : id_(id) {}
    id_type *id_{nullptr};
};

using node_id = std::size_t;
using edge_id = std::size_t;

struct NodeHandle : public Handle<node_id> {
    NodeHandle() = default;
    explicit NodeHandle(node_id *id) : Handle(id) {}
    template <typename, typename>
    friend class ListDiGraph;
    template <typename, typename>
    friend class ListGraph;
};

struct EdgeHandle : public Handle<edge_id> {
    EdgeHandle() = default;
    explicit EdgeHandle(node_id *id) : Handle(id) {}
    template <typename, typename>
    friend class ListDiGraph;
    template <typename, typename>
    friend class ListGraph;
};

} // namespace graphlib

#endif

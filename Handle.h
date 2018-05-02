#ifndef HANDLE_H
#define HANDLE_H
#include "Matrix.h"
#include "GraphTraits.h"

namespace graphlib {

template<typename IdType>
struct Handle {
	using id_type = IdType;
	Handle() : id_(nullptr) {}
	bool operator==(const Handle& handle) const {
		return id_ == handle.id_;
	}
	bool operator!=(const Handle& handle) const {
		return id_ != handle.id_;
	}
	id_type getId() const {
		return *id_;
	}
private:
	template<typename, typename>
	friend class ListDiGraph;
	template<typename, typename>
	friend class ListGraph;
	Handle(id_type *id) : id_(id) {}
	id_type *id_;
};

using node_id = std::size_t;
using edge_id = std::size_t;
using NodeHandle = Handle<node_id>;
using EdgeHandle = Handle<edge_id>;

}

#endif

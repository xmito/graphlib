#ifndef HANDLE_H
#define HANDLE_H

namespace graphlib {

template<typename IdType>
struct Handle {
	Handle() : id_(nullptr) {}
	bool operator==(const Handle& handle) const {
		return id_ == handle.id_;
	}
	bool operator!=(const Handle& handle) const {
		return id_ != handle.id_;
	}
	IdType *id_;
private:
	template<typename, typename>
	friend class ListDiGraph;
	Handle(IdType *id) : id_(id) {}
};

using node_id = size_t;
using edge_id = size_t;
using NodeHandle = Handle<node_id>;
using EdgeHandle = Handle<edge_id>;

}

#endif

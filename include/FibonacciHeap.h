#ifndef FIBONACCIHEAP_H
#define FIBONACCIHEAP_H
#include <cstddef>
#include <memory>
#include <cassert>
#include <iostream>
#include <functional>
#include <iterator>
#include <unordered_map>
#include "CircularList.h"
#include "GraphTraits.h"
#include "Comparators.h"

namespace graphlib {

template<typename Graph,
         typename Compare = LessDistance<Graph>>
struct FibonacciHeap {
	struct Node;
	using value_type = typename graph_traits<Graph>::node_handle;
	using value_compare = Compare;
	using reference = typename graph_traits<Graph>::node_handle&;
	using const_reference = const typename graph_traits<Graph>::node_handle&;
	using size_type = std::size_t;
	using difference_type = typename CircularList<Node>::iterator::difference_type;

private:
	using iterator = typename CircularList<Node>::iterator;
	using const_iterator = typename CircularList<Node>::const_iterator;
	using list_container = CircularList<Node>;
	using node_handle_id = typename graph_traits<Graph>::node_handle::id_type;

public:
	struct Node {
		bool mark_;
		value_type key_;
		iterator parent_;
		list_container clist_;

		Node(const value_type& key) :
		    mark_(false), key_(key), parent_(nullptr) {}
		Node(const value_type& key, const_iterator parent) :
		    mark_(false), key_(key), parent_(parent) {}
		Node(const Node& node) : mark_(node.mark_),
		    key_(node.key_), parent_(node.parent_), clist_(node.clist_) {}
		Node(Node&& node) : mark_(node.mark_), key_(std::move(node.key_)),
		    parent_(node.parent_), clist_(std::move(node.clist_)) {}
		bool operator==(const Node& node) const {
			return key_ == node.key_;
		}
		bool operator!=(const Node& node) const {
			return !(*this == node);
		}
	};

	/* Constructors */
	template<typename InputIt>
	FibonacciHeap(InputIt first, InputIt last, const Compare& comp) :
	    nodes_(0), comp_(comp) {
		while (first != last)
			push(*first++);
	}
	FibonacciHeap(std::initializer_list<value_type> ilist, const Compare &comp) :
	    FibonacciHeap(ilist.begin(), ilist.end(), comp) {}
	FibonacciHeap(const Compare& comp) : nodes_(0), comp_(comp) {}
	FibonacciHeap(const Graph& graph) :
	    FibonacciHeap(graph.beginNode(), graph.endNode(), Compare(&graph)) {}
	FibonacciHeap(const Graph& graph, const Compare &comp) :
	    FibonacciHeap(graph.beginNode(), graph.endNode(), comp) {}
	FibonacciHeap(const FibonacciHeap& fh) :
	    nodes_(fh.nodes_), rlist_(fh.rlist_), comp_(fh.comp_), map_(fh.map_) {
		auto diff = std::distance(fh.rlist_.begin(), const_iterator(fh.top_));
		top_ = std::next(rlist_.begin(), diff);
	}
	FibonacciHeap& operator=(const FibonacciHeap& fh) {
		if (this == &fh)
			return *this;
		nodes_ = fh.nodes_;
		comp_ = fh.comp_;
		rlist_ = fh.rlist_;
		map_ = fh.map_;

		auto diff = std::distance(fh.rlist_.begin(), const_iterator(fh.top_));
		top_ = std::next(rlist_.begin(), diff);
		return *this;
	}
	FibonacciHeap(FibonacciHeap&& fh) : nodes_(fh.nodes_), top_(std::move(fh.top_)),
	    rlist_(std::move(fh.rlist_)), comp_(std::move(fh.comp_)),
	    map_(std::move(fh.map_)) {
		fh.nodes_ = 0;
		fh.top_ = nullptr;
	}
	FibonacciHeap& operator=(FibonacciHeap&& fh) {
		if (this == &fh)
			return *this;
		nodes_ = fh.nodes_;
		comp_ = std::move(fh.comp_);
		rlist_ = std::move(fh.rlist_);
		map_ = std::move(fh.map_);
		top_ = std::move(fh.top_);

		fh.nodes_ = 0;
		fh.top_ = nullptr;
		return *this;
	}

	/* Lookup methods */
	bool empty() const {
		return nodes_ == 0;
	}
	size_t size() const {
		return nodes_;
	}
	const_reference top() const {
		assert(top_);
		return const_iterator(top_)->key_;
	}

	/* Modifiers */
	void push(const value_type& nh) {
		iterator node = rlist_.emplace_back(nh);
		if (!top_ || comp_(node->key_, top_->key_))
			top_ = node;
		++nodes_;
		map_[nh.getId()] = node;
	}
	void pop() {
		value_type nh = top_->key_;

		insertChildren_(top_);
		map_.erase(nh.getId());
		rlist_.erase(top_);
		--nodes_;
		if (rlist_.empty())
			top_ = nullptr;
		else
			consolidate_();
	}
	/* decUpdate - method is used to restore heap property
	 * after some value of graph node has been decreased */
	void decUpdate(const value_type& nh) {
		iterator node = map_[nh.getId()];
		iterator parent = node->parent_;
		if (parent && comp_(node->key_, parent->key_)) {
			cut_(node);
			cascading_cut_(parent);
		}
		if (comp_(node->key_, top_->key_))
			top_ = node;
	}

private:
	std::size_t nodes_;
	iterator top_;
	list_container rlist_;
	value_compare comp_;
	std::unordered_map<node_handle_id, iterator> map_;

	/* link_ - method links node to child list of parent.
	 * If node was current top_, it is released */
	void link_(iterator node, iterator parent) {
		assert(node);
		assert(node->parent_ == nullptr);
		assert(parent);
		assert(node != parent);

		node->mark_ = false;
		node->parent_ = parent;
		auto& pclist = parent->clist_;
		pclist.splice(pclist.end(), rlist_, node);
	}

	/* Consolidate all inserted nodes by merging those, that
	 * have same degree. Therefore after consolidate, root list
	 * has nodes, that each have different degree */
	void consolidate_() {
		iterator trees[64]{nullptr};
		iterator bit = rlist_.begin();
		iterator eit = rlist_.end();
		while (bit != eit) {
			size_t degree = bit->clist_.size();
			iterator nnode = bit;
			++nnode;
			while (trees[degree] != nullptr) {
				iterator node = trees[degree];
				if (comp_(node->key_, bit->key_))
					std::swap(bit, node);
				link_(node, bit);
				trees[degree] = nullptr;
				++degree;
			}
			trees[degree] = bit;
			bit = nnode;
		}
		iterator top = nullptr;
		for (size_t i = 0; i < 64; ++i) {
			if (trees[i]) {
				if (!top || comp_(trees[i]->key_, top->key_))
					top = trees[i];
			}
		}
		top_ = top;
	}


	/* unMarkUnParentChild_ - method sets parent_ to nullptr and
	 * its mark_ to false */
	void unMarkUnParentChild_(Node &child) {
		child.parent_ = nullptr;
		child.mark_ = false;
	}

	/* unMarkUnParentChildren_ - method goes through children
	 * list and sets parent_ pointer to nullptr and mark_ to
	 * false */
	void unMarkUnParentChildren_(iterator parent) {
		assert(parent);

		if (!parent->clist_.size())
			return;
		for (auto& node : parent->clist_)
			unMarkUnParentChild_(node);
	}

	/* insertChildren_ - method inserts into root list all
	 * nodes that are present in child list of parent.
	 * The function assumes that FibonacciHeap instance is not
	 * empty and root list contains at least one node. */
	void insertChildren_(iterator parent) {
		assert(parent);

		if (parent->clist_.empty())
			return;
		unMarkUnParentChildren_(parent);
		rlist_.merge(std::move(parent->clist_));
	}

	/* cut_ - method removes a node from child list of
	 * its parent and appends it to the root list */
	void cut_(iterator node) {
		assert(node);
		assert(node->parent_ != nullptr);

		iterator parent = node->parent_;
		unMarkUnParentChild_(*node);
		rlist_.splice(rlist_.end(), parent->clist_, node);
	}

	/* cascading_cut_ - method cuts already marked node and
	 * does cascading cut on its parent, because it may be
	 * marked and eligible to be cut */
	void cascading_cut_(iterator node) {
		iterator parent = node->parent_;
		if (parent != nullptr) {
			if (node->mark_) {
				cut_(node);
				cascading_cut_(parent);
			} else
				parent->mark_ = true;
		}
	}
};

} // namespace graphlib
#endif // FIBONACCIHEAP_H

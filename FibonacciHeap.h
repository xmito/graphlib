#ifndef FIBONACCIHEAP_H
#define FIBONACCIHEAP_H
#include <cstddef>
#include <memory>
#include <cassert>
#include <iostream>
#include <functional>
#include "CircularList.h"

template<typename T,
         typename Compare = std::less<T>>
struct FibHeap {
	struct Node;
	using value_type = T;
	using value_compare = Compare;
	using reference = T&;
	using const_reference = const T&;
	using difference_type = std::ptrdiff_t;
	using size_type = std::size_t;

private:
	using iterator = typename CircularList<Node>::iterator;
	using const_iterator = typename CircularList<Node>::const_iterator;
	using list_container = CircularList<Node>;

public:
	struct Node {
		bool mark_;
		value_type key_;
		const_iterator parent_;
		list_container clist_;

		Node(const T& key) : mark_(false), key_(key), parent_(nullptr) {}
		Node(const T& key, const_iterator parent) : mark_(false), key_(key), parent_(parent) {}
		Node(const Node& node) : mark_(node.mark_), key_(node.key_), parent_(node.parent_), clist_(node.clist_) {}
		Node(Node&& node) : mark_(node.mark_), key_(std::move(node.key_)), parent_(node.parent_), clist_(std::move(node.clist_)) {}
		bool operator<(const Node& node) const {
			return key_ < node.key_;
		}
		bool operator>=(const Node& node) const {
			return !(key_ < node.key_);
		}
		bool operator>(const Node& node) const {
			return key_ > node.key_;
		}
		bool operator <=(const Node& node) const {
			return !(key_ > node.key_);
		}
	};

	FibHeap(const Compare& comp = Compare()) : nodes_(0), comp_(comp) {}
	FibHeap(std::initializer_list<T> ilist, const Compare& comp = Compare()) : FibHeap(ilist.begin(), ilist.end(), comp) {}
	template<typename InputIt>
	FibHeap(InputIt first, InputIt last, const Compare& comp = Compare()) : nodes_(0), comp_(comp) {
		while (first != last)
			push(*first++);
	}
	template<typename Container>
	FibHeap(const Container& cont, const Compare& comp = Compare()) : FibHeap(cont.begin(), cont.end(), comp) {}
	FibHeap(const FibHeap& fh) : nodes_(fh.nodes_), rlist_(fh.rlist_), comp_(fh.comp_) {
		auto bit = fh.rlist_.begin();
		top_ = rlist_.begin();
		while(bit != fh.top_) {
			++bit;
			++top_;
		}
	}
	FibHeap& operator=(const FibHeap& fh) {
		if (this == &fh)
			return *this;
		nodes_ = fh.nodes_;
		comp_ = fh.comp_;
		rlist_ = fh.rlist_;

		auto bit = fh.rlist_.begin();
		top_ = rlist_.begin();
		while (bit != fh.top_) {
			++bit;
			++top_;
		}
		return *this;
	}
	FibHeap(FibHeap&& fh) : nodes_(fh.nodes_),
	                        top_(fh.top_),
	                        rlist_(std::move(fh.rlist_)),
	                        comp_(std::move(fh.comp_)) {}
	FibHeap& operator=(FibHeap&& fh) {
		if (this == &fh)
			return *this;
		nodes_ = fh.nodes_;
		rlist_ = std::move(fh.rlist_);
		top_ = fh.top_;
		comp_ = std::move(fh.comp_);
		return *this;
	}

	bool empty() const {return nodes_ == 0;}
	size_t size() const {return nodes_;}
	const_reference top() const {
		assert(top_);
		return const_iterator(top_)->key_;
	}
	void push(const value_type& key) {
		iterator node = rlist_.emplace_back(key);
		if (!top_ || comp_(node->key_, top_->key_))
			top_ = node;
		++nodes_;
	}
	void push(value_type&& key) {
		iterator node = rlist_.emplace_back(std::move(key));
		if (!top_ || comp_(node->key_, top_->key_))
			top_ = node;
		++nodes_;
	}
	void pop() {removeTop_();}

private:
	std::size_t nodes_;
	iterator top_;
	list_container rlist_;
	value_compare comp_;

	// removeTop - method removes top_ node
	void removeTop_() {
		insertChildren_(top_);
		rlist_.erase(top_);
		--nodes_;
		if (rlist_.empty())
			top_ = nullptr;
		else
			consolidate_();
	}

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
	 * nodes that are present in child list of Node parent.
	 * The function assumes that FibHeap instance is not
	 * empty and root list contains at least one node.
	 * Use case : extractMin */
	void insertChildren_(iterator parent) {
		assert(parent);
		if (parent->clist_.empty())
			return;
		unMarkUnParentChildren_(parent);
		rlist_.merge(std::move(parent->clist_));
	}
};

#endif // FIBONACCIHEAP_H

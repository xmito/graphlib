#ifndef BINARY_HEAP_H
#define BINARY_HEAP_H
#include <functional>
#include <vector>
#include <memory>
#include <algorithm>
#include <map>
#include <cassert>
#include "GraphTraits.h"
#include "Comparators.h"

namespace graphlib {

template<typename Graph,
         typename Compare = LessDistance<Graph>>
class BinaryHeap {
	using node_handle_id = typename graph_traits<Graph>::node_handle::id_type;
public:
	struct Handle;
	using value_type = typename graph_traits<Graph>::node_handle;
	using value_compare = Compare;
	using reference = typename graph_traits<Graph>::node_handle&;
	using const_reference = const typename graph_traits<Graph>::node_handle&;
	using size_type = typename std::vector<value_type>::size_type;
	using heap_handle = Handle;

	struct Handle {
		using id_type = size_t;
		Handle() : index_(nullptr) {}
		bool operator==(const Handle& handle) const {
			return *index_ == *handle.index_;
		}
		bool operator!=(const Handle& handle) const {
			return !(*this == handle);
		}
		id_type getId() const {
			return *index_;
		}
	private:
		friend class BinaryHeap;
		id_type *index_;
		Handle(id_type *index) : index_(index) {}
	};

	template<typename InputIt>
	BinaryHeap(InputIt first, InputIt last, const Compare& comp) : comp_(comp) {
		size_t idx = 0;
		vec_.reserve(std::distance(first, last));
		while (first != last) {
			const Element &elem = vec_.emplace_back(*first, idx++);
			map_.emplace(first->getId(), elem.getHandle());
			++first;
		}
		buildHeap_();
	}
	BinaryHeap(std::initializer_list<value_type> ilist, const Compare& comp) :
	    BinaryHeap(ilist.begin(), ilist.end(), comp) {}
	BinaryHeap(const Compare& comp) : comp_(comp) {}
	BinaryHeap(const Graph &graph) :
	    BinaryHeap(graph.beginNode(), graph.endNode(), Compare(&graph)) {}
	BinaryHeap(const Graph &graph, const Compare &comp) :
	    BinaryHeap(graph.beginNode(), graph.endNode(), comp) {}
	BinaryHeap(const BinaryHeap&) = delete;
	BinaryHeap& operator=(const BinaryHeap&) = delete;
	BinaryHeap(BinaryHeap&&) = default;
	BinaryHeap& operator=(BinaryHeap&&) = default;

	const_reference top() const {
		assert(!empty());
		return vec_[0].value_;
	}
	heap_handle topHandle() const {
		assert(!empty());
		return vec_[0].getHandle();
	}
	const_reference get(const heap_handle& h) const {
		assert(*h.index_ < vec_.size());
		return vec_[*h.index_].value_;
	}
	heap_handle getHandle(const value_type&nh) {
		assert(nh.getId() < map_.size());
		return map_[nh.getId()];
	}
	bool empty() const {
		return vec_.empty();
	}
	size_type size() const {
		return vec_.size();
	}
	heap_handle push(const value_type& nh) {
		size_t idx = vec_.size();
		const Element& elem = vec_.emplace_back(nh, idx);
		map_[nh.getId()] = elem.getHandle();
		return upReheap_(idx);
	}
	void pop() {
		if (vec_.size() > 1) {
			vec_.front().swap(vec_.back());
			value_type nh = vec_.back().value_;
			map_.erase(nh.getId());
			vec_.pop_back();
			bottomUpReheap_(0);
		} else if (vec_.size() == 1) {
			vec_.pop_back();
			map_.clear();
		}
	}
	void swap(BinaryHeap& heap) {
		using std::swap;
		vec_.swap(heap.vec_);
		map_.swap(heap.map_);
		std::swap(comp_, heap.comp_);
	}
	heap_handle decUpdate(const value_type &nh) {
		return decUpdate(map_[nh.getId()]);
	}
	heap_handle decUpdate(const heap_handle &handle) {
		size_t idx = *handle.index_;
		if (idx > 0)
			upReheap_(idx);
		else if (comp_(vec_[left_(idx)].value_, vec_[idx].value_) ||
		           comp_(vec_[right_(idx)].value_, vec_[idx].value_)) {
			bottomUpReheap_(idx);
		}
		return handle;
	}
	friend bool operator==(const BinaryHeap& bha, const BinaryHeap& bhb) {
		return bha.vec_ == bhb.vec_;
	}
	friend bool operator!=(const BinaryHeap& bha, const BinaryHeap& bhb) {
		return !(bha == bhb);
	}

private:
	struct Element {
		Element(const value_type& value, size_t index) : value_(value), index_(std::make_unique<size_t>(index)) {}
		Element(value_type&& value, size_t index) : value_(std::move(value)), index_(std::make_unique<size_t>(index)) {}
		Element(const Element&) = delete;
		Element& operator=(const Element&) = delete;
		Element(Element&&) = default;
		Element& operator=(Element&& elem) = default;
		heap_handle getHandle() const {
			return heap_handle(index_.get());
		}
		void swap(Element& elem) {
			using std::swap;
			swap(value_, elem.value_);
			index_.swap(elem.index_);
			swap(*index_, *elem.index_);
		}
		bool operator==(const Element &elem) const {
			return value_ == elem.value_;
		}
		bool operator!=(const Element& elem) const {
			return !(*this == elem);
		}

		value_type value_;
		std::unique_ptr<size_t> index_;
	};

	size_t up_(size_t child) {return (child - 1)/2;}
	size_t left_(size_t parent) {return 2*parent + 1;}
	size_t right_(size_t parent) {return 2*parent + 2;}

	/* upReheap_ climbs heap and places element at idx to correct
	 * position. */
	heap_handle upReheap_(size_t idx) {
		while (idx > 0 && comp_(vec_[idx].value_, vec_[up_(idx)].value_)) {
			vec_[idx].swap(vec_[up_(idx)]);
			idx = up_(idx);
		}
		return vec_[idx].getHandle();
	}

	/* leafSearch_ traverses special path and returns index
	 * to leaf node */
	size_t leafSearch_(size_t src) {
		size_type child = left_(src);
		while (child + 1 < vec_.size()) {
			if (comp_(vec_[child + 1].value_, vec_[child].value_))
				++child;
			child = left_(child);
		}
		if (child >= vec_.size())
			child = up_(child);
		return child;
	}

	/* Interchange elements at positions i and j. All other
	 * elements that are between nodes i and j in the tree
	 * are pushed up by one position */
	void interchange_(size_t i, size_t j) {
		if (i >= j)
			return;
		vec_[j].swap(vec_[i]);
		j = up_(j);
		while (i != j) {
			vec_[j].swap(vec_[i]);
			j = up_(j);
		}
	}

	/* Search suitable position for element with index i from
	 * the leaf element with index j. */
	size_t bottomUpSearch_(size_t i, size_t j) {
		while (i < j && comp_(vec_[i].value_, vec_[j].value_))
			j = up_(j);
		return j;
	}

	/* Restore heap with root node, that has index idx */
	void bottomUpReheap_(size_t idx) {
		size_t leaf = leafSearch_(idx);
		size_t pos = bottomUpSearch_(idx, leaf);
		interchange_(idx, pos);
	}

	/* Build heap using bottomUpReheap_ */
	void buildHeap_(void) {
		for (int i = vec_.size()/2 - 1; i >= 0; --i)
			bottomUpReheap_(i);
	}

	std::vector<Element> vec_;
	std::map<node_handle_id, heap_handle> map_;
	Compare comp_;
};

} // namespace graphlib
#endif

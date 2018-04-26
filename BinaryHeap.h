#ifndef BINARY_HEAP_H
#define BINARY_HEAP_H
#include <functional>
#include <vector>
#include <memory>
#include <algorithm>

namespace graphlib {

template<typename T,
         typename Compare = std::less<T>>
struct BinHeap {
	using value_type = T;
	using value_compare = Compare;
	using reference = T&;
	using const_reference = const T&;
	using size_type = typename std::vector<T>::size_type;

	BinHeap(const Compare& comp = Compare()) : comp_(comp) {}
	BinHeap(std::initializer_list<T> ilist, const Compare& comp = Compare()) : BinHeap(ilist.begin(), ilist.end(), comp) {}
	template<typename InputIt>
	BinHeap(InputIt first, InputIt last, const Compare& comp = Compare()) : comp_(comp) {
		size_t idx = 0;
		std::transform(first, last, std::back_inserter(vec_), [&](const T& val) {return Element(val, idx++);});
		buildHeap_();
	}
	template<typename Container>
	BinHeap(const Container& cont, const Compare& comp = Compare()) : BinHeap(cont.begin(), cont.end(), comp) {}

	struct Handle {
		Handle() : index_(nullptr) {}
		bool operator==(const Handle& handle) {
			return *index_ == *handle.index_;
		}
		bool operator!=(const Handle& handle) {
			return !(*this == handle);
		}
	private:
		friend struct BinHeap;
		size_t *index_;
		Handle(size_t *index) : index_(index) {}
	};

	const_reference top() const {
		assert(!empty());
		return vec_[0].value_;
	}
	Handle topHandle() const {
		return vec_[0].handle();
	}
	const_reference get(const Handle& h) {
		return vec_[*h.index_].value_;
	}
	bool empty() const {
		return vec_.empty();
	}
	size_type size() const {
		return vec_.size();
	}
	Handle push(const value_type& value) {
		size_t idx = vec_.size();
		vec_.emplace_back(value, idx);
		return upReheap_(idx);
	}
	Handle push(value_type&& value) {
		size_t idx = vec_.size();
		vec_.emplace_back(std::move(value), idx);
		return upReheap_(idx);
	}
	template<class... Args>
	Handle emplace(Args&&... args) {
		size_t idx = vec_.size();
		vec_.emplace_back(idx, std::forward<Args>(args)...);
		return upReheap_(idx);
	}
	void pop() {
		if (vec_.size() > 1) {
			vec_.front().swap(vec_.back());
			vec_.pop_back();
			bottomUpReheap_(0);
		} else if (vec_.size() == 1)
			vec_.pop_back();
	}
	void swap(BinHeap& heap) {
		using std::swap;
		vec_.swap(heap.vec_);
	}
	void update(const Handle& handle, const T& value) {
		size_t idx = *handle.index_;
		vec_[idx].value_ = value;
		if (idx > 0)
			upReheap_(idx);
		else if (comp_(vec_[left_(idx)].value_, vec_[idx].value_) ||
		         comp_(vec_[right_(idx)].value_, vec_[idx].value_))
			bottomUpReheap_(idx);
	}
	friend bool operator==(const BinHeap& bha, const BinHeap& bhb) {
		return bha.vec_ == bhb.vec_;
	}
	friend bool operator!=(const BinHeap& bha, const BinHeap& bhb) {
		return !(bha == bhb);
	}

private:
	struct Element {
		Element(const T& value, size_t index) : value_(value), index_(std::make_unique<size_t>(index)) {}
		Element(T&& value, size_t index) : value_(std::move(value)), index_(std::make_unique<size_t>(index)) {}
		Element(const Element& elem) : value_(elem.value_), index_(std::make_unique<size_t>(*elem.index_)) {}
		Element(Element&& elem) : value_(std::move(elem.value_)), index_(std::move(elem.index_)) {}
		template<typename... Args>
		Element(size_t index, Args&&... args) : value_(std::forward<Args>(args)...), index_(std::make_unique<size_t>(index)) {}
		bool operator==(const Element& elem) const {
			return value_ == elem.value_ && *index_ == *elem.index_;
		}
		bool operator!=(const Element& elem) const {
			return !(*this == elem);
		}
		bool operator<(const Element& elem) const {
			return value_ < elem.value_;
		}
		bool operator>=(const Element& elem) const {
			return !(*this < elem);
		}
		bool operator>(const Element& elem) const {
			return value_ > elem.value_;
		}
		bool operator<=(const Element& elem) const {
			return !(*this > elem);
		}
		Handle handle() const {
			return index_.get();
		}
		void swap(Element& elem) {
			using std::swap;
			swap(value_, elem.value_);
			index_.swap(elem.index_);
			swap(*index_, *elem.index_);
		}

		T value_;
		std::unique_ptr<size_t> index_;
	};

	size_t up_(size_t child) {return (child - 1)/2;}
	size_t left_(size_t parent) {return 2*parent + 1;}
	size_t right_(size_t parent) {return 2*parent + 2;}

	/* upReheap_ climbs heap and places element at idx to correct
	 * position. */
	Handle upReheap_(size_t idx) {
		size_t tmp = idx;
		while (idx > 0 && comp_(vec_[idx].value_, vec_[up_(idx)].value_)) {
			vec_[idx].swap(vec_[up_(idx)]);
			idx = up_(idx);
		}
		return vec_[idx].handle();
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
	Compare comp_;
};

}
#endif

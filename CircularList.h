/* Author: Matej Hulin
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef CIRCULAR_LIST_H
#define CIRCULAR_LIST_H
#include <iterator>
#include <memory>
#include <optional>
#include <iostream>
#include <cassert>
#include <cstddef>
#include <functional>

struct ClBaseNode;

template<typename>
struct ClNormalNode;

template<typename>
struct ClSentinelNode;

template<typename>
struct CircularList;

template<typename Type>
struct NodeData {
	std::unique_ptr<ClBaseNode> next_;
	ClBaseNode *prev_;
	std::optional<Type> value_;

	NodeData(std::unique_ptr<ClNormalNode<Type>> ptr) : next_(std::move(ptr)), prev_(next_.get()) {}
	NodeData(std::unique_ptr<ClSentinelNode<Type>> ptr) : next_(std::move(ptr)), prev_(next_.get()) {}
	template<class... Args>
	NodeData(Args&&... args) : prev_(nullptr), value_(std::in_place, std::forward<Args>(args)...) {}
	NodeData(NodeData&& data) : next_(std::move(data.next_)), prev_(data.prev_), value_(std::move(data.value_)) {}
	NodeData& operator=(NodeData&& data) {
		if (this == &data)
			return *this;
		next_ = std::move(data.next_);
		prev_ = data.prev_;
		value_ = std::move(data.value_);
		return *this;
	}
};

struct ClBaseNode {
	virtual ~ClBaseNode() = default;
};

template<typename Type>
struct ClBaseNodeInterface : public virtual ClBaseNode {
	virtual NodeData<Type>& data() = 0;
	virtual Type& value() = 0;
	std::unique_ptr<ClBaseNode> &next() {return data().next_;}
	ClBaseNode *&prev() {return data().prev_;}
};


template<typename Type>
struct ClNormalNode : public virtual ClBaseNode, public ClBaseNodeInterface<Type> {
	NodeData<Type> data_;

	template<typename... Args>
	ClNormalNode(Args&&... args) : ClBaseNode(), data_(std::forward<Args>(args)...) {}
	NodeData<Type> &data() override {return data_;}
	Type& value() override {return *data_.value_;}
};

struct IsSentinelException : public std::runtime_error {
	IsSentinelException() : runtime_error("Attempt to access value_ in ClSentinelNode") {}
	IsSentinelException(const std::string& arg) : runtime_error(arg.c_str()) {}
};

template<typename Type>
struct ClSentinelNode : public virtual ClBaseNode, public ClBaseNodeInterface<Type> {
	CircularList<Type> *list_;

	ClSentinelNode(CircularList<Type>& list) : ClBaseNode(), list_(&list) {}
	ClSentinelNode(ClSentinelNode&& snode) : ClBaseNode(), list_(snode->list_) {}
	ClSentinelNode& operator=(ClSentinelNode&& snode) {
		if (this == &snode)
			return *this;
		list_ = snode->list_;
		return *this;
	}
	//NodeData<Type>& data() override {return list_->snode_data_;}
	NodeData<Type>& data() override {return *list_->snode_data_;}
	Type& value() override {throw IsSentinelException();}
};

template<typename Type>
struct CircularList {
	template<typename>
	struct ClIterator;
	template<typename>
	struct ClReverseIterator;

	using value_type = Type;
	using reference = Type&;
	using const_reference = const Type&;
	using size_type = std::size_t;
	using iterator = ClIterator<Type>;
	using const_iterator = ClIterator<const Type>;
	using reverse_iterator = ClReverseIterator<Type>;
	using const_reverse_iterator = ClReverseIterator<const Type>;

	enum class LastAction {NOA, INC, DEC};
	template<typename>
	friend struct ClSentinelNode;

	template<typename T>
	struct ClIterator {
		using value_type = T;
		using reference = T&;
		using pointer = T*;
		using difference_type = std::ptrdiff_t;
		using iterator_category = std::bidirectional_iterator_tag;

		template<typename U>
		friend struct ClIterator;
		template<typename>
		friend struct CircularList;

		ClIterator() : cit_(nullptr), act_(LastAction::NOA) {}
		ClIterator(ClBaseNode *ptr) : cit_(ptr), act_(LastAction::NOA) {}
		ClIterator(std::nullptr_t) : cit_(nullptr), act_(LastAction::NOA) {}
		/* Allows for construction of const_iterator from iterator. However, for casual
		 * iterator, it is same as explicit copy constructor. Same goes for assignment operator */
		ClIterator(const ClIterator<std::remove_const_t<T>>& it) : cit_(it.cit_), act_(it.act_) {}
		ClIterator& operator=(const ClIterator<std::remove_const_t<T>>& it) {
			if (this->cit_ == it.cit_)
				return *this;
			cit_ = it.cit_;
			act_ = it.act_;
			return *this;
		}
		ClIterator& operator=(std::nullptr_t) {
			cit_ = nullptr;
			act_ = LastAction::NOA;
			return *this;
		}
		ClIterator& operator++() {
			cit_ = dynamic_cast<ClBaseNodeInterface<Type>*>(cit_)->next().get();
			act_ = LastAction::INC;
			return *this;
		}
		ClIterator operator++(int) {
			auto cp(*this);
			cit_ = dynamic_cast<ClBaseNodeInterface<Type>*>(cit_)->next().get();
			act_ = LastAction::INC;
			return cp;
		}
		ClIterator& operator--() {
			cit_ = dynamic_cast<ClBaseNodeInterface<Type>*>(cit_)->prev();
			act_ = LastAction::DEC;
			return *this;
		}
		ClIterator operator--(int) {
			auto cp(*this);
			cit_ = dynamic_cast<ClBaseNodeInterface<Type>*>(cit_)->prev();
			act_ = LastAction::DEC;
			return cp;
		}
		/* If end iterator (to sentinel node) is dereferenced, the value
		 * function throws and operator* catches this exception. It then
		 * either increases/decreases iterator to next valid value, or
		 * otherwise rethrows catched exception (if list is empty) */
		reference operator*() {
			try {
				return dynamic_cast<ClBaseNodeInterface<Type>*>(cit_)->value();
			} catch (const IsSentinelException& ex) {
				if (cit_ == dynamic_cast<ClBaseNodeInterface<Type>*>(cit_)->prev())
					throw;
				switch (act_) {
				case LastAction::NOA:
					throw;
					break;
				case LastAction::INC:
					++*this;
					break;
				case LastAction::DEC:
					--*this;
					break;
				}
			}
			return dynamic_cast<ClBaseNodeInterface<Type>*>(cit_)->value();
		}
		pointer operator->() {
			return &(operator*());
		}
		explicit operator bool() const {
			return cit_ != nullptr;
		}

		template<typename K>
		bool operator==(ClIterator<K> it) const {
			return cit_ == it.cit_;
		}
		template<typename K>
		bool operator!=(ClIterator<K> it) const {
			return cit_ != it.cit_;
		}
		bool operator==(std::nullptr_t) const {
			return cit_ == nullptr;
		}
		bool operator!=(std::nullptr_t) const {
			return cit_ != nullptr;
		}

	private:
		ClBaseNode *cit_;
		LastAction act_;
	};

	template<typename T>
	struct ClReverseIterator {
		using value_type = T;
		using reference = T&;
		using pointer = T*;
		using difference_type = std::ptrdiff_t;
		using iterator_category = std::bidirectional_iterator_tag;

		template<typename>
		friend struct ClReverseIterator;
		template<typename>
		friend struct CircularList;

		ClReverseIterator() : cit_(nullptr), act_(LastAction::NOA) {}
		ClReverseIterator(ClBaseNode *ptr) : cit_(dynamic_cast<ClBaseNodeInterface<Type>*>(ptr)->prev()), act_(LastAction::NOA) {}
		ClReverseIterator(std::nullptr_t) : cit_(nullptr), act_(LastAction::NOA) {}
		// Construct const_reverse_iterator from reverse_iterator
		template<typename O,
		         typename = std::enable_if_t<std::is_same_v<T, std::add_const_t<O>>>>
		ClReverseIterator(ClReverseIterator<O> it) : cit_(it.cit_), act_(it.act_) {}

		// Assign from reverse_iterator to const_reverse_iterator
		template<typename O,
		         typename = std::enable_if_t<std::is_same_v<T, std::add_const_t<O>>>>
		ClReverseIterator& operator=(ClReverseIterator<O> it) {
			if (cit_ == it.cit_)
				return *this;
			cit_ = it.cit_;
			act_ = it.act_;
			return *this;
		}
		ClReverseIterator& operator=(std::nullptr_t) {
			cit_ = nullptr;
			act_ = LastAction::NOA;
			return *this;
		}
		// Construct reverse_iterator or const_reverse_iterator from iterator
		template<typename O,
		         typename = std::enable_if_t<std::is_same_v<O, T>  || std::is_same_v<T, std::add_const_t<O>>>>
		ClReverseIterator(ClIterator<O> it) : cit_(dynamic_cast<ClBaseNodeInterface<Type>*>(it.cit_)->prev()),
		    act_(it.act_) {}

		ClReverseIterator& operator++() {
			cit_ = dynamic_cast<ClBaseNodeInterface<Type>*>(cit_)->prev();
			act_ = LastAction::INC;
			return *this;
		}
		ClReverseIterator operator++(int) {
			auto cp(*this);
			cit_ = dynamic_cast<ClBaseNodeInterface<Type>*>(cit_)->prev();
			act_ = LastAction::INC;
			return cp;
		}
		ClReverseIterator& operator--() {
			cit_ = dynamic_cast<ClBaseNodeInterface<Type>*>(cit_)->next().get();
			act_ = LastAction::DEC;
			return *this;
		}
		ClReverseIterator operator--(int) {
			auto cp(*this);
			cit_ = dynamic_cast<ClBaseNodeInterface<Type>*>(cit_)->next().get();
			act_ = LastAction::DEC;
			return cp;
		}
		/* If end iterator (to sentinel node) is dereferenced, the value
		 * function throws and operator* catches this exception. It then
		 * either increases/decreases iterator to next valid value, or
		 * otherwise rethrows catched exception (if list is empty) */
		reference operator*() {
			try {
				return dynamic_cast<ClBaseNodeInterface<Type>*>(cit_)->value();
			} catch (const IsSentinelException& ex) {
				if (cit_ == dynamic_cast<ClBaseNodeInterface<Type>*>(cit_)->prev())
					throw;
				switch (act_) {
				case LastAction::NOA:
				case LastAction::INC:
					++*this;
					break;
				case LastAction::DEC:
					--*this;
					break;
				}
			}
			return dynamic_cast<ClBaseNodeInterface<Type>*>(cit_)->value();
		}
		pointer operator->() {
			return &(operator*());
		}
		explicit operator bool() const {
			return cit_ != nullptr;
		}
		template<typename K>
		bool operator==(ClReverseIterator<K> it) const {
			return cit_ == it.cit_;
		}
		template<typename K>
		bool operator!=(ClReverseIterator<K> it) const {
			return cit_ != it.cit_;
		}
		bool operator==(std::nullptr_t) const {return cit_ == nullptr;}
		bool operator!=(std::nullptr_t) const {return cit_ != nullptr;}
		ClIterator<T> base() {
			return ClIterator<T>(dynamic_cast<ClBaseNodeInterface<Type>*>(cit_)->next().get());
		}

	private:
		ClBaseNode *cit_;
		LastAction act_;
	};

	CircularList() :
	    snode_data_(std::make_unique<NodeData<Type>>(std::make_unique<ClSentinelNode<Type>>(*this))), size_(0) {}
	CircularList(const CircularList& clist) : snode_data_(std::make_unique<NodeData<Type>>(std::make_unique<ClSentinelNode<Type>>(*this))),
	    size_(0) {
		for(auto& val : clist)
			push_back(val);
	}
	CircularList& operator=(const CircularList& clist) {
		if (this == &clist)
			return *this;
		clear();
		for (auto& val : clist)
			push_back(val);
		return *this;
	}
	CircularList(CircularList&& clist) :
	    snode_data_(std::move(clist.snode_data_)), size_(clist.size_) {
		// Change pointer in sentinel node so that it points to correct CircularList
		ClSentinelNode<Type> *node = dynamic_cast<ClSentinelNode<Type>*>(getPrev(snode_data_->next_));
		node->list_ = this;
		clist.size_ = 0;
		clist.snode_data_ = std::make_unique<NodeData<Type>>(std::make_unique<ClSentinelNode<Type>>(clist));
	}
	CircularList& operator=(CircularList&& clist) {
		if (this == &clist)
			return *this;
		snode_data_ = std::move(clist.snode_data_);
		size_ = clist.size_;
		clist.size_ = 0;
		clist.snode_data_ = std::make_unique<NodeData<Type>>(std::make_unique<ClSentinelNode<Type>>(clist));
		ClSentinelNode<Type> *node = dynamic_cast<ClSentinelNode<Type>*>(getPrev(snode_data_->next_));
		node->list_ = this;
		return *this;
	}

	// Capacity
	bool empty() const {return size_ == 0;}
	size_type size() const {return size_;}

	// Element access
	reference front() {
		assert(!empty());
		return getValue(head());
	}
	const_reference front() const {
		assert(!empty());
		return getValue(head());
	}
	reference back() {
		assert(!empty());
		return getValue(tail());
	}
	const_reference back() const {
		assert(!empty());
		return getValue(tail());
	}
	iterator find(const Type& value) {
		return find_(begin(), value);
	}
	const_iterator find(const Type& value) const {
		return find_(begin(), value);
	}

	//Modifiers
	void clear() {
		snode_data_->next_ = std::move(getNext(snode_data_->prev_));
		snode_data_->prev_ = snode_data_->next_.get();
		size_ = 0;
	}
	iterator insert(const_iterator pos, const Type& value) {
		return insert_(pos, std::make_unique<ClNormalNode<Type>>(value));
	}
	iterator insert(const_iterator pos, Type&& value) {
		return insert_(pos, std::make_unique<ClNormalNode<Type>>(std::move(value)));
	}
	iterator insert(const_iterator pos, size_type count, const Type& value) {
		if (!count)
			return iterator(pos.cit_);
		CircularList<Type>::iterator ret = insert_(pos, std::make_unique<ClNormalNode<Type>>(value));
		while(--count)
			insert_(pos, std::make_unique<ClNormalNode<Type>>(value));
		return ret;
	}
	template<typename InputIt>
	iterator insert(const_iterator pos, InputIt first, InputIt last) {
		if (first == last)
			return iterator(pos.cit_);
		iterator ret = insert_(pos, std::make_unique<ClNormalNode<Type>>(*first));
		++first;
		while (first != last) {
			insert_(pos, std::make_unique<ClNormalNode<Type>>(*first));
			++first;
		}
		return ret;
	}
	iterator insert(const_iterator pos, std::initializer_list<Type> ilist) {
		if (!ilist.size())
			return iterator(pos.cit_);
		return insert(pos, ilist.begin(), ilist.end());
	}

	iterator push_front(const Type& value) {
		return insert_(cbegin(), std::make_unique<ClNormalNode<Type>>(value));
	}
	iterator push_front(Type&& value) {
		return insert_(cbegin(), std::make_unique<ClNormalNode<Type>>(std::move(value)));
	}
	iterator push_back(const Type& value) {
		return insert_(cend(), std::make_unique<ClNormalNode<Type>>(value));
	}
	iterator push_back(Type&& value) {
		return insert_(cend(), std::make_unique<ClNormalNode<Type>>(std::move(value)));
	}
	void assign(size_type count, const Type& value) {
		clear();
		insert(cbegin(), count, value);
	}
	template<typename InputIt>
	void assign(InputIt first, InputIt last) {
		clear();
		insert(cbegin(), first, last);
	}
	void assign(std::initializer_list<Type> ilist) {
		assign(ilist.begin(), ilist.end());
	}
	template<class... Args>
	iterator emplace_back(Args&&... args) {
		return insert_(cend(), std::make_unique<ClNormalNode<Type>>(std::forward<Args>(args)...));
	}
	template<class... Args>
	iterator emplace_front(Args&&... args) {
		return insert_(cbegin(), std::make_unique<ClNormalNode<Type>>(std::forward<Args>(args)...));
	}
	void resize(size_type count) {
		resize(count, Type());
	}
	void resize(size_type count, const value_type& value) {
		if (size_ > count) {
			erase(std::next(cbegin(), count), cend());
		}
		else if (size_ < count)
			insert(cend(), count - size_, value);
	}
	iterator erase(const_iterator pos) {
		return erase_(pos);
	}
	iterator erase(const_iterator first, const_iterator last) {
		return erase_(first, last);
	}
	void pop_front() {
		assert(!empty());
		erase(head());
	}
	void pop_back() {
		assert(!empty());
		erase(tail());
	}
	void merge(const CircularList<Type>& circ) {
		for (auto& val : circ)
			push_back(val);
	}
	void merge(CircularList<Type>&& circ) {
		if (circ.empty())
			return;
		ClBaseNode *listTail = tail();
		NodeData<Type> &tailData = getData(listTail);
		NodeData<Type> &circData = *circ.snode_data_;
		// Connect tail of circ to snode_data
		snode_data_->prev_ = circData.prev_;
		ClBaseNode *circSentinel = getNext(snode_data_->prev_).release();
		getNext(snode_data_->prev_) = std::move(tailData.next_);
		// Connect tail to first node of circ
		tailData.next_ = std::move(circData.next_);
		getPrev(tailData.next_) = listTail;
		size_ += circ.size();
		circ.size_ = 0;
		circData.prev_ = circSentinel;
		circData.next_ = std::unique_ptr<ClBaseNode>(circSentinel);
	}

	void splice(const_iterator pos, CircularList& other, const_iterator it) {
		if (it.cit_ == other.sentinel())
			return;
		auto& posdata = getData(pos.cit_);
		auto& itdata = other.getData(it.cit_);

		// Detach node from other CircularList
		getPrev(itdata.next_) = itdata.prev_;
		getNext(itdata.prev_).release();
		getNext(itdata.prev_) = std::move(itdata.next_);

		// Insert node to this CircularList
		itdata.prev_ = posdata.prev_;
		itdata.next_ = std::move(getNext(posdata.prev_));
		getNext(posdata.prev_) = std::unique_ptr<ClBaseNode>(it.cit_);
		posdata.prev_ = it.cit_;

		// Increment and decrement
		--other.size_;
		++size_;
	}

	// Iterators
	iterator begin() {return iterator(head());}
	const_iterator begin() const {return const_iterator(head());}
	const_iterator cbegin() const {return const_iterator(head());}
	reverse_iterator rbegin() {return reverse_iterator(sentinel());}
	const_reverse_iterator crbegin() const {return const_reverse_iterator(sentinel());}

	iterator end() {return iterator(sentinel());}
	const_iterator end() const {return const_iterator(sentinel());}
	const_iterator cend() const {return const_iterator(sentinel());}
	reverse_iterator rend() {return reverse_iterator(head());}
	const_reverse_iterator crend() const {return const_reverse_iterator(head());}

private:
	std::unique_ptr<NodeData<Type>> snode_data_;
	size_type size_;
	ClBaseNode *head() const {return snode_data_->next_.get();}
	ClBaseNode *sentinel() const {return getPrev(snode_data_->next_);}
	ClBaseNode *tail() const {return snode_data_->prev_;}

	ClBaseNode *&getPrev(std::unique_ptr<ClBaseNode>& node) {
		return dynamic_cast<ClBaseNodeInterface<Type>*>(node.get())->prev();
	}
	ClBaseNode *&getPrev(const std::unique_ptr<ClBaseNode>& node) const {
		return dynamic_cast<ClBaseNodeInterface<Type>*>(node.get())->prev();
	}
	std::unique_ptr<ClBaseNode> &getNext(ClBaseNode *node) {
		return dynamic_cast<ClBaseNodeInterface<Type>*>(node)->next();
	}
	reference getValue(ClBaseNode *node) {
		return dynamic_cast<ClBaseNodeInterface<Type>*>(node)->value();
	}
	const_reference getValue(ClBaseNode *node) const {
		return dynamic_cast<ClBaseNodeInterface<const Type>*>(node)->value();
	}
	NodeData<Type> &getData(ClBaseNode *node) {
		return dynamic_cast<ClBaseNodeInterface<Type>*>(node)->data();
	}

	iterator insert_(const_iterator pos, std::unique_ptr<ClBaseNode> ptr) {
		NodeData<Type>& pos_data = getData(pos.cit_);
		NodeData<Type>& ptr_data = getData(ptr.get());
		ClBaseNode *ret = ptr.get();
		ptr_data.prev_ = pos_data.prev_;
		ptr_data.next_ = std::move(getNext(pos_data.prev_));
		pos_data.prev_ = ptr.get();
		getNext(ptr_data.prev_) = std::move(ptr);
		++size_;
		return iterator(ret);
	}
	iterator erase_(const_iterator pos) {
		if (pos == cend())
			return iterator(pos.cit_);
		auto& pdata = getData(pos.cit_);
		ClBaseNode *ret(pdata.next_.get());
		getPrev(pdata.next_) = pdata.prev_;
		getNext(pdata.prev_) = std::move(pdata.next_);
		--size_;
		return iterator(ret);
	}
	iterator erase_(const_iterator first, const_iterator last) {
		if (first == last)
			return iterator(first.cit_);
		size_ -= std::distance(first, last);
		NodeData<Type> &fdata = getData(first.cit_);
		NodeData<Type> &ldata = getData(last.cit_);
		ClBaseNode *prev = fdata.prev_;
		getNext(fdata.prev_) = std::move(getNext(ldata.prev_));
		ldata.prev_ = prev;
		return iterator(last.cit_);
	}
	iterator find_(const_iterator pos, const Type& value) {
		iterator clend = end();
		while (pos != clend) {
			if (value == *pos)
				return iterator(pos.cit_);
			++pos;
		}
		return clend;
	}
	const_iterator find_(const_iterator pos, const Type& value) const {
		return const_cast<CircularList<Type>&>(*this).find_(pos, value);
	}
};

// Non-member functions
template<typename U>
bool operator==(const CircularList<U>& cla, const CircularList<U>& clb) {
	if (cla.size() != clb.size())
		return false;
	typename CircularList<U>::const_iterator ita = cla.begin();
	typename CircularList<U>::const_iterator itb = clb.begin();
	for (; ita != cla.end(); ++ita, ++itb)
		if (*ita != *itb)
			return false;
	return true;
}

template<typename U>
bool operator!=(const CircularList<U>& cla, const CircularList<U>& clb) {
	return !(cla == clb);
}

template<typename U>
bool operator<(const CircularList<U>& cla, const CircularList<U>& clb) {
	if (cla.size() > clb.size())
		return false;
	typename CircularList<U>::const_iterator ita = cla.begin();
	typename CircularList<U>::const_iterator itb = clb.begin();
	for (; ita != cla.end(); ++ita, ++itb)
		if (*ita >= *itb)
			return false;
	return true;
}

template<typename U>
bool operator<=(const CircularList<U>& cla, const CircularList<U>& clb) {
	return !(cla > clb);
}

template<typename U>
bool operator>(const CircularList<U>& cla, const CircularList<U>& clb) {
	if (cla.size() < clb.size())
		return false;
	typename CircularList<U>::const_iterator ita = cla.begin();
	typename CircularList<U>::const_iterator itb = clb.begin();
	for (; itb != clb.end(); ++ita, ++itb)
		if (*ita <= *itb)
			return false;
	return true;
}

template<typename U>
bool operator>=(const CircularList<U>& cla, const CircularList<U>& clb) {
	return !(cla < clb);
}
#endif

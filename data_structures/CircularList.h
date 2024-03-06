#ifndef CIRCULAR_LIST_H
#define CIRCULAR_LIST_H

#include <algorithm>
#include <functional>
#include <list>
#include <type_traits>

namespace graphlib {

/**
 * @brief CircularList is a circular list implementation based on regular std::list
 * @tparam T Type of elements stored inside circular list
 */
template <typename T>
class CircularList {
    using list_iterator = typename std::list<T>::iterator;

    enum class Behaviour : char {
        CLIST,
        LIST
    };
    template <bool IsConst>
    struct CListIterator {
        using value_type = T;
        using reference = std::conditional_t<IsConst, const T &, T &>;
        using pointer = std::conditional_t<IsConst, const T *, T *>;
        using difference_type = std::ptrdiff_t;
        using iterator_category = std::bidirectional_iterator_tag;

      private:
        using list_type =
            std::conditional_t<IsConst, const std::list<value_type>,
                               std::list<value_type>>;
        using list_iterator =
            std::conditional_t<IsConst,
                               typename std::list<value_type>::const_iterator,
                               typename std::list<value_type>::iterator>;
        list_type *list_;
        list_iterator cit_;
        Behaviour bhv_;

        template <bool>
        friend struct CListIterator;
        template <typename>
        friend class CircularList;

      public:
        explicit CListIterator(Behaviour bhv = Behaviour::CLIST)
            : list_(nullptr), bhv_(bhv) {}
        explicit CListIterator(list_type &list,
                               Behaviour bhv = Behaviour::CLIST)
            : list_(&list), cit_(list->begin()), bhv_(bhv) {}
        explicit CListIterator(list_type &list, list_iterator cit,
                               Behaviour bhv = Behaviour::CLIST)
            : list_(&list), cit_(cit), bhv_(bhv) {}
        template <bool K = IsConst, typename std::enable_if_t<K, int> = 0>
        CListIterator(const CListIterator<false> &it)
            : list_(it.list_), cit_(it.cit_), bhv_(it.bhv_) {}
        template <bool K = IsConst, typename std::enable_if_t<K, int> = 0>
        CListIterator &operator=(const CListIterator<false> &it) {
            cit_ = it.cit_;
            list_ = it.list_;
            bhv_ = it.bhv_;
            return *this;
        }
        explicit CListIterator(std::nullptr_t) : list_(nullptr) {}
        CListIterator &operator=(std::nullptr_t) {
            cit_ = list_->end();
            list_ = nullptr;
            return *this;
        }
        CListIterator &operator++() {
            ++cit_;
            if (list_ && !list_->empty() && cit_ == list_->end() &&
                bhv_ == Behaviour::CLIST)
                cit_ = list_->begin();
            return *this;
        }
        CListIterator operator++(int) {
            auto clit(*this);
            ++(*this);
            return clit;
        }
        CListIterator &operator--() {
            --cit_;
            if (list_ && !list_->empty() && cit_ == --list_->begin() &&
                bhv_ == Behaviour::CLIST)
                cit_ = --list_->end();
            return *this;
        }
        CListIterator operator--(int) {
            auto clit(*this);
            --(*this);
            return clit;
        }
        reference operator*() { return *cit_; }
        pointer operator->() { return &(*cit_); }
        template <bool K>
        bool operator==(CListIterator<K> it) const {
            return cit_ == it.cit_;
        }
        template <bool K>
        bool operator!=(CListIterator<K> it) const {
            return !(*this == it);
        }
        bool operator==(std::nullptr_t) const { return list_ == nullptr; }
        bool operator!=(std::nullptr_t) const { return list_ != nullptr; }
        explicit operator bool() const { return list_ != nullptr; }
        void setBehaviour(Behaviour bhv) { bhv_ = bhv; }
        Behaviour getBehaviour() const { return bhv_; }
    };

  public:
    /** Alias for type of elements stored inside circular list */
    using value_type = T;
    /** Alias for reference to value_type */
    using reference = T &;
    /** Alias for constant reference to value_type */
    using const_reference = const T &;
    /** Alias for circular list iterator used to pass stored elements */
    using iterator = CListIterator<false>;
    /** Alias for constant circular list iterator used to pass stored elements */
    using const_iterator = CListIterator<true>;
    /** Alias for type used to represent count of currently stored elements */
    using size_type = size_t;

    /**
     * @brief Constructs empty CircularList instance
     */
    CircularList() = default;
    /**
     * @brief Constructs CircularList with count copies of elements with value value
     * @param count Number of copies to create
     * @param value Instance of template argument, that should be replicated
     */
    explicit CircularList(size_type count, const_reference value = T())
        : list_(count, value) {}
    /**
     * @brief Constructs CircularList with the contents of the range [first, last)
     * @tparam Iterator Type of iterator used to provide first and last constraints of range
     * @param first iterator, that points to the first element of the range
     * @param last iterator, that points to the last element of the range
     */
    template <typename Iterator>
    CircularList(Iterator first, Iterator last) : list_(first, last) {}

    /**
     * @brief Returns true if circular list is empty
     * @return bool value, that signifies emptiness of the circular list
     */
    bool empty() const { return list_.empty(); }
    /**
     * @brief Returns count of currently stored elements in circular list
     * @return Count of elements stored in circular list
     */
    size_type size() const { return list_.size(); }
    /**
     * @brief Returns reference to the first element in circular list
     * @return reference to the first element
     */
    reference front() { return list_.front(); }
    /**
     * @brief Returns constant reference to the first element in the circular list
     * @return const_reference to the first element
     */
    const_reference front() const { return list_.front(); }
    /**
     * @brief Returns reference to the last element in the circular list
     * @return reference to the last element
     */
    reference back() { return list_.back(); }
    /**
     * @brief Returns constant reference to the last element int the circular list
     * @return const_reference to the last element
     */
    const_reference back() const { return list_.back(); }
    /**
     * @brief Returns an iterator to the first element of the circular list.
     * @return iterator to the first element
     */
    iterator begin() { return iterator(list_, list_.begin(), Behaviour::LIST); }
    /**
     * @brief Returns a constant iterator to the first element of the circular list.
     * @return const_iterator to the first element
     */
    const_iterator begin() const {
        return const_iterator(list_, list_.begin(), Behaviour::LIST);
    }
    /**
     * @brief Returns a constant iterator to the first element of the circular list.
     * @return const_iterator to the first element
     */
    const_iterator cbegin() {
        return const_iterator(list_, list_.cbegin(), Behaviour::LIST);
    }
    /**
     * @brief Returns an iterator to the last element in the circular list
     * @return iterator to the last element
     */
    iterator end() { return iterator(list_, list_.end(), Behaviour::LIST); }
    /**
     * @brief Returns a constant iterator to the last element in the circular list
     * @return const_iterator to the last element
     */
    const_iterator end() const {
        return const_iterator(list_, list_.end(), Behaviour::LIST);
    }
    /**
     * @brief Returns a constant iterator to the last element in the circular list
     * @return const_iterator to the last element
     */
    const_iterator cend() {
        return const_iterator(list_, list_.cend(), Behaviour::LIST);
    }
    /**
     * @brief Returns an iterator to random element stored in the circular list. Incrementing iterator pointing to the last element in the circular list will cause it to point to the first element
     * @return iterator to random element
     */
    iterator citer() { return iterator(list_, list_.begin()); }
    /**
     * @brief Returns a constant iterator to random element stored in the circular list. Incrementing iterator pointing to the last element in the circular list will cause it to point to the first element
     * @return const_iterator to random element
     */
    const_iterator citer() const {
        return const_iterator(list_, list_.begin());
    }

    /**
     * @brief Removes all elements from the circular list. Invalidates any references, pointers, or iterators referring to contained elements. Any past-the-end iterators are also invalidated.
     */
    void clear() { list_.clear(); }
    /**
     * @brief Inserts value before position pos
     * @param pos Position before which to place value
     * @param value Value to insert into circular list
     * @return iterator pointing to the inserted value
     */
    iterator insert(const_iterator pos, const T &value) {
        auto cit = list_.insert(pos.cit_, value);
        return iterator(list_, cit);
    }
    /**
     * @brief Inserts value before position pos
     * @param pos Position before which to place value
     * @param value Value to insert into circular list
     * @return iterator pointing to the inserted value
     */
    iterator insert(const_iterator pos, T &&value) {
        auto cit = list_.insert(pos.cit_, std::move(value));
        return iterator(list_, cit);
    }
    /**
     * @brief Inserts count copies of value before position pos
     * @param pos Position before which to insert
     * @param value Value to insert into circular list
     * @param count Number of copies to insert
     * @return iterator pointing to the first element inserted, or pos if count==0
     */
    iterator insert(const_iterator pos, size_type count, const T &value) {
        auto cit = list_.insert(pos.cit_, count, value);
        return iterator(list_, cit);
    }
    /**
     * @brief Inserts elements from range [first, last) before position pos
     * @tparam InputIt Type of iterator used to represent range bounds
     * @param pos Position before which to insert
     * @param first InputIt instance, that points to the first element of the range
     * @param last InputIt instance, that points to the last element of the range
     * @return iterator pointing to the first element inserted, ot pos if first==last
     */
    template <typename InputIt>
    iterator insert(const_iterator pos, InputIt first, InputIt last) {
        auto it = list_.insert(pos.cit_, first, last);
        return iterator(list_, it);
    }
    /**
     * @brief Inserts elements from initializer list ilist before pos
     * @param pos Position before which to insert
     * @param ilist List of values to insert into circular_list
     * @return iterator pointing to the first element inserted, or pos if ilist is empty
     */
    iterator insert(const_iterator pos, std::initializer_list<T> ilist) {
        auto it = list_.insert(pos.cit_, ilist);
        return iterator(list_, it);
    }
    /**
     * @brief Prepends the given element value to the beginning of the circular list. No iterators or references are invalidated.
     * @param value Value to push to the circular list
     */
    void push_front(const T &value) { list_.push_front(value); }
    /**
     * @brief Prepends the given element value to the beginning of the circular list. No iterators or references are invalidated.
     * @param value Value to push to the circular list
     */
    void push_front(T &&value) { list_.push_front(std::move(value)); }
    /**
     * @brief Appends the given element value to the end of the circular list. No iterators or references are invalidated.
     * @param value Value to push to the circular list
     */
    void push_back(const T &value) { list_.push_back(value); }
    /**
     * @brief Appends the given element value to the end of the circular list. No iterators or references are invalidated.
     * @param value Value to push to the circular list
     */
    void push_back(T &&value) { list_.push_back(std::move(value)); }
    /**
     * @brief Replaces the contents with count copies of value value
     * @param count Number of copies to insert
     * @param value Value to push to the circular list
     */
    void assign(size_type count, const T &value) { list_.assign(count, value); }
    /**
     * @brief Replaces the contents with copies of those in range [first, last)
     * @tparam InputIt Type of iterator used to represent range bounds
     * @param first InputIt instance, that points to the first element of the range
     * @param last InputIt instance, that points to the last element of the range
     */
    template <typename InputIt>
    void assign(InputIt first, InputIt last) {
        list_.assign(first, last);
    }
    /**
     * @brief Replaces the contents with elements from the initializer list
     * @param ilist List of values to be inserted
     */
    void assign(std::initializer_list<T> ilist) { list_.assign(ilist); }
    /**
     * @brief Inserts a new element into the circular list directly before position pos. The element is constructed by forwarding provided arguments to element constructor.
     * @tparam Args Parameter pack, which specifies types of provided constructor arguments
     * @param pos Position before which an emplaced element should be constructed
     * @param args Arguments needed to construct the element
     * @return iterator pointing to the emplaced element
     */
    template <class... Args>
    iterator emplace(const_iterator pos, Args &&...args) {
        auto it = list_.emplace(pos, std::forward<Args>(args)...);
        return iterator(list_, it);
    }
    /** @brief Appends a new element to the end of the circular list. The element is constructed by forwarding provided arguments to element constructor.
     * @tparam Args Parameter pack, which specifies types of provided constructor arguments
     * @param args Arguments needed to construct the element
     * @return iterator pointing to the emplaced element
     */
    template <class... Args>
    iterator emplace_back(Args &&...args) {
        list_.emplace_back(std::forward<Args>(args)...);
        return iterator(list_, --list_.end());
    }
    /** @brief Prepends a new element to the beginning of the circular list. The element is constructed by forwarding provided arguments to element constructor.
     * @tparam Args Parameter pack, which specifies types of provided constructor arguments
     * @param args Arguments needed to construct the element
     * @return iterator pointing to the emplaced element
     */
    template <class... Args>
    iterator emplace_front(Args &&...args) {
        list_.emplace_front(std::forward<Args>(args)...);
        return iterator(list_, list_.begin());
    }
    /**
     * @brief Resizes the circular list to contain count elements. If the current size is greater than count, the circular list is reduced to its first count elements. If the current size is less than count, additional default-inserted elements are appended
     * @param count New size of the circular list
     */
    void resize(size_type count) { list_.resize(count, T()); }
    /**
     * @brief Resizes the circular list to contain count elements. If the current size is greater than count, the circular list is reduced to its first count elements. If the current size is less than count, additional elements are appended and initialized with copies of value.
     * @param count New size of the circular list
     * @param value The value to initialize the new elements with
     */
    void resize(size_type count, const T &value) { list_.resize(count, value); }
    /**
     * @brief Removes element at position pos
     * @param pos Iterator to the element to remove
     * @return iterator following the last removed element. If iterator pos refers to the last element, erase method returns iterator to the first element.
     */
    iterator erase(const_iterator pos) {
        auto it = list_.erase(pos.cit_);
        return it == list_.end() ? iterator(list_, list_.begin())
                                 : iterator(list_, it);
    }
    /**
     * @brief Removes the elements in range [first, last)
     * @param first Constant iterator to the first element in the range
     * @param last Constant iterator to the last element in the range
     * @return iterator following the last removed element. If iterator pos refers to the last element, erase method returns iterator to the first element.
     */
    iterator erase(const_iterator first, const_iterator last) {
        auto it = list_.erase(first.cit_, last.cit_);
        return it == list_.end() ? iterator(list_, list_.begin())
                                 : iterator(list_, it);
    }

    /**
     * @brief Removes the first element of the circular list. References and iterators to the erased element are invalidated.
     */
    void pop_front() { list_.pop_front(); }
    /**
     * @brief Removes the last element in the circular list. References and iterators to the erased element are invalidated.
     */
    void pop_back() { list_.pop_back(); }
    /**
     * @brief Exchanges contents of the circular list with those of other.
     * @param other Reference to the circular list with which to exchange elements
     */
    void swap(CircularList &other) noexcept { list_.swap(other.list_); }
    /**
     * @brief Transfers all elements from clist into *this. The elements are inserted before element pointed to by pos
     * @param pos Constant iterator to element before which the content will be inserted
     * @param clist Circular list from which to transfer contents
     */
    void splice(const_iterator pos, CircularList &&clist) {
        list_.splice(pos.cit_, std::move(clist.list_));
    }
    /**
     * @brief Transfers the element pointed to by it from other into *this. The element is inserted before the element pointed to by pos.
     * @param pos Iterator to element before which the content will be inserted
     * @param clist Circular list, that contains element pointed to by it
     * @param it Constant iterator to the element, that should be moved
     */
    void splice(const_iterator pos, CircularList &&clist, const_iterator it) {
        list_.splice(pos.cit_, std::move(clist.list_), it.cit_);
    }

  private:
    std::list<T> list_;
};

/**
 * @brief Returns true if two circular lists are equal
 * @param cla Circular list to compare to clb
 * @param clb Circular list to compare to cla
 * @return bool value, that signifies equality of provided circular lists
 */
template <typename T>
bool operator==(const CircularList<T> &cla, const CircularList<T> &clb) {
    return std::equal(cla.begin(), cla.end(), clb.begin(), clb.end());
}

/**
 * @brief Returns true if two circular lists are not equal
 * @param cla Circular list to compare to clb
 * @param clb Circular list to compare to cla
 * @return bool value, that signifies non-equality of provided circular lists
 */
template <typename T>
bool operator!=(const CircularList<T> &cla, const CircularList<T> &clb) {
    return !(cla == clb);
}

/**
 * @brief Returns true if the first provided circular list is lexicographically less than the second one
 * @param cla Circular list to compare
 * @param clb Circular list to compare
 * @return bool value, that signifies, that the first circular list is lexicographically less than the second one
 */
template <typename T>
bool operator<(const CircularList<T> &cla, const CircularList<T> &clb) {
    return std::lexicographical_compare(cla.begin(), cla.end(), clb.begin(),
                                        clb.end());
}

/**
 * @brief Returns true if the first provided circular list is lexicographically greater than the second one
 * @param cla Circular list to compare
 * @param clb Circular list to compare
 * @return bool value, that signifies, that the first circular list is lexicographically greater than the second one
 */
template <typename T>
bool operator>(const CircularList<T> &cla, const CircularList<T> &clb) {
    return std::lexicographical_compare(cla.begin(), cla.end(), clb.begin(),
                                        clb.end(), std::greater<T>());
}
/**
 * @brief Returns true if the first provided circular list is lexicographically less or equal to the second one
 * @param cla Circular list to compare
 * @param clb Circular list to compare
 * @return bool value, that signifies, that the first circular list is lexicographically less or equal to the second one
 */
template <typename T>
bool operator<=(const CircularList<T> &cla, const CircularList<T> &clb) {
    return !(cla > clb);
}
/**
 * @brief Returns true if the first provided circular list is lexicographically greater or equal to the second one
 * @param cla Circular list to compare
 * @param clb Circular list to compare
 * @return bool value, that signifies, that the first circular list is lexicographically greater or equal to the second one
 */
template <typename T>
bool operator>=(const CircularList<T> &cla, const CircularList<T> &clb) {
    return !(cla < clb);
}

} // namespace graphlib

#endif

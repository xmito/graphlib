#ifndef GRAPH_ITERATORS_H
#define GRAPH_ITERATORS_H

#include "Handle.h"
#include <iterator>

namespace graphlib {

template <typename Graph, typename Iterator, typename Traits>
class ForwardGraphIterator {
    using range_iterator = typename Traits::range_iterator;
    void increment() {
        if (cit_ != eit_)
            ++cit_;
        self().find_next();
    }

  protected:
    Graph *graph_{nullptr};
    range_iterator cit_{nullptr};
    range_iterator eit_{nullptr};
    Iterator &self() { return static_cast<Iterator &>(*this); }
    const Iterator &self() const {
        return static_cast<const Iterator &>(*this);
    }
    ForwardGraphIterator() = default;
    ForwardGraphIterator(Graph *graph, range_iterator bit, range_iterator eit)
        : graph_(graph), cit_(bit), eit_(eit) {
        self().find_next();
    }

  public:
    using value_type = typename Traits::value_type;
    using reference = typename Traits::reference;
    using pointer = typename Traits::pointer;
    using difference_type = typename Traits::difference_type;
    using iterator_category = std::forward_iterator_tag;

    Iterator &operator++() {
        increment();
        return self();
    }
    Iterator operator++(int) {
        auto copy(self());
        increment();
        return copy;
    }
    reference operator*() { return self().dereference(); }
    pointer operator->() { return &self().dereference(); }
    friend bool operator==(const ForwardGraphIterator &it1, const ForwardGraphIterator &it2) {
        return it1.graph_ == it2.graph_ && it1.cit_ == it2.cit_;
    }
    friend bool operator!=(const ForwardGraphIterator &it1, const ForwardGraphIterator &it2) {
        return !(it1 == it2);
    }
};

template <typename Iterator, typename Traits>
class ForwardRangeIterator {
    using range_iterator = typename Traits::range_iterator;
    void increment() {
        if (cit_ != eit_)
            ++cit_;
        self().find_next();
    }

  protected:
    range_iterator cit_{nullptr};
    range_iterator eit_{nullptr};
    Iterator &self() { return static_cast<Iterator &>(*this); }
    const Iterator &self() const {
        return static_cast<const Iterator &>(*this);
    }
    ForwardRangeIterator() = default;
    ForwardRangeIterator(range_iterator bit, range_iterator eit)
        : cit_(bit), eit_(eit) {
        self().find_next();
    }

  public:
    using value_type = typename Traits::value_type;
    using reference = typename Traits::reference;
    using pointer = typename Traits::pointer;
    using difference_type = typename Traits::difference_type;
    using iterator_category = std::forward_iterator_tag;

    Iterator &operator++() {
        increment();
        return self();
    }
    Iterator operator++(int) {
        auto copy(self());
        increment();
        return copy;
    }
    reference operator*() { return self().dereference(); }
    pointer operator->() { return &self().dereference(); }
    friend bool operator==(const ForwardRangeIterator &it1, const ForwardRangeIterator &it2) {
        return it1.cit_ == it2.cit_;
    }
    friend bool operator!=(const ForwardRangeIterator &it1, const ForwardRangeIterator &it2) {
        return !(it1 == it2);
    }
};

template <typename Traits>
class ForwardIterator {
    using range_iterator = typename Traits::range_iterator;
    range_iterator cit_;

  public:
    using value_type = typename Traits::value_type;
    using reference = typename Traits::reference;
    using pointer = typename Traits::pointer;
    using difference_type = typename Traits::difference_type;
    using iterator_category = typename Traits::iterator_category;

    ForwardIterator() = default;
    explicit ForwardIterator(range_iterator cit) : cit_(cit) {}
    ForwardIterator &operator++() {
        ++cit_;
        return *this;
    }
    ForwardIterator operator++(int) {
        auto cp(*this);
        ++cit_;
        return cp;
    }
    reference operator*() { return cit_->getHandle(); }
    pointer operator->() { return &cit_->getHandle(); }
    bool operator==(const ForwardIterator &it) const { return cit_ == it.cit_; }
    bool operator!=(const ForwardIterator &it) const { return !(*this == it); }
};

template <typename RangeIterator>
struct EdgeTraits {
    using value_type = const EdgeHandle;
    using reference = const EdgeHandle &;
    using pointer = const EdgeHandle *;
    using difference_type = std::ptrdiff_t;
    using iterator_category = std::forward_iterator_tag;
    using range_iterator = RangeIterator;
};

template <typename RangeIterator>
struct NodeTraits {
    using value_type = const NodeHandle;
    using reference = const NodeHandle &;
    using pointer = const NodeHandle *;
    using difference_type = std::ptrdiff_t;
    using iterator_category = std::forward_iterator_tag;
    using range_iterator = RangeIterator;
};

} // namespace graphlib

#endif

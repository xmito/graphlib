#ifndef FIBONACCIHEAP_H
#define FIBONACCIHEAP_H
#include "CircularList.h"
#include "Comparators.h"
#include "graph_traits.h"
#include <cassert>
#include <cstddef>
#include <functional>
#include <iostream>
#include <iterator>
#include <memory>
#include <unordered_map>

namespace graphlib {

/**
 * @brief FibonacciHeap is a priority queue used to hold graph node handles
 * @tparam Graph type of graph, that defines node_handle alias type
 * @tparam Compare Type of comparator used to compare two node_handles(defaults to LessDistance<Graph> class template instantiation)
 */
template <typename Graph, typename Compare = LessDistance<Graph>>
class FibonacciHeap {
    struct Node;

  public:
    /** Alias for node_handle type, that should be used as value_type */
    using value_type = typename graph_traits<Graph>::node_handle;
    /** Alias for chosen comparator in FibonacciHeap class template instantiation */
    using value_compare = Compare;
    /** Alias for reference to node_handle type */
    using reference = typename graph_traits<Graph>::node_handle &;
    /** Alias for constant reference to node_handle type */
    using const_reference = const typename graph_traits<Graph>::node_handle &;
    /** Alias for std::size_t */
    using size_type = std::size_t;
    /** Alias for type, that can represent a difference of two iterators */
    using difference_type =
        typename CircularList<Node>::iterator::difference_type;

  private:
    using iterator = typename CircularList<Node>::iterator;
    using const_iterator = typename CircularList<Node>::const_iterator;
    using list_container = CircularList<Node>;
    using node_handle_id = typename graph_traits<Graph>::node_handle::id_type;

    struct Node {
        bool mark_;
        value_type key_;
        iterator parent_;
        list_container clist_;

        explicit Node(const value_type &key)
            : mark_(false), key_(key), parent_(nullptr) {}
        Node(const value_type &key, const_iterator parent)
            : mark_(false), key_(key), parent_(parent) {}
        Node(const Node &node)
            : mark_(node.mark_), key_(node.key_), parent_(node.parent_),
              clist_(node.clist_) {}
        Node(Node &&node) noexcept
            : mark_(node.mark_), key_(std::move(node.key_)),
              parent_(node.parent_), clist_(std::move(node.clist_)) {}
        Node &operator=(const Node &node) {
            if (*this == node)
                return *this;
            mark_ = node.mark_;
            key_ = node.key_;
            parent_ = node.parent_;
            clist_ = node.clist_;
            return *this;
        }
        bool operator==(const Node &node) const { return key_ == node.key_; }
        bool operator!=(const Node &node) const { return !(*this == node); }
    };

  public:
    /**
     * @brief Constructs FibonacciHeap object from input range and comparator
     * @tparam InputIt Type of iterator used to provide first and last constraints of range
     * @param first iterator, which points to the first element of range
     * @param last iterator, which points to the last element of range
     * @param comp constant reference to comparator
     */
    template <typename InputIt>
    FibonacciHeap(InputIt first, InputIt last, const Compare &comp)
        : nodes_(0), top_(nullptr), comp_(comp) {
        while (first != last)
            push(*first++);
    }
    /**
     * @brief Constructs FibonacciHeap object from initializer_list and comparator
     * @param ilist initializer_list of node_handles
     * @param comp comparator used to compare node_handles
     */
    FibonacciHeap(std::initializer_list<value_type> ilist, const Compare &comp)
        : FibonacciHeap(ilist.begin(), ilist.end(), comp) {}
    /**
     * @brief Constructs FibonacciHeap object from comparator
     * @param comp comparator used to compare node_handles
     */
    explicit FibonacciHeap(const Compare &comp) : nodes_(0), comp_(comp) {}
    /**
     * @brief Constructs FibonacciHeap object from comparator
     * @param graph Graph type instance
     */
    explicit FibonacciHeap(const Graph &graph)
        : FibonacciHeap(graph.beginNode(), graph.endNode(), Compare(&graph)) {}
    /**
     * @brief Constructs FibonacciHeap object from Graph instance and comparator
     * @param graph Graph type instance
     * @param comp comparator used to compare node_handles
     */
    FibonacciHeap(const Graph &graph, const Compare &comp)
        : FibonacciHeap(graph.beginNode(), graph.endNode(), comp) {}
    /**
     * @brief Copy constructs FibonacciHeap object from another FibonacciHeap
     * @param fh FibonacciHeap instance to copy from
     */
    FibonacciHeap(const FibonacciHeap &fh)
        : nodes_(fh.nodes_), rlist_(fh.rlist_), comp_(fh.comp_), map_(fh.map_) {
        auto diff = std::distance(fh.rlist_.begin(), const_iterator(fh.top_));
        top_ = std::next(rlist_.begin(), diff);
    }
    /**
     * @brief Copy assign FibonacciHeap instance
     * @param fh FibonacciHeap instance to copy from
     */
    FibonacciHeap &operator=(const FibonacciHeap &fh) {
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
    /**
     * @brief Move constructs FibonacciHeap object from provided FibonacciHeap instance
     * @param fh FibonacciHeap instance to move from
     */
    FibonacciHeap(FibonacciHeap &&fh) noexcept
        : nodes_(fh.nodes_), top_(std::move(fh.top_)),
          rlist_(std::move(fh.rlist_)), comp_(std::move(fh.comp_)),
          map_(std::move(fh.map_)) {
        fh.nodes_ = 0;
        fh.top_ = nullptr;
    }
    /**
     * @brief Move assigns to FibonacciHeap object from provided FibonacciHeap instance
     * @param fh FibonacciHeap instance to move from
     */
    FibonacciHeap &operator=(FibonacciHeap &&fh) noexcept {
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

    /**
     * @brief Returns true if FibonacciHeap instance is empty
     * @return bool, that signifies emptiness
     */
    bool empty() const { return nodes_ == 0; }
    /**
     * @brief Returns number of nodes in FibonacciHeap instance
     * @return size_t count of nodes
     */
    size_t size() const { return nodes_; }
    /**
     * @brief Returns node_handle instance from the top of FibonacciHeap
     * @return const_reference to the top node_handle in FibonacciHeap instance
     */
    const_reference top() const {
        assert(top_);
        return const_iterator(top_)->key_;
    }

    /**
     * @brief Pushes provided node_handle to priority queue
     * @param nh Constant reference to node_handle, that should be pushed to priority queue
     */
    void push(const value_type &nh) {
        iterator node = rlist_.emplace_back(nh);
        if (!top_ || comp_(node->key_, top_->key_))
            top_ = node;
        ++nodes_;
        map_[nh.getId()] = node;
    }
    /**
     * @brief Pops the top node_handle from the priority queue
     */
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
    /**
     * @brief Restores FiboancciHeap properties given that provided node_handle breaks its invariant
     * @param nh Constant reference to node_handle, that breaks FibonacciHeap invariant
     */
    void decUpdate(const value_type &nh) {
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
        auto &pclist = parent->clist_;
        pclist.splice(pclist.end(), std::move(rlist_), node);
    }

    /* Consolidate all inserted nodes by merging those, that
     * have same degree. Therefore after consolidate, root list
     * has nodes, that each have different degree */
    void consolidate_() {
        iterator trees[64];
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
        iterator top(nullptr);
        for (auto &tree : trees) {
            if (tree) {
                if (!top || comp_(tree->key_, top->key_))
                    top = tree;
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

        if (parent->clist_.empty())
            return;
        for (auto &node : parent->clist_)
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
        rlist_.splice(rlist_.end(), std::move(parent->clist_));
    }

    /* cut_ - method removes a node from child list of
     * its parent and appends it to the root list */
    void cut_(iterator node) {
        assert(node);
        assert(node->parent_ != nullptr);

        iterator parent = node->parent_;
        unMarkUnParentChild_(*node);
        rlist_.splice(rlist_.end(), std::move(parent->clist_), node);
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

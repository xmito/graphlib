#ifndef BINARY_HEAP_H
#define BINARY_HEAP_H
#include <functional>
#include <vector>
#include <memory>
#include <algorithm>
#include <map>
#include <cassert>
#include "graph_traits.h"
#include "Comparators.h"

namespace graphlib {

/**
 * @brief BinaryHeap is a priority queue used to hold graph node handles
 * @tparam Graph Type of graph, that defines node_handle alias type
 * @tparam Compare Type of comparator used to compare two node_handles(defaults to LessDistance<Graph> class template instantiation)
 */
template <typename Graph, typename Compare = LessDistance<Graph>>
class BinaryHeap {
    using node_handle_id = typename graph_traits<Graph>::node_handle::id_type;

  public:
    struct Handle;
    /** Alias for node_handle type, that should be used as value_type */
    using value_type = typename graph_traits<Graph>::node_handle;
    /** Alias for chosen comparator in BinaryHeap class template instantiation */
    using value_compare = Compare;
    /** Alias for reference to node_handle type */
    using reference = typename graph_traits<Graph>::node_handle &;
    /** Alias for constant reference to node_handle type */
    using const_reference = const typename graph_traits<Graph>::node_handle &;
    /** Alias for std::vector<...>::size_type, which is able to store count of node_handles stored in BinaryHeap */
    using size_type = typename std::vector<value_type>::size_type;
    /** Alias for Handle structure */
    using heap_handle = Handle;

    /**
     * @brief The Handle is auxiliary type, that encapsulates pointer to id_type used to access heap_handle in BinaryHeap instance
     */
    struct Handle {
        /** Alias for unsigned integer */
        using id_type = size_t;
        /** @brief Constructs default Handle instance */
        Handle() = default;
        /**
         * @brief Compares this to provided Handle instance for equality
         * @param handle Constant reference to Handle
         * @return bool value, that signifies equality with provided Handle instance
         */
        bool operator==(const Handle &handle) const {
            return *index_ == *handle.index_;
        }
        /**
         * @brief Compares this to provided Handle instance for non-equality
         * @param handle Constant reference to Handle
         * @return bool value, that signifies non-equality to provided handle argument
         */
        bool operator!=(const Handle &handle) const {
            return !(*this == handle);
        }
        /**
         * @brief Retrieves id_type numeric value
         * @return id_type pointing to BinaryHeap heap_handle storage
         */
        id_type getId() const { return *index_; }

      private:
        friend class BinaryHeap;
        id_type *index_;
        explicit Handle(id_type *index) : index_(index) {}
    };

    /**
     * @brief Constructs BinaryHeap object from input range and comparator
     * @tparam InputIt Type of iterator used to provide first and last constraints of range
     * @param first iterator, which points to the first element of range
     * @param last iterator, which points to the last element of range
     * @param comp constant reference to comparator
     */
    template <typename InputIt>
    BinaryHeap(InputIt first, InputIt last, const Compare &comp) : comp_(comp) {
        size_t idx = 0;
        vec_.reserve(std::distance(first, last));
        while (first != last) {
            const Element &elem = vec_.emplace_back(*first, idx++);
            map_.emplace(first->getId(), elem.getHandle());
            ++first;
        }
        buildHeap_();
    }
    /**
     * @brief Constructs BinaryHeap object from initializer_list and comparator
     * @param ilist initializer_list of node_handles
     * @param comp comparator used to compare node_handles
     */
    BinaryHeap(std::initializer_list<value_type> ilist, const Compare &comp)
        : BinaryHeap(ilist.begin(), ilist.end(), comp) {}
    /**
     * @brief Constructs BinaryHeap object from comparator
     * @param comp comparator used to compare node_handles
     */
    explicit BinaryHeap(const Compare &comp) : comp_(comp) {}
    /**
     * @brief Constructs BinaryHeap object from Graph instance
     * @param graph Graph type instance
     */
    explicit BinaryHeap(const Graph &graph)
        : BinaryHeap(graph.beginNode(), graph.endNode(), Compare(&graph)) {}
    /**
     * @brief Constructs BinaryHeap object from Graph instance and comparator
     * @param graph Graph type instance
     * @param comp comparator used to compare node_handles
     */
    BinaryHeap(const Graph &graph, const Compare &comp)
        : BinaryHeap(graph.beginNode(), graph.endNode(), comp) {}
    /** @brief Copy constructor is not allowed */
    BinaryHeap(const BinaryHeap &) = delete;
    /** @brief Copy assignment is not allowed*/
    BinaryHeap &operator=(const BinaryHeap &) = delete;
    /** @brief Default move construction */
    BinaryHeap(BinaryHeap &&) noexcept = default;
    /** @brief Default move assignment */
    BinaryHeap &operator=(BinaryHeap &&) noexcept = default;

    /**
     * @brief Returns node_handle instance from the top of BinaryHeap
     * @return const_reference to the top node_handle in BinaryHeap instance
     */
    const_reference top() const {
        assert(!empty());
        return vec_[0].value_;
    }
    /**
     * @brief Returns heap_handle instance of the top BinaryHeap element
     * @return heap_handle to the top BinaryHeap element
     */
    heap_handle topHandle() const {
        assert(!empty());
        return vec_[0].getHandle();
    }
    /**
     * @brief Returns node_handle from provided heap_handle
     * @param h constant reference to heap_handle for which node_handle should be retrieved
     * @return const_reference to node_handle corresponding to the provided heap_handle
     */
    const_reference get(const heap_handle &h) const {
        assert(*h.index_ < vec_.size());
        return vec_[*h.index_].value_;
    }
    /**
     * @brief Returns heap_handle from provided node_handle
     * @param nh Constant reference to node_handle for which heap_handle should be retrieved
     * @return heap_handle to corresponding node_handle
     */
    heap_handle getHandle(const value_type &nh) {
        assert(nh.getId() < map_.size());
        return map_[nh.getId()];
    }
    /**
     * @brief Returns true if BinaryHeap instance is empty
     * @return bool value, that signifies emptiness of the priority queue
     */
    bool empty() const { return vec_.empty(); }
    /**
     * @brief Returns count of currently stored node_handles
     * @return Count of node_handles in the priority queue
     */
    size_type size() const { return vec_.size(); }
    /**
     * @brief Pushes provided node_handle to priority queue
     * @param nh Constant reference to node_handle, that should be pushed to priority queue
     * @return heap_handle of the pushed node_handle
     */
    heap_handle push(const value_type &nh) {
        size_t idx = vec_.size();
        const Element &elem = vec_.emplace_back(nh, idx);
        map_[nh.getId()] = elem.getHandle();
        return upReheap_(idx);
    }
    /**
     * @brief Pops the top node_handle from the priority queue
     */
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
    /**
     * @brief Swaps all elements with the provided heap
     * @param heap Reference to BinaryHeap instance, with which this instance should be swapped
     */
    void swap(BinaryHeap &heap) {
        using std::swap;
        vec_.swap(heap.vec_);
        map_.swap(heap.map_);
        std::swap(comp_, heap.comp_);
    }
    /**
     * @brief Restores heap property if provided node_handle breaks it
     * @param nh Constant reference to node_handle, that may or may not break heap property
     * @return heap_handle of the provided node_handle
     */
    heap_handle decUpdate(const value_type &nh) {
        return decUpdate(map_[nh.getId()]);
    }
    /**
     * @brief Restores heap property if the provided heap_handle breaks it
     * @param handle Constant reference to heap_handle, which corresponding node_handle may violate heap property
     * @return heap_handle for the provided heap_handle argument after heap property is restored
     */
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
    /**
     * @brief Compares two BinaryHeap instances
     * @param bha Constant reference to BinaryHeap instance
     * @param bhb Constant reference to BinaryHeap instance
     * @return bool value, that signifies equality of provided arguments
     */
    friend bool operator==(const BinaryHeap &bha, const BinaryHeap &bhb) {
        return bha.vec_ == bhb.vec_;
    }
    /**
     * @brief Compares two BinaryHeap instances
     * @param bha Constant reference to BinaryHeap instance
     * @param bhb Constant reference to BinaryHeap instance
     * @return bool value, that signifies inequality of provided arguments
     */
    friend bool operator!=(const BinaryHeap &bha, const BinaryHeap &bhb) {
        return !(bha == bhb);
    }

  private:
    struct Element {
        Element(const value_type &value, size_t index)
            : value_(value), index_(std::make_unique<size_t>(index)) {}
        Element(value_type &&value, size_t index)
            : value_(std::move(value)),
              index_(std::make_unique<size_t>(index)) {}
        Element(const Element &) = delete;
        Element &operator=(const Element &) = delete;
        Element(Element &&) noexcept = default;
        Element &operator=(Element &&elem) noexcept = default;
        heap_handle getHandle() const { return heap_handle(index_.get()); }
        void swap(Element &elem) {
            using std::swap;
            swap(value_, elem.value_);
            index_.swap(elem.index_);
            swap(*index_, *elem.index_);
        }
        bool operator==(const Element &elem) const {
            return value_ == elem.value_;
        }
        bool operator!=(const Element &elem) const { return !(*this == elem); }

        value_type value_;
        std::unique_ptr<size_t> index_;
    };

    size_t up_(size_t child) { return (child - 1) / 2; }
    size_t left_(size_t parent) { return 2 * parent + 1; }
    size_t right_(size_t parent) { return 2 * parent + 2; }

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
    void buildHeap_() {
        for (int i = vec_.size() / 2 - 1; i >= 0; --i)
            bottomUpReheap_(i);
    }

    std::vector<Element> vec_;
    std::map<node_handle_id, heap_handle> map_;
    Compare comp_;
};

} // namespace graphlib
#endif

#ifndef LIST_GRAPH_H
#define LIST_GRAPH_H
#include <algorithm>
#include <boost/range.hpp>
#include <cassert>
#include <list>
#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "EdgeData.h"
#include "GraphIterators.h"
#include "Handle.h"
#include "NodeData.h"
#include "edge_traits.h"
#include "graph_traits.h"
#include "node_traits.h"

namespace graphlib {

/**
 * @brief ListGraph is a data structure, which can hold undirected graphs. Internally, it represents node adjacency using lists. Basic operations on ListGraph have the following complexity:
 * addNode : O(1) amortized
 * addEdge : O(1) amortized
 * removeNode : O(V) worst case
 * removeEdge : O(V) worst case
 *
 * addNode as well as addEdge have O(1) amortized complexity, because ListGraph internally uses std::vector to store nodes and edges. Creating entry in vector map, that maps node ids to edge lists has constant amortized complexity, too.
 *
 * removeNode is able to work in time linear in |V| using lazy approach. When a node is removed, all of its edges are marked invalid, or otherwise if the other node at the end of edge was already deleted, it is removed immediately.
 *
 * removeEdge has worst case complexity O(V), because edge removal involves only deleting edge handle from adjacency lists of both nodes and additional O(1) removal of edge structure from edge vector.
 * @tparam NodeData Data to store inside node structure
 * @tparam EdgeData Data to store inside edge structure
 */
template <typename NodeData, typename EdgeData>
class ListGraph {
    struct Node {
        template <typename... Args>
        explicit Node(node_id nid, Args &&...args)
            : nid_(std::make_unique<node_id>(nid)),
              data_(std::forward<Args>(args)...) {
            handle_.id_ = nid_.get();
        }
        bool operator==(const Node &node) {
            return nid_ == node.nid_ && data_ == node.data_;
        }
        bool operator!=(const Node &node) { return !(*this == node); }
        void swap(Node &node) {
            if (this == &node)
                return;
            using std::swap;
            swap(data_, node.data_);
            swap(handle_, node.handle_);
            nid_.swap(node.nid_);
            swap(*nid_, *node.nid_);
        }
        const NodeHandle &getHandle() const { return handle_; }
        using handle_type = NodeHandle;

      private:
        NodeHandle handle_;
        std::unique_ptr<node_id> nid_;
        NodeData data_;

        friend class ListGraph;
    };
    struct Edge {
        template <typename... Args>
        Edge(const NodeHandle &fst, const NodeHandle &snd, edge_id eid,
             Args &&...args)
            : valid_(true), fst_(fst), snd_(snd),
              eid_(std::make_unique<edge_id>(eid)),
              data_(std::forward<Args>(args)...) {
            handle_.id_ = eid_.get();
        }
        bool operator==(const Edge &edge) const {
            return eid_ == edge.eid_ && data_ == edge.data_;
        }
        bool operator!=(const Edge &edge) const { return !(*this == edge); }
        void swap(Edge &edge) {
            if (this == &edge)
                return;
            using std::swap;
            swap(data_, edge.data_);
            eid_.swap(edge.eid_);
            swap(*eid_, *edge.eid_);
            swap(fst_, edge.fst_);
            swap(snd_, edge.snd_);
            swap(handle_, edge.handle_);
            swap(last_valid_, edge.last_valid_);
            swap(valid_, edge.valid_);
        }
        const EdgeHandle &getHandle() const { return handle_; }
        using handle_type = EdgeHandle;

      private:
        bool valid_;
        NodeHandle fst_;
        NodeHandle snd_;
        NodeHandle last_valid_;
        EdgeHandle handle_;
        std::unique_ptr<edge_id> eid_;
        EdgeData data_;

        friend class ListGraph;
    };

    using erange_iterator = typename std::vector<Edge>::iterator;
    using cerange_iterator = typename std::vector<Edge>::const_iterator;
    using nrange_iterator = typename std::vector<Node>::iterator;
    using cnrange_iterator = typename std::vector<Node>::const_iterator;

    using EItTraits = EdgeTraits<erange_iterator>;
    template <typename Iterator>
    using EItBase = ForwardGraphIterator<ListGraph, Iterator, EItTraits>;

    class EdgeIterator : public EItBase<EdgeIterator> {
        using base_iterator = EItBase<EdgeIterator>;
        using base_iterator::cit_;
        using base_iterator::eit_;
        using base_iterator::graph_;

      public:
        using value_type = typename EItTraits::value_type;
        using reference = typename EItTraits::reference;
        using pointer = typename EItTraits::pointer;
        using difference_type = typename EItTraits::difference_type;
        using iterator_category = typename EItTraits::iterator_category;

        EdgeIterator() = default;
        EdgeIterator(ListGraph *graph, erange_iterator bit, erange_iterator eit)
            : base_iterator(graph, bit, eit) {}
        template <typename OIterator>
        bool operator==(const OIterator &it) const {
            return graph_ == it.graph_ && cit_ == it.cit_;
        }
        template <typename OIterator>
        bool operator!=(const OIterator &it) const {
            return !(*this == it);
        }

      private:
        void find_next() {
            while (cit_ != eit_ && !graph_->hasEdge(cit_->getHandle())) {
                EdgeHandle eh = cit_->getHandle();
                ++cit_;
                graph_->removeEdge(eh);
            }
        }
        reference dereference() const { return cit_->getHandle(); }
        template <typename, typename, typename>
        friend class ForwardGraphIterator;
        friend class ConstEdgeIterator;
    };

    using CEItTraits = EdgeTraits<cerange_iterator>;
    template <typename Iterator>
    using CEItBase = ForwardGraphIterator<const ListGraph, Iterator, CEItTraits>;

    class ConstEdgeIterator : public CEItBase<ConstEdgeIterator> {
        using base_iterator = CEItBase<ConstEdgeIterator>;
        using base_iterator::cit_;
        using base_iterator::eit_;
        using base_iterator::graph_;

      public:
        using value_type = typename CEItTraits::value_type;
        using reference = typename CEItTraits::reference;
        using pointer = typename CEItTraits::pointer;
        using difference_type = typename CEItTraits::difference_type;
        using iterator_category = typename CEItTraits::iterator_category;

        ConstEdgeIterator() = default;
        ConstEdgeIterator(const ListGraph *graph, cerange_iterator bit,
                          cerange_iterator eit)
            : base_iterator(graph, bit, eit) {}
        explicit ConstEdgeIterator(const EdgeIterator &it)
            : base_iterator(it.graph_, it.cit_, it.eit_) {}
        ConstEdgeIterator &operator=(const EdgeIterator &it) {
            graph_ = it.graph_;
            cit_ = it.cit_;
            eit_ = it.eit_;
            return *this;
        }
        template <typename OIterator>
        bool operator==(const OIterator &it) const {
            return graph_ == it.graph_ && cit_ == it.cit_;
        }
        template <typename OIterator>
        bool operator!=(const OIterator &it) const {
            return !(*this == it);
        }

      private:
        void find_next() {
            while (cit_ != eit_ && !graph_->hasEdge(cit_->getHandle()))
                ++cit_;
        }
        reference dereference() const { return cit_->getHandle(); }
        template <typename, typename, typename>
        friend class ForwardGraphIterator;
        friend class EdgeIterator;
    };

    template <typename T>
    class ListWrapper {
        ListGraph *graph_{nullptr};
        std::list<T> list_;
        using list_iterator = typename std::list<T>::const_iterator;

        using WrapItTraits = EdgeTraits<list_iterator>;
        template <typename Iterator>
        using WrapItBase = ForwardGraphIterator<ListGraph, Iterator, WrapItTraits>;

        class WrapIterator : public WrapItBase<WrapIterator> {
            using base_iterator = WrapItBase<WrapIterator>;
            using base_iterator::cit_;
            using base_iterator::eit_;
            using base_iterator::graph_;

          public:
            using value_type = typename WrapItTraits::value_type;
            using reference = typename WrapItTraits::reference;
            using pointer = typename WrapItTraits::pointer;
            using difference_type = typename WrapItTraits::difference_type;
            using iterator_category = typename WrapItTraits::iterator_category;

            WrapIterator() = default;
            WrapIterator(ListGraph *graph, list_iterator bit, list_iterator eit)
                : base_iterator(graph, bit, eit) {}

          private:
            void find_next() {
                while (cit_ != eit_ && !graph_->hasEdge(*(cit_))) {
                    EdgeHandle eh = *(cit_);
                    ++cit_;
                    graph_->removeEdge(eh);
                }
            }
            reference dereference() const { return *cit_; }
            template <typename, typename, typename>
            friend class ForwardGraphIterator;
        };

      public:
        using value_type = typename std::list<T>::value_type;
        using reference = typename std::list<T>::reference;
        using const_reference = typename std::list<T>::const_reference;
        using iterator = WrapIterator;
        using const_iterator = WrapIterator;
        using difference_type =
            typename std::iterator_traits<list_iterator>::difference_type;
        using size_type = typename std::list<T>::size_type;

        ListWrapper() = default;
        explicit ListWrapper(ListGraph *graph) : graph_(graph) {}
        iterator begin() const {
            return iterator(graph_, list_.begin(), list_.end());
        }
        iterator end() const { return iterator(graph_, list_.end(), list_.end()); }
        size_type size() const {
            size_type valid = 0;
            for (auto it = list_.begin(); it != list_.end(); ++it) {
                if (graph_->hasEdge(*it))
                    ++valid;
            }
            return valid;
        }
        bool empty() const { return size() == 0; }

      private:
        void push_back(const T &val) { list_.push_back(val); }
        void push_back(T &&val) { list_.push_back(std::move(val)); }
        void erase(list_iterator it) { list_.erase(it); }

        template <typename, typename>
        friend class ListGraph;
    };

    std::vector<Node> nodes_;
    std::vector<Edge> edges_;
    std::vector<std::unique_ptr<ListWrapper<EdgeHandle>>> edges_map_;
    size_t valid_edges_{0};

  public:
    /** directedTag constant signifies, that ListGraph is undirected graph */
    static const bool directedTag = false;
    /** weightedTag constant signifies, that ListGraph is weighted depending on EdgeData template argument */
    static const bool weightedTag = edge_traits<EdgeData>::weighted;
    /** traversableTag constant signifies, that ListGraph has color and predecessor member variables in node data. It means, the graph can be used with basic algorithms, that do traversing on graph. */
    static const bool traversableTag =
        node_traits<NodeData>::color && node_traits<NodeData>::predecessor;
    /** pathTag constant signifies, that graph fulfills traversableTag and it also has in node data member variable able to store distance. It means, the graph can be used with algorithms, that compute distances and lengths of paths */
    static const bool pathTag = traversableTag && node_traits<NodeData>::distance;
    /** heuristicpathTag constant signifies, that graph fulfills pathTag and it also has in node data member variable able to store information about node's location. This can be used to run heuristic algorithms on the graph */
    static const bool heuristicpathTag =
        pathTag && node_traits<NodeData>::location;

    /** Alias for node handle type, that stores pointer to node_id. Internally, node_id value is used to access node structure in node vector. */
    using node_handle = NodeHandle;
    /** Alias for edge handle type, that stores pointer to edge_id. Internally, edge_id value is used to access edge structure in edge vector. */
    using edge_handle = EdgeHandle;
    /** Alias for std::list wrapper, that helps to skip edges having only one valid end node. */
    using adj_range = ListWrapper<EdgeHandle>;
    /** Alias for std::list wrapper's forward iterator, that implicitely skips edges with only one valid end node, when incremented */
    using adj_iterator = typename ListWrapper<EdgeHandle>::iterator;
    /** Alias for std::list wrapper's constant forward iterator, that implicitely skips edges with only one valid end node, when incremented */
    using const_adj_iterator = typename ListWrapper<EdgeHandle>::const_iterator;
    /** Alias for ListGraph's NodeData template argument */
    using node_data = NodeData;
    /** Alias for ListGraph's EdgeData template argument */
    using edge_data = EdgeData;
    /** Alias for forward iterator used to iterate over nodes */
    using node_iterator = ForwardIterator<NodeTraits<cnrange_iterator>>;
    /** Alias for constant forward iterator used to iterate over nodes */
    using const_node_iterator = ForwardIterator<NodeTraits<cnrange_iterator>>;
    /** Alias for forward iterator used to iterate over edges */
    using edge_iterator = EdgeIterator;
    /** Alias for constant forward iterator used to iterate over edges */
    using const_edge_iterator = ConstEdgeIterator;
    /** Alias for type used to represent edge weights */
    using weight_type = typename edge_traits<EdgeData>::weight_type;
    /** Alias for type used to represent distance between nodes */
    using distance_type = typename node_traits<NodeData>::distance_type;
    /** Alias for type used to represent node's location for heuristic algorithms */
    using location_type = typename node_traits<NodeData>::location_type;
    /** Alias for type used to store priority computed by heuristic algorithms */
    using priority_type = typename node_traits<NodeData>::priority_type;

    /**
     * @brief Default constructor. Constructs empty undirected graph
     */
    ListGraph() = default;
    /**
     * @brief Constructs undirected graph with number of nodes
     * @param nonodes Number of nodes to create
     */
    explicit ListGraph(size_t nonodes) {
        while (nonodes--)
            addNode();
    }
    /**
     * @brief Deleted copy constructor
     */
    ListGraph(const ListGraph &) = delete;
    /**
     * @brief Deleted copy assignment
     */
    ListGraph &operator=(const ListGraph &) = delete;

    /**
     * @brief Returns true if there is an edge between two nodes
     * @param nha Node handle to the source node
     * @param nhb Node handle to the target node
     * @return bool value, that signifies presence of edge
     */
    bool hasEdge(const NodeHandle &nha, const NodeHandle &nhb) const {
        if (*nha.id_ >= nodes_.size() && *nhb.id_ >= nodes_.size())
            return false;
        auto &list = *edges_map_[*nha.id_];
        auto eit =
            std::find_if(list.begin(), list.end(), [&, this](const EdgeHandle &eh) {
                return nhb == getOther(eh, nha);
            });
        return eit != list.end();
    }
    /**
     * @brief Returns true if there is an edge with provided edge handle
     * @param eh Edge handle to verify
     * @return bool value, that signifies presence of edge
     */
    bool hasEdge(const EdgeHandle &eh) const {
        return (*eh.id_ < edges_.size() && edges_[*eh.id_].valid_) ? true : false;
    }
    /**
     * @brief Returns true if there is a node with provided node handle
     * @param nh Node handle to verify
     * @return bool value, that signifies existence of such node
     */
    bool hasNode(const NodeHandle &nh) const {
        return (*nh.id_ < nodes_.size()) ? true : false;
    }
    /**
     * @brief Returns a degree of node specified by provided node handle
     * @param nh Valid node handle
     */
    size_t degree(const NodeHandle &nh) const {
        assert(*nh.id_ < nodes_.size());
        return edges_map_[*nh.id_]->size();
    }
    /**
     * @brief Returns edge data to provided edge handle
     * @param eh Valid edge handle
     * @return Reference to EdgeData stored inside edge
     */
    EdgeData &getEdge(const EdgeHandle &eh) {
        assert(*eh.id_ < edges_.size() && edges_[*eh.id_].valid_);
        return edges_[*eh.id_].data_;
    }
    /**
     * @brief Returns edge data to provided edge handle
     * @param eh Valid edge handle
     * @return Constant reference to EdgeData stored inside edge
     */
    const EdgeData &getEdge(const EdgeHandle &eh) const {
        assert(*eh.id_ < edges_.size() && edges_[*eh.id_].valid_);
        return edges_[*eh.id_].data_;
    }
    /**
     * @brief Returns node data to provided node handle
     * @param nh Valid node handle
     * @return Reference to NodeData stored inside node
     */
    NodeData &getNode(const NodeHandle &nh) {
        assert(*nh.id_ < nodes_.size());
        return nodes_[*nh.id_].data_;
    }
    /**
     * @brief Returns node data to provided node handle
     * @param nh Valid node handle
     * @return Constant reference to NodeData stored inside node
     */
    const NodeData &getNode(const NodeHandle &nh) const {
        assert(*nh.id_ < nodes_.size());
        return nodes_[*nh.id_].data_;
    }
    /**
     * @brief Returns node handle to end node, given the other edge end node was provided as argument
     * @param eh Valid edge handle
     * @param nh Valid node handle of node, which is present on one end of the edge
     * @return Node handle to the other end node
     */
    NodeHandle getOther(const EdgeHandle &eh, const NodeHandle &nh) const {
        assert(*eh.id_ < edges_.size() && *nh.id_ < nodes_.size() &&
               edges_[*eh.id_].valid_);
        return edges_[*eh.id_].fst_ == nh ? edges_[*eh.id_].snd_
                                          : edges_[*eh.id_].fst_;
    }
    /**
     * @brief Returns a pair of node handles, that are connected by the provided edge
     * @param eh Valid edge handle
     * @return Pair of node handles
     */
    std::pair<NodeHandle, NodeHandle> getBoth(const EdgeHandle &eh) const {
        assert(*eh.id_ < edges_.size() && edges_[*eh.id_].valid_);
        return std::make_pair(edges_[*eh.id_].fst_, edges_[*eh.id_].snd_);
    }
    /**
     * @brief Returns node data associated with node, which is connected to the node with provided node handle by edge with provided edge handle
     * @param eh Valid edge handle
     * @param nh Valid node handle
     * @return Reference to node data associated with the other end node
     */
    NodeData &getOtherNode(const EdgeHandle &eh, const NodeHandle &nh) {
        assert(*eh.id_ < edges_.size() && *nh.id_ < nodes_.size() &&
               edges_[*eh.id_].valid_);
        const NodeHandle &other = getOther(eh, nh);
        return nodes_[*other.id_].data_;
    }
    /**
     * @brief Returns node data associated with node, which is connected to the node with provided node handle by edge with provided edge handle
     * @param eh Valid edge handle
     * @param nh Valid node handle
     * @return Constant reference to node data associated with the other end node
     */
    const NodeData &getOtherNode(const EdgeHandle &eh,
                                 const NodeHandle &nh) const {
        assert(*eh.id_ < edges_.size() && *nh.id_ < nodes_.size() &&
               edges_[*eh.id_].valid_);
        const NodeHandle &other = getOther(eh, nh);
        return nodes_[*other.id_].data_;
    }
    /**
     * @brief Returns a list wrapper of edge handles corresponding to edges, that are connected to node with provided node handle
     * @param nh Valid node handle
     * @return Reference to list wrapper of edge handles
     */
    adj_range &operator[](const NodeHandle &nh) {
        assert(*nh.id_ < nodes_.size());
        return *edges_map_[*nh.id_];
    }
    /**
     * @brief Returns a list wrapper of edge handles corresponding to edges, that have source in node specified by provided node handle
     * @param nh Valid node handle
     * @return Constant reference to list wrapper of edge handles
     */
    const adj_range &operator[](const NodeHandle &nh) const {
        assert(*nh.id_ < nodes_.size());
        return *edges_map_[*nh.id_];
    }
    /**
     * @brief Returns count of nodes in the graph
     * @return Count of nodes
     */
    node_id nodeCount() const { return nodes_.size(); }
    /**
     * @brief Returns count of edges in the graph
     * @return Count of edges
     */
    edge_id edgeCount() const { return valid_edges_; }
    /**
     * @brief Returns iterator to the first node
     * @return Node iterator
     */
    node_iterator beginNode() { return node_iterator(nodes_.begin()); }
    /**
     * @brief Returns constant iterator to the first node
     * @return Constant node iterator
     */
    const_node_iterator beginNode() const {
        return const_node_iterator(nodes_.begin());
    }
    /**
     * @brief Returns constant iterator to the first node
     * @return Constant node iterator
     */
    const_node_iterator cbeginNode() const {
        return const_node_iterator(nodes_.cbegin());
    }
    /**
     * @brief Returns iterator to the node following the last node of the graph
     * @return Node iterator
     */
    node_iterator endNode() { return node_iterator(nodes_.end()); }
    /**
     * @brief Returns constant iterator to the node following the last node of the graph
     * @return Constant node iterator
     */
    const_node_iterator endNode() const {
        return const_node_iterator(nodes_.end());
    }
    /**
     * @brief Returns constant iterator to the node following the last node of the graph
     * @return Constant node iterator
     */
    const_node_iterator cendNode() const {
        return const_node_iterator(nodes_.cend());
    }

    /**
     * @brief Returns iterator to the first edge
     * @return Edge iterator
     */
    edge_iterator beginEdge() {
        return edge_iterator(this, edges_.begin(), edges_.end());
    }
    /**
     * @brief Returns constant iterator to the first edge
     * @return Constant edge iterator
     */
    const_edge_iterator beginEdge() const {
        return const_edge_iterator(this, edges_.begin(), edges_.end());
    }
    /**
     * @brief Returns constant iterator to the first edge
     * @return Constant edge iterator
     */
    const_edge_iterator cbeginEdge() const {
        return const_edge_iterator(this, edges_.cbegin(), edges_.cend());
    }
    /**
     * @brief Returns iterator to the edge following the last edge of the graph
     * @return Edge iterator
     */
    edge_iterator endEdge() {
        return edge_iterator(this, edges_.end(), edges_.end());
    }
    /**
     * @brief Returns constant iterator to the edge following the last edge of the graph
     * @return Constant edge iterator
     */
    const_edge_iterator endEdge() const {
        return const_edge_iterator(this, edges_.end(), edges_.end());
    }
    /**
     * @brief Returns constant iterator to the edge following the last edge of the graph
     * @return Constant edge iterator
     */
    const_edge_iterator cendEdge() const {
        return const_edge_iterator(this, edges_.cend(), edges_.cend());
    }

    /**
     * @brief Returns boost::iterator_range of begin and end node iterators
     * @return Range of node iterators
     */
    boost::iterator_range<node_iterator> nodes() {
        return boost::make_iterator_range(beginNode(), endNode());
    }
    /**
     * @brief Returns boost::iterator_range of begin and end constant node iterators
     * @return Range of constant node iterators
     */
    boost::iterator_range<const_node_iterator> nodes() const {
        return boost::make_iterator_range(beginNode(), endNode());
    }
    /**
     * @brief Returns boost::iterator_range of begin and end constant node iterators
     * @return Range of constant node iterators
     */
    boost::iterator_range<const_node_iterator> cnodes() const {
        return boost::make_iterator_range(cbeginNode(), cendNode());
    }
    /**
     * @brief Returns boost::iterator_range of begin and end edge iterators
     * @return Range of edge iterators
     */
    boost::iterator_range<edge_iterator> edges() {
        return boost::make_iterator_range(beginEdge(), endEdge());
    }
    /**
     * @brief Returns boost::iterator_range of begin and end constant edge iterators
     * @return Range of constant edge iterators
     */
    boost::iterator_range<const_edge_iterator> edges() const {
        return boost::make_iterator_range(beginEdge(), endEdge());
    }
    /**
     * @brief Returns boost::iterator_range of begin and end constant edge iterators
     * @return Range of constant edge iterators
     */
    boost::iterator_range<const_edge_iterator> cedges() const {
        return boost::make_iterator_range(cbeginEdge(), cendEdge());
    }

    /**
     * @brief Sets weight of edge to provided value
     * @param eh Valid edge handle
     * @param weight Weight to set
     */
    template <typename EData = EdgeData,
              typename = std::enable_if_t<EData::weighted>>
    void setWeight(const EdgeHandle &eh,
                   typename edge_traits<EData>::weight_type weight) {
        assert(*eh.id_ < edges_.size());
        auto &data = getEdge(eh);
        data.weight_ = weight;
    }
    /**
     * @brief Modifies weight of edge by provided value
     * @param eh Valid edge handle
     * @param weight Weight by which should be edge weight modified
     */
    template <typename EData = EdgeData,
              typename = std::enable_if_t<EData::weighted>>
    void modWeight(const EdgeHandle &eh,
                   typename edge_traits<EData>::weight_type weight) {
        assert(*eh.id_ < edges_.size());
        auto &edata = getEdge(eh);
        edata.weight_ += weight;
    }
    /**
     * @brief Returns edge weight to provided edge handle
     * @param eh Valid edge handle
     * @return Edge weight
     */
    template <typename EData = EdgeData,
              typename = std::enable_if_t<EData::weighted>>
    typename EData::weight_type getWeight(const EdgeHandle &eh) const {
        assert(*eh.id_ < edges_.size());
        auto &data = getEdge(eh);
        return data.weight_;
    }
    /**
     * @brief Returns color to provided node handle
     * @param nh Valid node handle
     * @return Color of node specified by node handle
     */
    template <typename Graph = ListGraph<NodeData, EdgeData>,
              typename = std::enable_if_t<Graph::traversableTag>>
    Color getNodeColor(const node_handle &nh) const {
        assert(*nh.id_ < nodes_.size());
        auto &data = getNode(nh);
        return data.color_;
    }
    /**
     * @brief Sets color of provided node
     * @param nh Valid node handle
     * @param color Color to be set
     */
    template <typename Graph = ListGraph<NodeData, EdgeData>,
              typename = std::enable_if_t<Graph::traversableTag>>
    void setNodeColor(const node_handle &nh, Color color) {
        assert(*nh.id_ < nodes_.size());
        auto &data = getNode(nh);
        data.color_ = color;
    }
    /**
     * @brief Returns node handle to the node predecessor
     * @param nh Valid node handle
     * @return Node handle to the predecessor
     */
    template <typename Graph = ListGraph<NodeData, EdgeData>,
              typename = std::enable_if_t<Graph::traversableTag>>
    node_handle
    getNodePred(const typename graph_traits<Graph>::node_handle &nh) const {
        assert(*nh.id_ < nodes_.size());
        auto &data = getNode(nh);
        return data.pred_;
    }
    /**
     * @brief Sets predecessor to node specified by provided node handle
     * @param nh Valid node handle
     * @param pred Valid node handle to predecessor
     */
    template <typename Graph = ListGraph<NodeData, EdgeData>,
              typename = std::enable_if_t<Graph::traversableTag>>
    void setNodePred(const typename graph_traits<Graph>::node_handle &nh,
                     const typename graph_traits<Graph>::node_handle &pred) {
        assert(*nh.id_ < nodes_.size());
        auto &data = getNode(nh);
        data.pred_ = pred;
    }
    /**
     * @brief Returns distance value associated with node specified by provided node handle.
     * @param nh Valid node handle
     * @return Distance associated with node
     */
    template <typename Graph = ListGraph<NodeData, EdgeData>,
              typename = std::enable_if_t<Graph::pathTag>>
    typename graph_traits<Graph>::distance_type
    getNodeDist(const typename graph_traits<Graph>::node_handle &nh) const {
        assert(*nh.id_ < nodes_.size());
        auto &data = getNode(nh);
        return data.dist_;
    }
    /**
     * @brief Sets distance value to node specified by provided node handle
     * @param nh Valid node handle
     * @param dist Distance to be set
     */
    template <typename Graph = ListGraph<NodeData, EdgeData>,
              typename = std::enable_if_t<Graph::pathTag>>
    void setNodeDist(const typename graph_traits<Graph>::node_handle &nh,
                     typename graph_traits<Graph>::distance_type dist) {
        assert(*nh.id_ < nodes_.size());
        auto &data = getNode(nh);
        data.dist_ = dist;
    }
    /**
     * @brief Returns location_type instance of node specified by provided node handle
     * @param nh Valid node handle
     * @return Location data structure used by heuristic algorithms
     */
    template <typename Graph = ListGraph<NodeData, EdgeData>,
              typename = std::enable_if_t<Graph::heuristicpathTag>>
    const typename graph_traits<Graph>::location_type &
    getNodeLoc(const typename graph_traits<Graph>::node_handle &nh) const {
        assert(*nh.id_ < nodes_.size());
        auto &data = getNode(nh);
        return data.loc_;
    }
    /**
     * @brief Sets location data of node specified by node handle to provided value
     * @param nh Valid node handle
     * @param loc Location data
     */
    template <typename Graph = ListGraph<NodeData, EdgeData>,
              typename = std::enable_if_t<Graph::heuristicpathTag>>
    void setNodeLoc(const typename graph_traits<Graph>::node_handle &nh,
                    const typename graph_traits<Graph>::location_type &loc) {
        assert(*nh.id_ < nodes_.size());
        auto &data = getNode(nh);
        data.loc_ = loc;
    }
    /**
     * @brief Returns priority value of node specified by provided node handle
     * @param nh Valid node handle
     * @return Priority value stored in node data
     */
    template <typename Graph = ListGraph<NodeData, EdgeData>,
              typename = std::enable_if_t<Graph::heuristicpathTag>>
    typename graph_traits<Graph>::priority_type
    getNodePrio(const typename graph_traits<Graph>::node_handle &nh) const {
        assert(*nh.id_ < nodes_.size());
        auto &data = getNode(nh);
        return data.prio_;
    }
    /**
     * @brief Sets priority value of node specified by provided node handle
     * @param nh Valid node handle
     * @param prio Priority to be set
     */
    template <typename Graph = ListGraph<NodeData, EdgeData>,
              typename = std::enable_if_t<Graph::heuristicpathTag>>
    void setNodePrio(const typename graph_traits<Graph>::node_handle &nh,
                     typename graph_traits<Graph>::priority_type prio) {
        assert(*nh.id_ < nodes_.size());
        auto &data = getNode(nh);
        data.prio_ = prio;
    }
    /**
     * @brief Adds new node to the undirected graph with O(1) amortized complexity
     * @tparam Args Types of arguments forwarded to node constructor
     * @param args Arguments passed to node constructor
     * @return Node handle to the newly constructed node
     */
    template <typename... Args>
    NodeHandle addNode(Args &&...args) {
        node_id nid = nodes_.size();
        Node &node = nodes_.emplace_back(nid, std::forward<Args>(args)...);
        NodeHandle nh = node.getHandle();
        edges_map_.emplace_back(std::make_unique<ListWrapper<EdgeHandle>>(this));
        return nh;
    }
    /**
     * @brief Adds new edge to the undirected graph with O(1) amortized complexity
     * @tparam Args Types of arguments forwarded to edge constructor
     * @param args Arguments passed to edge constructor
     * @return Edge handle to the newly constructed edge
     */
    template <typename... Args>
    EdgeHandle addEdge(const NodeHandle &nha, const NodeHandle &nhb,
                       Args &&...args) {
        assert(*nha.id_ < nodes_.size() && *nhb.id_ < nodes_.size());
        edge_id eid = edges_.size();
        Edge &edge =
            edges_.emplace_back(nha, nhb, eid, std::forward<Args>(args)...);
        edges_map_[*nha.id_]->push_back(edge.getHandle());
        edges_map_[*nhb.id_]->push_back(edge.getHandle());
        ++valid_edges_;
        return edge.getHandle();
    }
    /**
     * @brief Removes node specified by provided node handle
     * @param nh Valid node handle
     */
    void removeNode(const NodeHandle &nh) {
        assert(*nh.id_ < nodes_.size());
        /* Go through all outgoing edges, mark them invalid and set last_valid
         * NodeHandle to remaining valid node. If some edge is already invalid,
         * it is remove instantly. O(V) */
        for (auto &eh : edges_map_[*nh.id_]->list_) {
            assert(*eh.id_ < edges_.size());
            auto &edge = edges_[*eh.id_];
            if (edge.valid_) {
                edge.valid_ = false;
                edge.last_valid_ = edge.fst_ == nh ? edge.snd_ : edge.fst_;
                --valid_edges_;
            } else {
                removeEdge(eh);
            }
        }
        Node &bnode = nodes_.back();
        if (bnode.getHandle() != nh) {
            std::swap(edges_map_[*nh.id_], edges_map_[*bnode.nid_]);
            nodes_[*nh.id_].swap(bnode);
        }
        edges_map_.pop_back();
        nodes_.pop_back();
    }
    /**
     * @brief Removes edge specified by provided edge handle
     * @param eh Valid edge handle
     */
    void removeEdge(const EdgeHandle &eh) {
        assert(*eh.id_ < edges_.size());
        if (edges_[*eh.id_].valid_) {
            /* Remove EdgeHandles from lists of two adjacent nodes. O(V)*/
            NodeHandle &fst = edges_[*eh.id_].fst_;
            NodeHandle &snd = edges_[*eh.id_].snd_;
            auto &fst_list = edges_map_[*fst.id_]->list_;
            auto &snd_list = edges_map_[*snd.id_]->list_;
            auto it = std::find_if(
                fst_list.begin(), fst_list.end(),
                [&eh](const EdgeHandle &list_eh) { return eh == list_eh; });
            edges_map_[*fst.id_]->erase(it);
            it = std::find_if(
                snd_list.begin(), snd_list.end(),
                [&eh](const EdgeHandle &list_eh) { return eh == list_eh; });
            edges_map_[*snd.id_]->erase(it);
            --valid_edges_;
        } else {
            /* Remove EdgeHandle from list of remaining valid node. O(V)*/
            NodeHandle &valid = edges_[*eh.id_].last_valid_;
            auto &list = edges_map_[*valid.id_]->list_;
            auto it = std::find_if(
                list.begin(), list.end(),
                [&eh](const EdgeHandle &list_eh) { return eh == list_eh; });
            edges_map_[*valid.id_]->erase(it);
        }
        /* Remove corresponding Edge from edges_ vector O(1)*/
        Edge &bedge = edges_.back();
        if (bedge.getHandle() != eh)
            edges_[*eh.id_].swap(edges_.back());
        edges_.pop_back();
    }
};

template <typename NodeData, typename EdgeData>
struct graph_traits<ListGraph<NodeData, EdgeData>> {
  private:
    using Graph = ListGraph<NodeData, EdgeData>;

  public:
    static const bool directedTag = Graph::directedTag;
    static const bool weightedTag = Graph::weightedTag;
    static const bool traversableTag = Graph::traversableTag;
    static const bool pathTag = Graph::pathTag;
    static const bool heuristicpathTag = Graph::heuristicpathTag;

    using node_handle = typename Graph::node_handle;
    using edge_handle = typename Graph::edge_handle;

    using node_data = NodeData;
    using edge_data = EdgeData;

    using node_iterator = typename Graph::node_iterator;
    using const_node_iterator = typename Graph::const_node_iterator;

    using edge_iterator = typename Graph::edge_iterator;
    using const_edge_iterator = typename Graph::const_edge_iterator;

    using adj_range = typename Graph::adj_range;
    using adj_iterator = typename Graph::adj_iterator;
    using const_adj_iterator = typename Graph::const_adj_iterator;

    using weight_type = typename Graph::weight_type;
    using distance_type = typename Graph::distance_type;
    using location_type = typename Graph::location_type;
    using priority_type = typename Graph::priority_type;
};

} // namespace graphlib
#endif

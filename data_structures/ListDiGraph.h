#ifndef LIST_DI_GRAPH_H
#define LIST_DI_GRAPH_H
#include <algorithm>
#include <boost/range.hpp>
#include <cassert>
#include <fstream>
#include <iterator>
#include <list>
#include <memory>
#include <set>
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
 * @brief ListDiGraph is a data structure, which can hold directed graphs. Internally, it represents node adjacency using lists. Basic operations on ListDiGraph have the following complexity:
 * addNode : O(1) amortized
 * addEdge : O(1) amortized
 * removeNode : O(V) worst case
 * removeEdge : O(V) worst case
 *
 * addNode as well as addEdge  have O(1) amortized time complexity, because ListDiGraph internally uses std::vector to store nodes and edges. Creating entry in vector map, that maps node ids to edge lists has constant amortized complexity, too.
 *
 * removeNode is able to work in time linear in |V| using lazy approach. When a node is removed, its outgoing egdes are discarded, and it is marked invalid. A real removal happens when a number of edges, that point to this node reaches zero. Edges, that point to invalid nodes are obviously invalid and should not be considered in subsequent graph traversal. To prevent access to these edges, the ListDiGraph class implements iterators, that automatically remove and skip edges, that have invalid target node. In addition, access through operator[] method offers std::list wrapper which provides range begin and end methods, that return iterators with the same functionality as those described above. Similarly, to iterate over all valid nodes, the class provides equivalent node iterator, except that invalid nodes are not erased, only skipped.
 * removeEdge removes edge of source node with complexity O(V). If a target node is invalid, its inEdges_ reference count is decreased by one. When this reference count drops to zero, it is removed in time O(1). For valid nodes, we just decrease inEdges_ member variable.
 * @tparam NodeData Data to store inside node structure
 * @tparam EdgeData Data to store inside edge structure
 */
template <typename NodeData, typename EdgeData>
class ListDiGraph {
    struct Node {
        template <typename... Args>
        explicit Node(node_id nid, Args &&... args)
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
            swap(inEdges_, node.inEdges_);
            swap(valid_, node.valid_);
            swap(handle_, node.handle_);
            nid_.swap(node.nid_);
            swap(*nid_, *node.nid_);
        }
        const NodeHandle &getHandle() const { return handle_; }

      private:
        NodeHandle handle_;
        std::unique_ptr<node_id> nid_;
        size_t inEdges_{0};
        bool valid_{true};
        NodeData data_;

        template <typename, typename>
        friend class ListDiGraph;
        template <typename>
        friend class ListWrapper;
    };
    struct Edge {
        template <typename... Args>
        Edge(const NodeHandle &src_node, const NodeHandle &tg_node, edge_id eid,
             Args &&... args)
            : src_node_(src_node), tg_node_(tg_node),
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
            swap(src_node_, edge.src_node_);
            swap(tg_node_, edge.tg_node_);
            swap(handle_, edge.handle_);
        }
        const EdgeHandle &getHandle() const { return handle_; }

      private:
        NodeHandle src_node_;
        NodeHandle tg_node_;
        EdgeHandle handle_;
        std::unique_ptr<edge_id> eid_;
        EdgeData data_;

        template <typename, typename>
        friend class ListDiGraph;
    };

    using erange_iterator = typename std::vector<Edge>::iterator;
    using cerange_iterator = typename std::vector<Edge>::const_iterator;
    using nrange_iterator = typename std::vector<Node>::iterator;
    using cnrange_iterator = typename std::vector<Node>::const_iterator;

    using EItTraits = EdgeTraits<erange_iterator>;
    template <typename Iterator>
    using EItBase = ForwardGraphIterator<ListDiGraph, Iterator, EItTraits>;

    class EdgeIterator : public EItBase<EdgeIterator> {
        using base_iterator = EItBase<EdgeIterator>;
        using base_iterator::graph_;
        using base_iterator::cit_;
        using base_iterator::eit_;

      public:
        using value_type = typename EItTraits::value_type;
        using reference = typename EItTraits::reference;
        using pointer = typename EItTraits::pointer;
        using difference_type = typename EItTraits::difference_type;
        using iterator_category = typename EItTraits::iterator_category;

        EdgeIterator() = default;
        EdgeIterator(ListDiGraph *graph, erange_iterator bit, erange_iterator eit)
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
            while (cit_ != eit_ && !graph_->hasNode(cit_->tg_node_)) {
                Edge &edge = *cit_;
                ++cit_;
                graph_->removeEdge(edge.getHandle());
            }
        }
        reference dereference() { return cit_->getHandle(); }
        template <typename, typename, typename>
        friend class ForwardGraphIterator;
        friend class ConstEdgeIterator;
    };

    using CEItTraits = EdgeTraits<cerange_iterator>;
    template <typename Iterator>
    using CEItBase =
        ForwardGraphIterator<const ListDiGraph, Iterator, CEItTraits>;

    class ConstEdgeIterator : public CEItBase<ConstEdgeIterator> {
        using base_iterator = CEItBase<ConstEdgeIterator>;
        using base_iterator::graph_;
        using base_iterator::cit_;
        using base_iterator::eit_;

      public:
        using value_type = typename CEItTraits::value_type;
        using reference = typename CEItTraits::reference;
        using pointer = typename CEItTraits::pointer;
        using difference_type = typename CEItTraits::difference_type;
        using iterator_category = typename CEItTraits::iterator_category;

        ConstEdgeIterator() = default;
        ConstEdgeIterator(const ListDiGraph *graph, cerange_iterator bit,
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
            while (cit_ != eit_ && !graph_->hasNode(cit_->tg_node_))
                ++cit_;
        }
        reference dereference() const { return cit_->getHandle(); }
        template <typename, typename, typename>
        friend class ForwardGraphIterator;
        friend class EdgeIterator;
    };

    using NodeItTraits = NodeTraits<cnrange_iterator>;
    template <typename Iterator>
    using NodeItBase = ForwardRangeIterator<Iterator, NodeItTraits>;

    class NodeIterator : public NodeItBase<NodeIterator> {
        using base_iterator = NodeItBase<NodeIterator>;
        using base_iterator::cit_;
        using base_iterator::eit_;
        using range_iterator = typename NodeItTraits::range_iterator;

      public:
        using value_type = typename NodeItTraits::value_type;
        using reference = typename NodeItTraits::reference;
        using pointer = typename NodeItTraits::pointer;
        using difference_type = typename NodeItTraits::difference_type;
        using iterator_category = typename NodeItTraits::iterator_category;

        NodeIterator() = default;
        NodeIterator(range_iterator bit, range_iterator eit)
            : base_iterator(bit, eit) {}

      private:
        void find_next() {
            while (cit_ != eit_ && !cit_->valid_)
                ++cit_;
        }
        reference dereference() const { return cit_->getHandle(); }
        template <typename, typename>
        friend class ForwardRangeIterator;
    };

    template <typename T>
    class ListWrapper {
        ListDiGraph *graph_{nullptr};
        std::list<T> list_;
        using list_iterator = typename std::list<T>::iterator;
        using const_list_iterator = typename std::list<T>::const_iterator;

        using WrapItTraits = EdgeTraits<const_list_iterator>;
        template <typename Iterator>
        using WrapItBase =
            ForwardGraphIterator<ListDiGraph, Iterator, WrapItTraits>;

        class WrapIterator : public WrapItBase<WrapIterator> {
            using base_iterator = WrapItBase<WrapIterator>;
            using base_iterator::graph_;
            using base_iterator::cit_;
            using base_iterator::eit_;

          public:
            using value_type = typename WrapItTraits::value_type;
            using reference = typename WrapItTraits::reference;
            using pointer = typename WrapItTraits::pointer;
            using difference_type = typename WrapItTraits::difference_type;
            using iterator_category = typename WrapItTraits::iterator_category;

            WrapIterator() = default;
            WrapIterator(ListDiGraph *graph, const_list_iterator bit,
                         const_list_iterator eit)
                : base_iterator(graph, bit, eit) {}
          private:
            void find_next() {
                while (cit_ != eit_ && !graph_->hasNode(graph_->getTarget(*cit_))) {
                    EdgeHandle eh = *cit_;
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
        using difference_type = typename list_iterator::difference_type;
        using size_type = typename std::list<T>::size_type;

        explicit ListWrapper(ListDiGraph *graph) : graph_(graph) {}
        ListWrapper() = default;
        iterator begin() const {
            return iterator(graph_, list_.begin(), list_.end());
        }
        iterator end() const { return iterator(graph_, list_.end(), list_.end()); }
        size_type size() const {
            size_type valid = 0;
            for (auto eh : list_)
                if (graph_->hasNode(graph_->getTarget(eh)))
                    ++valid;
            return valid;
        }
        bool empty() const { return size() == 0; }

      private:
        void push_back(const T &val) { list_.push_back(val); }
        void push_back(T &&val) { list_.push_back(std::move(val)); }
        void erase(const_list_iterator it) { list_.erase(it); }
        void clear() { list_.clear(); }

        template <typename, typename>
        friend class ListDiGraph;
    };

    /* Holds unique_ptr so that references to ListWrapper are not broken on node
   * removal */
    std::vector<std::unique_ptr<ListWrapper<EdgeHandle>>> edges_map_;
    std::vector<Node> nodes_;
    std::vector<Edge> edges_;
    std::size_t valid_nodes_{0};
    std::size_t valid_edges_{0};

  public:
    /** directedTag constant signifies, that ListDiGraph is directed graph */
    static const bool directedTag = true;
    /** weightedTag constant signifies, that ListDiGraph is weighted depending on EdgeData template argument */
    static const bool weightedTag = edge_traits<EdgeData>::weighted;
    /** traversableTag constant signifies, that ListDiGraph has color and predecessor member variables in node data. It means, the graph can be used with basic algorithms, that do traversing on graph. */
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
    /** Alias for std::list wrapper, that helps to skip edges pointing to invalidated nodes */
    using adj_range = ListWrapper<EdgeHandle>;
    /** Alias for std::list wrapper's forward iterator, that implicitely skips edges pointing to invalidated nodes, when incremented */
    using adj_iterator = typename ListWrapper<EdgeHandle>::iterator;
    /** Alias for std::list wrapper's constant forward iterator, that implicitely skips edges pointing to invalidated nodes, when incremented */
    using const_adj_iterator = typename ListWrapper<EdgeHandle>::const_iterator;
    /** Alias ListDiGraph's EdgeData template argument */
    using edge_data = EdgeData;
    /** Alias for ListDiGraph's NodeData template argument */
    using node_data = NodeData;
    /** Alias for iterator used to iterate over nodes */
    using node_iterator = NodeIterator;
    /** Alias for constant iterator used to iterate over nodes */
    using const_node_iterator = NodeIterator;
    /** Alias for iterator used to iterate over edges */
    using edge_iterator = EdgeIterator;
    /** Alias for constant iterator used to iterate over edges */
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
     * @brief Default constructor. Constructs empty directed graph.
     */
    ListDiGraph() = default;
    /**
     * @brief Constructs directed graph with number of nodes
     * @param nonodes Number of nodes to create
     */
    explicit ListDiGraph(size_t nonodes) : valid_nodes_(nonodes) {
        while (nonodes--)
            addNode();
    }
    /**
     * @brief Deleted copy constructor
     */
    ListDiGraph(const ListDiGraph &) = delete;
    /**
     * @brief Deleted copy assignment
     */
    ListDiGraph &operator=(const ListDiGraph &) = delete;

    /**
     * @brief Returns true if there is an edge between two nodes
     * @param nha Node handle to the source node
     * @param nhb Node handle to the target node
     * @return bool value, that signifies presence of edge
     */
    bool hasEdge(const NodeHandle &nha, const NodeHandle &nhb) const {
        if (*nha.id_ >= nodes_.size() || *nhb.id_ >= nodes_.size() ||
            !nodes_[*nha.id_].valid_ || !nodes_[*nhb.id_].valid_)
            return false;
        auto &list = *edges_map_[*nha.id_];
        auto it = std::find_if(list.begin(), list.end(),
                               [this, &nhb](const EdgeHandle &eh) {
                                   return nhb == edges_[*eh.id_].tg_node_;
                               });
        return it != list.end();
    }
    /**
     * @brief Returns true if there is an edge with provided edge handle
     * @param eh Edge handle to verify
     * @return bool value, that signifies presence of edge
     */
    bool hasEdge(const EdgeHandle &eh) const {
        return *eh.id_ < edges_.size() ? true : false;
    }
    /**
     * @brief Returns true if there is a node with provided node handle
     * @param nh Node handle to verify
     * @return bool value, that signifies existence of such node
     */
    bool hasNode(const NodeHandle &nh) const {
        return (*nh.id_ < nodes_.size() && nodes_[*nh.id_].valid_) ? true : false;
    }
    /**
     * @brief Returns count of edges, that point to provided node handle
     * @param nh Valid node handle
     * @return Count of edges directed to node with provided node handle
     */
    size_t inNodeDegree(const NodeHandle &nh) const {
        assert(*nh.id_ < nodes_.size());
        return nodes_[*nh.id_].inEdges_;
    }
    /* Returns number of outgoing edges, that point to valid nodes.
   * outNodeDegree method has asymptotic time complexity O(V) */
    /**
     * @brief Returns count of edges, that have source in provided node handle
     * @param nh Valid node handle
     * @return Count of edges, that have source in provided node handle
     */
    size_t outNodeDegree(const NodeHandle &nh) const {
        assert(*nh.id_ < nodes_.size());
        if (!nodes_[*nh.id_].valid_)
            return 0;
        return (*edges_map_[*nh.id_]).size();
    }
    /**
     * @brief Returns edge data to provided edge handle
     * @param eh Valid edge handle
     * @return Reference to EdgeData stored inside edge
     */
    EdgeData &getEdge(const EdgeHandle &eh) {
        assert(*eh.id_ < edges_.size());
        return edges_[*eh.id_].data_;
    }
    /**
     * @brief Returns edge data to provided edge handle
     * @param eh Valid edge handle
     * @return Constant reference to EdgeData stored inside edge
     */
    const EdgeData &getEdge(const EdgeHandle &eh) const {
        assert(*eh.id_ < edges_.size());
        return edges_[*eh.id_].data_;
    }
    /**
     * @brief Returns node data to provided node handle
     * @param nh Valid node handle
     * @return Reference to NodeData stored inside node
     */
    NodeData &getNode(const NodeHandle &nh) {
        assert(*nh.id_ < nodes_.size() && nodes_[*nh.id_].valid_);
        return nodes_[*nh.id_].data_;
    }
    /**
     * @brief Returns node data to provided node handle
     * @param nh Valid node handle
     * @return Constant reference to NodeData stored inside node
     */
    const NodeData &getNode(const NodeHandle &nh) const {
        assert(*nh.id_ < nodes_.size() && nodes_[*nh.id_].valid_);
        return nodes_[*nh.id_].data_;
    }
    /**
     * @brief Adds new node to the directed graph with O(1) amortized complexity
     * @tparam Args Types of arguments forwarded to node constructor
     * @param args Arguments passed to node constructor
     * @return Node handle to the newly constructed node
     */
    template <typename... Args>
    NodeHandle addNode(Args &&... args) {
        node_id nid = nodes_.size();
        Node &node = nodes_.emplace_back(nid, std::forward<Args>(args)...);
        NodeHandle nh = node.getHandle();
        edges_map_.emplace_back(std::make_unique<ListWrapper<EdgeHandle>>(this));
        ++valid_nodes_;
        return nh;
    }
    /**
     * @brief Adds new edge to the directed graph with O(1) amortized complexity
     * @tparam Args Types of arguments forwarded to edge constructor
     * @param args Arguments passed to edge constructor
     * @return Edge handle to the newly constructed edge
     */
    template <typename... Args>
    EdgeHandle addEdge(const NodeHandle &nha, const NodeHandle &nhb,
                       Args &&... args) {
        assert(*nha.id_ < nodes_.size() && *nhb.id_ < nodes_.size() &&
               nodes_[*nha.id_].valid_ && nodes_[*nhb.id_].valid_);
        edge_id eid = edges_.size();
        Edge &edge =
            edges_.emplace_back(nha, nhb, eid, std::forward<Args>(args)...);
        (*edges_map_[*nha.id_]).push_back(edge.getHandle());
        ++nodes_[*nhb.id_].inEdges_;
        ++valid_edges_;
        return edge.getHandle();
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
    template <typename Graph = ListDiGraph<NodeData, EdgeData>,
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
    template <typename Graph = ListDiGraph<NodeData, EdgeData>,
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
    template <typename Graph = ListDiGraph<NodeData, EdgeData>,
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
    template <typename Graph = ListDiGraph<NodeData, EdgeData>,
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
    template <typename Graph = ListDiGraph<NodeData, EdgeData>,
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
    template <typename Graph = ListDiGraph<NodeData, EdgeData>,
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
    template <typename Graph = ListDiGraph<NodeData, EdgeData>,
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
    template <typename Graph = ListDiGraph<NodeData, EdgeData>,
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
    template <typename Graph = ListDiGraph<NodeData, EdgeData>,
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
    template <typename Graph = ListDiGraph<NodeData, EdgeData>,
              typename = std::enable_if_t<Graph::heuristicpathTag>>
    void setNodePrio(const typename graph_traits<Graph>::node_handle &nh,
                     typename graph_traits<Graph>::priority_type prio) {
        assert(*nh.id_ < nodes_.size());
        auto &data = getNode(nh);
        data.prio_ = prio;
    }
    /**
     * @brief Returns node handle to node, which is source of edge specified by provided edge handle
     * @param eh Valid edge handle
     * @return Node handle of source node
     */
    NodeHandle getSource(const EdgeHandle &eh) const {
        assert(*eh.id_ < edges_.size());
        return edges_[*eh.id_].src_node_;
    }
    /**
     * @brief Returns node data of node, which is source of edge specified by provided edge handle
     * @param eh Valid edge handle
     * @return Reference to node data of source node
     */
    NodeData &getSourceNode(const EdgeHandle &eh) {
        assert(*eh.id_ < edges_.size());
        NodeHandle nh = getSource(eh);
        return nodes_[*nh.id_].data_;
    }
    /**
     * @brief Returns node data of node, which is source of edge specified by provided edge handle
     * @param eh Valid edge handle
     * @return Constant reference to node data of source node
     */
    const NodeData &getSourceNode(const EdgeHandle &eh) const {
        assert(*eh.id_ < edges_.size());
        NodeHandle nh = getSource(eh);
        return nodes_[*nh.id_].data_;
    }
    /**
     * @brief Returns node handle to node, which is target of edge specified by provided edge handle
     * @param eh Valid edge handle
     * @return Node handle of target node
     */
    NodeHandle getTarget(const EdgeHandle &eh) const {
        assert(*eh.id_ < edges_.size());
        return edges_[*eh.id_].tg_node_;
    }
    /**
     * @brief Returns node data of node, which is target of edge specified by provided edge handle
     * @param eh Valid edge handle
     * @return Reference to node data of target node
     */
    NodeData &getTargetNode(const EdgeHandle &eh) {
        assert(*eh.id_ < edges_.size());
        NodeHandle nh = getTarget(eh);
        return nodes_[*nh.id_].data_;
    }
    /**
     * @brief Returns node data of node, which is target of edge specified by provided edge handle
     * @param eh Valid edge handle
     * @return Constant reference to node data of target node
     */
    const NodeData &getTargetNode(const EdgeHandle &eh) const {
        assert(*eh.id_ < edges_.size());
        NodeHandle nh = getTarget(eh);
        return nodes_[*nh.id_].data_;
    }
    /**
     * @brief Returns a list wrapper of edge handles corresponding to edges, that have source in node specified by provided node handle
     * @param nh Valid node handle
     * @return Reference to list wrapper of edge handles
     */
    adj_range &operator[](const NodeHandle &nh) {
        assert(*nh.id_ < nodes_.size() && nodes_[*nh.id_].valid_);
        return *edges_map_[*nh.id_];
    }
    /**
     * @brief Returns a list wrapper of edge handles corresponding to edges, that have source in node specified by provided node handle
     * @param nh Valid node handle
     * @return Constant reference to list wrapper of edge handles
     */
    const adj_range &operator[](const NodeHandle &nh) const {
        assert(*nh.id_ < nodes_.size() && nodes_[*nh.id_].valid_);
        return *edges_map_[*nh.id_];
    }
    /**
     * @brief Removes edge specified by provided edge handle
     * @param eh Valid edge handle
     */
    void removeEdge(const EdgeHandle &eh) {
        assert(*eh.id_ < edges_.size());
        /* Decrease reference count (inEdges_) of target node and if
     * it is zero and at the same time valid_ bool flag is false,
     * then remove target node from nodes_ vector. O(1) */
        NodeHandle tg_nh = getTarget(eh);
        Node &tg_node = nodes_[*tg_nh.id_];
        --tg_node.inEdges_;
        if (tg_node.valid_)
            --valid_edges_;
        if (!tg_node.valid_ && !tg_node.inEdges_)
            removeNode(tg_nh);

        /* Remove EdgeHandle eh from source node's outgoing edge list
     * with the complexity O(V) */
        NodeHandle src_nh = getSource(eh);
        auto &list = (*edges_map_[*src_nh.id_]).list_;
        auto eh_it = std::find_if(
            list.begin(), list.end(),
            [&eh](const EdgeHandle &list_eh) { return eh == list_eh; });
        (*edges_map_[*src_nh.id_]).erase(eh_it);

        /* Remove Edge corresponding to EdgeHandler eh from edges_
     * vector. O(1) */
        if (edges_.size() > 1)
            edges_[*eh.id_].swap(edges_.back());
        edges_.pop_back();
    }

    /* removeNode - removes Node from graph in a lazy manner. Therefore, it
   * is able to achieve final complexity of O(V) instead of O(V + E), iow.
   * quadratic in number of nodes. A node is completely removed when it has
   * no entering edges */
    /**
     * @brief Removes node specified by provided node handle
     * @param nh Valid node handle
     */
    void removeNode(const NodeHandle &nh) {
        assert(*nh.id_ < nodes_.size());
        Node &node = nodes_[*nh.id_];
        /* If node is not valid anymore and no edge is ingoing, then
     * there is no reference to it, so we can remove it from vector.
     * Complexity is O(1) */
        if (!node.valid_ && !node.inEdges_) {
            /* Swap node to be removed with back node, same for their edges */
            if (nodes_.size() > 1) {
                Node &bnode = nodes_.back();
                edges_map_[*nh.id_].swap(edges_map_[*bnode.nid_]);
                nodes_[*nh.id_].swap(bnode);
            }
            edges_map_.pop_back();
            nodes_.pop_back();
        } else if (node.valid_) {
            /* Go through edges and decrease inEdges_(reference count)
       * of target nodes. Then remove edge from vector. O(V) */
            for (auto &eh : (*edges_map_[*nh.id_]).list_) {
                NodeHandle tg_nh = getTarget(eh);
                --nodes_[*tg_nh.id_].inEdges_;

                if (nodes_[*tg_nh.id_].valid_)
                    --valid_edges_;
                /* If a target node is invalid and has inEdges_ equal to zero,
         * remove it. Complexity O(1) */
                if (!nodes_[*tg_nh.id_].inEdges_ && !nodes_[*tg_nh.id_].valid_)
                    removeNode(tg_nh);

                /* Remove edge from vector of edges. O(1) */
                if (edges_.size() > 1)
                    edges_[*eh.id_].swap(edges_.back());
                edges_.pop_back();
            }
            /* Erase list of outgoing edges. Complexity O(V) */
            (*edges_map_[*nh.id_]).clear();
            nodes_[*nh.id_].valid_ = false;
            valid_edges_ -= nodes_[*nh.id_].inEdges_;
            /* If the node was valid and had no entering edges,
       * then remove it rightaway. O(1) */
            if (!nodes_[*nh.id_].inEdges_)
                removeNode(nh);
            --valid_nodes_;
        }
    }
    /**
     * @brief Returns count of nodes in the graph
     * @return Count of nodes
     */
    node_id nodeCount() const { return valid_nodes_; }
    /**
     * @brief Returns count of edges in the graph
     * @return Count of edges
     */
    edge_id edgeCount() const { return valid_edges_; }
    /**
     * @brief Returns iterator to the first node
     * @return Node iterator
     */
    node_iterator beginNode() {
        return node_iterator(nodes_.begin(), nodes_.end());
    }
    /**
     * @brief Returns constant iterator to the first node
     * @return Constant node iterator
     */
    const_node_iterator beginNode() const {
        return const_node_iterator(nodes_.begin(), nodes_.end());
    }
    /**
     * @brief Returns constant iterator to the first node
     * @return Constant node iterator
     */
    const_node_iterator cbeginNode() const {
        return const_node_iterator(nodes_.cbegin(), nodes_.cend());
    }
    /**
     * @brief Returns iterator to the node following the last node of the graph
     * @return Node iterator
     */
    node_iterator endNode() { return node_iterator(nodes_.end(), nodes_.end()); }
    /**
     * @brief Returns constant iterator to the node following the last node of the graph
     * @return Constant node iterator
     */
    const_node_iterator endNode() const {
        return const_node_iterator(nodes_.end(), nodes_.end());
    }
    /**
     * @brief Returns constant iterator to the node following the last node of the graph
     * @return Constant node iterator
     */
    const_node_iterator cendNode() const {
        return const_node_iterator(nodes_.cend(), nodes_.cend());
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
};

template <typename NodeData, typename EdgeData>
struct graph_traits<ListDiGraph<NodeData, EdgeData>> {
  private:
    using Graph = ListDiGraph<NodeData, EdgeData>;

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

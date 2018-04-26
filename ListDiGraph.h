#ifndef LIST_GRAPH_H
#define LIST_GRAPH_H
#include <memory>
#include <vector>
#include <list>
#include <unordered_map>
#include <cassert>
#include <algorithm>
#include <iterator>

#include "NodeData.h"
#include "EdgeData.h"
#include "NodeTraits.h"
#include "EdgeTraits.h"
#include "GraphTraits.h"

namespace graphlib {

/* Complexity:
 * addNode : O(1) amortized
 * addEdge : O(1) amortized
 * removeNode : O(V) worst case
 * removeEdge : O(V) worst case
 *
 * addNode as well as addEdge  have O(1) amortized time complexity, because ListDiGraph
 * internally uses std::vector to store nodes and edges.
 *
 * removeNode is able to work in time linear in |V| using lazy approach. When
 * a node is removed, its outgoing egdes are discarded, and it is marked invalid. A real
 * removal happens when number of edges, that point to this node reaches zero.
 * Edges, that point to invalid nodes are obviously invalid and should not be considered
 * in subsequent graph traversal. To prevent access to these edges, the ListDiGraph
 * class defines iterators, that automatically remove and skip edges, that have invalid
 * target node. In addition, access through operator[] method offers std::list wrapper
 * which provides range begin and end methods, that return iterators with same functionality
 * as those described above. Similarly, to iterate over all valid nodes, the class provides
 * equivalent node iterator, except that invalid nodes are not erased, only skipped.
 *
 * removeEdge removes edge of source node with complexity O(V). If a target node is invalid,
 * its inEdges_ reference count is decreased by one. When this reference count drops to
 * zero, it is removed in time O(1). For valid nodes, we just decrease inEdges_ member variable.
*/

template<typename NodeData,
         typename EdgeData>
class ListDiGraph {
	using node_id = size_t;
	using edge_id = size_t;

	template<typename IdType>
	struct Handle {
		Handle() : id_(nullptr) {}
		bool operator==(const Handle& handle) const {
			return id_ == handle.id_;
		}
		bool operator!=(const Handle& handle) const {
			return id_ == handle.id_;
		}
		IdType getValue() const {
			return *id_;
		}
	private:
		friend struct ListDiGraph;
		Handle(IdType *id) : id_(id) {}
		IdType *id_;
	};

	using NodeHandle = Handle<node_id>;
	using EdgeHandle = Handle<edge_id>;

	struct Node {
		template<typename... Args>
		Node(node_id nid, Args&&... args) : nid_(std::make_unique<node_id>(nid)),
		    inEdges_(0), valid_(true), data_(std::forward<Args>(args)...) {}
		bool operator==(const Node& node) {
			return nid_ == node.nid_ && data_ == node.data_;
		}
		bool operator!=(const Node& node) {
			return !(*this == node);
		}
		void swap(Node& node) {
			if (this == &node)
				return;
			using std::swap;
			swap(data_, node.data_);
			swap(inEdges_, node.inEdges_);
			swap(valid_, node.valid_);
			nid_.swap(node.nid_);
			swap(*nid_, *node.nid_);
		}
		NodeHandle getHandle() const {
			return NodeHandle(nid_.get());
		}
	private:
		std::unique_ptr<node_id> nid_;
		size_t inEdges_;
		bool valid_;
		NodeData data_;

		friend struct ListDiGraph;
		template<typename, typename>
		friend class LazyDataIterator;
	};
	struct Edge {
		template<typename... Args>
		Edge(const NodeHandle& src_node,
		     const NodeHandle& tg_node,
		     edge_id eid,
		     Args&&... args) : src_node_(src_node), tg_node_(tg_node),
		    eid_(std::make_unique<edge_id>(eid)), data_(std::forward<Args>(args)...) {}
		bool operator==(const Edge& edge) const {
			return eid_ == edge.eid_ && data_ == edge.data_;
		}
		bool operator!=(const Edge& edge) const {
			return !(*this == edge);
		}
		void swap(Edge& edge) {
			if (this == &edge)
				return;
			using std::swap;
			swap(data_, edge.data_);
			eid_.swap(edge.eid_);
			swap(*eid_, *edge.eid_);
			swap(src_node_, edge.src_node_);
			swap(tg_node_, edge.tg_node_);
		}
		EdgeHandle getHandle() const {
			return EdgeHandle(eid_.get());
		}
	private:
		NodeHandle src_node_;
		NodeHandle tg_node_;
		std::unique_ptr<edge_id> eid_;
		EdgeData data_;

		friend struct ListDiGraph;
		template<typename, typename>
		friend class DataIterator;
	};

	template<typename T>
	class ListWrapper {
		ListDiGraph *graph_;
		std::list<T> list_;
		using list_iterator = typename std::list<T>::iterator;
		using const_list_iterator = typename std::list<T>::const_iterator;

		class WrapIterator {
			using traits = std::iterator_traits<const_list_iterator>;
			ListDiGraph *graph_;
			const_list_iterator cit_;
			const_list_iterator eit_;

			void find_next() {
				while (cit_ != eit_ && !graph_->hasNode(graph_->getTarget(*cit_))) {
					auto tmp(cit_);
					++cit_;
					graph_->removeEdge(*tmp);
				}
			}
			void increment() {
				if (cit_ != eit_)
					++cit_;
				find_next();
			}

		public:
			using value_type = typename traits::value_type;
			using reference = typename traits::reference;
			using pointer = typename traits::pointer;
			using difference_type = typename traits::difference_type;
			using iterator_category = std::forward_iterator_tag;

			WrapIterator(ListDiGraph *graph,
			             const_list_iterator bit,
			             const_list_iterator eit) : graph_(graph), cit_(bit), eit_(eit) {
				find_next();
			}
			WrapIterator &operator++() {
				increment();
				return *this;
			}
			WrapIterator operator++(int) {
				auto copy(*this);
				increment();
				return copy;
			}
			reference operator*() {
				return *cit_;
			}
			pointer operator->() {
				return &(*cit_);
			}
			bool operator==(const WrapIterator& it) const {
				return graph_ == it.graph_ && cit_ == it.cit_;
			}
			bool operator!=(const WrapIterator& it) const {
				return !(*this == it);
			}
		};
	public:
		using value_type = typename std::list<T>::value_type;
		using reference = typename std::list<T>::reference;
		using const_reference = typename std::list<T>::const_reference;
		using iterator = WrapIterator;
		using const_iterator = WrapIterator;
		using difference_type = typename list_iterator::difference_type;
		using size_type = typename std::list<T>::size_type;

		ListWrapper(ListDiGraph *graph) : graph_(graph) {}
		ListWrapper() : graph_(nullptr) {}
		iterator begin() const {return iterator(graph_, list_.begin(), list_.end());}
		iterator end() const {return iterator(graph_, list_.end(), list_.end());}
		size_type size() const {
			auto it = begin();
			while(it != end())
				++it;
			return list_.size();
		}
		bool empty() const {return list_.empty();}

	private:
		list_iterator priv_begin() {return list_.begin();}
		const_list_iterator priv_begin() const {return list_.begin();}
		const_list_iterator priv_cbegin() const {return list_.cbegin();}

		list_iterator priv_end() {return list_.end();}
		const_list_iterator priv_end() const {return list_.end();}
		const_list_iterator priv_cend() const {return list_.cend();}
		void push_back(const T& val) {list_.push_back(val);}
		void push_back(T&& val) {list_.push_back(std::move(val));}
		void erase(const_list_iterator it) {list_.erase(it);}

		template<typename, typename>
		friend class ListDiGraph;
	};

	using edge_range_iterator = typename std::vector<Edge>::iterator;
	using cedge_range_iterator = typename std::vector<Edge>::const_iterator;

	class ConstEdgeIterator;
	class EdgeIterator {
		ListDiGraph* graph_;
		edge_range_iterator cit_;
		edge_range_iterator eit_;

		void find_next() {
			while (cit_ != eit_ && !graph_->hasNode(cit_->tg_node_)) {
				auto tmp(cit_);
				++cit_;
				graph_->removeEdge(tmp->getHandle());
			}
		}
		void increment() {
			if (cit_ != eit_)
				++cit_;
			find_next();
		}

	public:
		using value_type = EdgeHandle;
		using reference = EdgeHandle;
		using pointer = EdgeHandle;
		using difference_type = std::ptrdiff_t;
		using iterator_category = std::forward_iterator_tag;

		EdgeIterator() : graph_(nullptr), cit_(nullptr), eit_(nullptr) {}
		EdgeIterator(ListDiGraph *graph,
		             edge_range_iterator cit,
		             edge_range_iterator eit) : graph_(graph), cit_(cit), eit_(eit) {
			find_next();
		}
		EdgeIterator &operator++() {
			increment();
			return *this;
		}
		EdgeIterator operator++(int) {
			auto copy(*this);
			increment();
			return copy;
		}
		reference operator*() {
			return cit_->getHandle();
		}
		/*pointer operator->() {
			return &cit_->data_;
		}*/
		bool operator==(const EdgeIterator& it) const {
			return graph_ == it.graph_ && cit_ == it.cit_;
		}
		bool operator!=(const EdgeIterator& it) const {
			return !(*this == it);
		}
		bool operator==(const ConstEdgeIterator& it) const {
			return graph_ == it.graph_ && cit_ == it.cit_;
		}
		bool operator!=(const ConstEdgeIterator& it) const {
			return !(*this == it);
		}
	};
	class ConstEdgeIterator {
		const ListDiGraph *graph_;
		cedge_range_iterator cit_;
		cedge_range_iterator eit_;

		void find_next() {
			while (cit_ != eit_ && !graph_->hasNode(cit_->tg_node_))
				++cit_;
		}
		void increment() {
			if (cit_ != eit_)
				++cit_;
			find_next();
		}
	public:
		using value_type = const EdgeHandle;
		using reference = const EdgeHandle;
		using pointer = const EdgeHandle;
		using difference_type = std::ptrdiff_t;
		using iterator_category = std::forward_iterator_tag;

		ConstEdgeIterator() : graph_(nullptr), cit_(), eit_() {}
		ConstEdgeIterator(const ListDiGraph *graph,
		                  cedge_range_iterator bit,
		                  cedge_range_iterator eit) : graph_(graph), cit_(bit), eit_(eit) {
			find_next();
		}
		ConstEdgeIterator(const EdgeIterator &it) : graph_(it.graph_), cit_(it.cit_), eit_(it.eit_) {}
		ConstEdgeIterator& operator++() {
			increment();
			return *this;
		}
		ConstEdgeIterator operator++(int) {
			auto copy(*this);
			increment();
			return copy;
		}
		reference operator*() {
			return cit_->getHandle();
		}
		/*pointer operator->() {
			return &cit_->data_;
		}*/
		bool operator==(const ConstEdgeIterator& it) const {
			return graph_ == it.graph_ && cit_ == it.cit_;
		}
		bool operator!=(const ConstEdgeIterator& it) const {
			return !(*this == it);
		}
		bool operator==(const EdgeIterator& it) const {
			return graph_ == it.graph_ && cit_ == it.cit_;
		}
		bool operator!=(const EdgeIterator& it) const {
			return !(*this == it);
		}
	};

	template<typename Iterator>
	class NodeIterator {
		Iterator cit_;
		Iterator eit_;

		void find_next() {
			while (cit_ != eit_ && !cit_->valid_)
				++cit_;
		}
		void increment() {
			if (cit_ != eit_)
				++cit_;
			find_next();
		}
		template<typename>
		friend class NodeIterator;

	public:
		using value_type = NodeHandle;
		using reference = NodeHandle;
		using pointer = NodeHandle;
		using difference_type = std::ptrdiff_t;
		using iterator_category = std::forward_iterator_tag;

		NodeIterator() = default;
		template<typename T>
		NodeIterator(T it) : cit_(it.cit_), eit_(it.eit_) {}
		NodeIterator(Iterator bit, Iterator eit) : cit_(bit), eit_(eit) {
			find_next();
		}
		NodeIterator& operator++() {
			increment();
			return *this;
		}
		NodeIterator operator++(int) {
			auto copy(*this);
			increment();
			return copy;
		}
		reference operator*() {
			return cit_->getHandle();
		}
		/*pointer operator->() {
			return &cit_->data_;
		}*/
		template<typename I>
		bool operator==(const NodeIterator<I> it) const {
			return cit_ == it.cit_ && eit_ == it.eit_;
		}
		template<typename I>
		bool operator!=(const NodeIterator<I> it) const {
			return !(*this == it);
		}
	};

	std::vector<Node> nodes_;
	std::vector<Edge> edges_;
	std::unordered_map<node_id, ListWrapper<EdgeHandle>> edges_map_;
	std::size_t valid_nodes_;
	std::size_t valid_edges_;

public:
	// Traits
	using node_handle = NodeHandle;
	using edge_handle = EdgeHandle;
	using edge_range = ListWrapper<EdgeHandle>;
	using edge_data = EdgeData;
	using node_data = NodeData;
	using node_iterator = NodeIterator<typename std::vector<Node>::iterator>;
	using const_node_iterator = NodeIterator<typename std::vector<Node>::const_iterator>;
	using edge_iterator = EdgeIterator;
	using const_edge_iterator = ConstEdgeIterator;

	static const bool directedTag = true;
	static const bool weightedTag = edge_traits<EdgeData>::weighted;
	static const bool traversableTag = node_traits<NodeData>::color && node_traits<NodeData>::predecessor;
	static const bool pathTag = traversableTag && node_traits<NodeData>::distance;
	static const bool heuristicpathTag = pathTag && node_traits<NodeData>::location;

	ListDiGraph() : valid_nodes_(0), valid_edges_(0) {}
	ListDiGraph(size_t nonodes) : valid_nodes_(nonodes), valid_edges_(0) {
		while (nonodes--)
			addNode();
	}
	ListDiGraph(const ListDiGraph&) = delete;
	ListDiGraph& operator=(const ListDiGraph&) = delete;

	// Methods
	bool hasEdge(const NodeHandle& nha, const NodeHandle& nhb) const {
		if (*nha.id_ >= nodes_.size() || *nhb.id_ >= nodes_.size() ||
		        !nodes_[*nha.id_].valid_ || !nodes_[*nhb.id_].valid_)
			return false;
		auto& list = edges_map_.find(*nha.id_)->second;
		auto it = std::find_if(list.begin(), list.end(), [this, &nhb](const EdgeHandle& eh) {
			return nhb == edges_[*eh.id_].tg_node_;
		});
		return it != list.end();
	}
	bool hasEdge(const EdgeHandle& eh) const {
		if (*eh.id_ >= edges_.size())
			return false;
		return true;
	}
	bool hasNode(const NodeHandle& nh) const {
		if (*nh.id_ >= nodes_.size() || !nodes_[*nh.id_].valid_)
			return false;
		return true;
	}
	size_t inNodeDegree(const NodeHandle &nh) const {
		assert(*nh.id_ < nodes_.size());
		return nodes_[*nh.id_].inEdges_;
	}
	size_t outNodeDegree(const NodeHandle& nh) const {
		assert(*nh.id_ < nodes_.size());
		if (!nodes_[*nh.id_].valid_)
			return 0;
		return edges_map_.find(*nh.id_)->second.size();
	}
	EdgeData& getEdge(const EdgeHandle& eh) {
		assert(*eh.id_ < edges_.size());
		return edges_[*eh.id_].data_;
	}
	const EdgeData& getEdge(const EdgeHandle& eh) const {
		assert(*eh.id_ < edges_.size());
		return edges_[*eh.id_].data_;
	}
	NodeData& getNode(const NodeHandle& nh) {
		assert(*nh.id_ < nodes_.size() && nodes_[*nh.id_].valid_);
		return nodes_[*nh.id_].data_;
	}
	const NodeData& getNode(const NodeHandle& nh) const {
		assert(*nh.id_ < nodes_.size() && nodes_[*nh.id_].valid_);
		return nodes_[*nh.id_].data_;
	}
	/* addNode - adds new node to nodes_ vector, initializes list of
	 * outgoing edges in edges_map_. Complexity O(1) */
	template<typename... Args>
	NodeHandle addNode(Args&&... args) {
		node_id nid = nodes_.size();
		Node &node = nodes_.emplace_back(nid, std::forward<Args>(args)...);
		NodeHandle nh = node.getHandle();
		edges_map_.emplace(*nh.id_, this);
		++valid_nodes_;
		return nh;
	}
	/* addEdge - adds new edge to edges_ vector, increases inEdges_
	 * counter of target node and returns handle. Complexity O(1) */
	template<typename... Args>
	EdgeHandle addEdge(const NodeHandle& nha,
	                   const NodeHandle& nhb,
	                   Args&&... args) {
		assert(*nha.id_ < nodes_.size() && *nhb.id_ < nodes_.size() &&
		       nodes_[*nha.id_].valid_ && nodes_[*nhb.id_].valid_);
		edge_id eid = edges_.size();
		Edge& edge = edges_.emplace_back(nha, nhb, eid, std::forward<Args>(args)...);
		edges_map_[*nha.id_].push_back(edge.getHandle());
		++nodes_[*nhb.id_].inEdges_;
		++valid_edges_;
		return edge.getHandle();
	}
	template<typename... Args, typename EData = EdgeData>
	std::enable_if_t<EData::weighted, EdgeHandle>
	addEdge(const NodeHandle& nha,
	        const NodeHandle& nhb,
	        typename EData::weight_type weight,
	        Args&&... args) {
		assert(*nha.id_ < nodes_.size() && *nhb.id_ < nodes_.size() &&
		       nodes_[*nha.id_].valid_ && nodes_[*nhb.id_].valid_);
		edge_id eid = edges_.size();
		Edge& edge = edges_.emplace_back(nha, nhb, eid, weight, std::forward<Args>(args)...);
		edges_map_[*nha.id_].push_back(edge.getHandle());
		++nodes_[*nhb.id_].inEdges_;
		++valid_edges_;
		return edge.getHandle();
	}
	template<typename EData = EdgeData>
	std::enable_if_t<EData::weighted, void>
	setWeight(const EdgeHandle& eh,
	          typename EData::weight_type weight) {
		assert(*eh.id_ < edges_.size());
		edges_[*eh.id_].weight_ = weight;
	}
	template<typename EData = EdgeData>
	std::enable_if_t<EData::weighted, typename EData::weight_type>
	getWeight(const EdgeHandle& eh) const {
		assert(*eh.id_ < edges_.size());
		return edges_[*eh.id_].data_.weight_;
	}
	NodeHandle getSource(const EdgeHandle& eh) const {
		assert(*eh.id_ < edges_.size());
		return edges_[*eh.id_].src_node_;
	}
	NodeData &getSourceNode(const EdgeHandle& eh) {
		assert(*eh.id_ < edges_.size());
		NodeHandle nh = getSource(eh);
		return nodes_[*nh.id_].data_;
	}
	const NodeData& getSourceNode(const EdgeHandle& eh) const {
		assert(*eh.id_ < edges_.size());
		NodeHandle nh = getSource(eh);
		return nodes_[*nh.id_].data_;
	}
	NodeHandle getTarget(const EdgeHandle& eh) const {
		assert(*eh.id_ < edges_.size());
		return edges_[*eh.id_].tg_node_;
	}
	NodeData &getTargetNode(const EdgeHandle& eh) {
		assert(*eh.id_ < edges_.size());
		NodeHandle nh = getTarget(eh);
		return nodes_[*nh.id_].data_;
	}
	const NodeData &getTargetNode(const EdgeHandle& eh) const {
		assert(*eh.id_ < edges_.size());
		NodeHandle nh = getTarget(eh);
		return nodes_[*nh.id_].data_;
	}
	edge_range &operator[](const NodeHandle& nh) {
		assert(*nh.id_ < nodes_.size() && nodes_[*nh.id_].valid_);
		return edges_map_[*nh.id_];
	}
	const edge_range &operator[](const NodeHandle& nh) const {
		assert(*nh.id_ < nodes_.size() && nodes_[*nh.id_].valid_);
		return edges_map_.find(*nh.id_)->second;
	}
	void removeEdge(const EdgeHandle& eh) {
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
		auto eh_it = std::find_if(edges_map_[*src_nh.id_].priv_begin(),
		                       edges_map_[*src_nh.id_].priv_end(),
		                       [&eh](const EdgeHandle& list_eh){return eh == list_eh;});
		edges_map_[*src_nh.id_].erase(eh_it);

		/* Remove Edge corresponding to EdgeHandler eh from edges_
		 * vector. O(1) */
		if (edges_.size() > 1)
			edges_[*eh.id_].swap(edges_.back());
		edges_.pop_back();
	}

	/* removeNode - removes Node from graph in a lazy manner. Therefore, it
	 * is able to achieve final complexity of O(V) instead of O(V + E), iow
	 * quadratic in number of nodes. The removed node is completely removed
	 * when it has no entering edges */
	void removeNode(const NodeHandle& nh) {
		assert(*nh.id_ < nodes_.size());
		Node &node = nodes_[*nh.id_];
		/* If node is not valid anymore and no edge is ingoing, then
		 * there is no reference to it, so we can remove it from vector
		 * Complexity is O(1) in average */
		if (!node.valid_ && !node.inEdges_) {
			/* We need to change back node key to unordered_map, so that
			 * after node index swap, we can access correct list */
			Node &bnode = nodes_.back();
			if (bnode.valid_) {
				auto nh_map = edges_map_.extract(*bnode.nid_);
				nh_map.key() = *nh.id_;
				edges_map_.insert(std::move(nh_map));
			}
			if (nodes_.size() > 1)
				nodes_[*nh.id_].swap(bnode);
			nodes_.pop_back();
		} else if (node.valid_) {
			/* Go through edges and decrease inEdges_(reference count)
			 * of target nodes. Then remove edge from vector. O(V) */
			//for (auto& eh : edges_map_[*nh.id_]) {
			for (auto eh_it = edges_map_[*nh.id_].priv_begin();
			     eh_it != edges_map_[*nh.id_].priv_end(); ++eh_it) {
				NodeHandle tg_nh = getTarget(*eh_it);
				--nodes_[*tg_nh.id_].inEdges_;

				if (nodes_[*tg_nh.id_].valid_)
					--valid_edges_;
				/* If a target node has inEdges_ equal to zero, remove it.
				 * O(1) in average due to extract and subsequence insert */
				if (!nodes_[*tg_nh.id_].inEdges_ && !nodes_[*tg_nh.id_].valid_)
					removeNode(tg_nh);

				/* Remove edge from vector of edges. O(1) */
				if (edges_.size() > 1)
					edges_[*(*eh_it).id_].swap(edges_.back());
				edges_.pop_back();
			}
			/* Erase list of outgoing edges. At most O(V), otherwise
			 * constant O(1) on average */
			edges_map_.erase(*nh.id_);
			nodes_[*nh.id_].valid_ = false;
			valid_edges_ -= nodes_[*nh.id_].inEdges_;
			/* If the node was valid and had no entering edges,
			 * then remove it rightaway. O(1) */
			if (!nodes_[*nh.id_].inEdges_)
				removeNode(nh);
			--valid_nodes_;
		}
	}
	node_id nodeCount() const {
		return valid_nodes_;
	}
	edge_id edgeCount() const {
		return valid_edges_;
	}
	node_iterator beginNode() {return node_iterator(nodes_.begin(), nodes_.end());}
	const_node_iterator beginNode() const {return const_node_iterator(nodes_.begin(), nodes_.end());}
	const_node_iterator cbeginNode() const {return const_node_iterator(nodes_.cbegin(), nodes_.cend());}
	node_iterator endNode() {return node_iterator(nodes_.end(), nodes_.end());}
	const_node_iterator endNode() const {return const_node_iterator(nodes_.end(), nodes_.end());}
	const_node_iterator cendNode() const {return const_node_iterator(nodes_.cend(), nodes_.cend());}

	edge_iterator beginEdge() {return edge_iterator(this, edges_.begin(), edges_.end());}
	const_edge_iterator beginEdge() const {return const_edge_iterator(this, edges_.begin(), edges_.end());}
	const_edge_iterator cbeginEdge() const {return const_edge_iterator(this, edges_.cbegin(), edges_.cend());}
	edge_iterator endEdge() {return edge_iterator(this, edges_.end(), edges_.end());}
	const_edge_iterator endEdge() const {return const_edge_iterator(this, edges_.end(), edges_.end());}
	const_edge_iterator cendEdge() const {return const_edge_iterator(this, edges_.cend(), edges_.cend());}
};


template<typename NodeData, typename EdgeData>
struct graph_traits<ListDiGraph<NodeData, EdgeData>> {
	static const bool directedTag = ListDiGraph<NodeData, EdgeData>::directedTag;
	static const bool weightedTag = ListDiGraph<NodeData, EdgeData>::weightedTag;
	static const bool traversableTag = ListDiGraph<NodeData, EdgeData>::traversableTag;
	static const bool pathTag = ListDiGraph<NodeData, EdgeData>::pathTag;
	static const bool heuristicpathTag = ListDiGraph<NodeData, EdgeData>::heuristicpathTag;
	using node_handle = typename ListDiGraph<NodeData, EdgeData>::node_handle;
	using edge_handle = typename ListDiGraph<NodeData, EdgeData>::edge_handle;
	using node_data = NodeData;
	using edge_data = EdgeData;
	using node_iterator = typename ListDiGraph<NodeData, EdgeData>::node_iterator;
	using edge_iterator = typename ListDiGraph<NodeData, EdgeData>::edge_iterator;
	using edge_range = typename ListDiGraph<NodeData, EdgeData>::edge_range;
};
}

#endif

#ifndef LIST_GRAPH_H
#define LIST_GRAPH_H
#include <vector>
#include <list>
#include <unordered_map>
#include <cassert>
#include <algorithm>
#include <memory>

#include "NodeData.h"
#include "EdgeData.h"
#include "NodeTraits.h"
#include "EdgeTraits.h"
#include "GraphTraits.h"
#include "Handle.h"

namespace graphlib {

template<typename NodeData,
         typename EdgeData>
class ListGraph {
	struct Node {
		template<typename... Args>
		Node(node_id nid, Args&&... args) : nid_(std::make_unique<node_id>(nid)),
		    data_(std::forward<Args>(args)...) {
			handle_.id_ = nid_.get();
		}
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
			swap(handle_, node.handle_);
			nid_.swap(node.nid_);
			swap(*nid_, *node.nid_);
		}
		const NodeHandle &getHandle() const {
			return handle_;
		}

	private:
		NodeHandle handle_;
		std::unique_ptr<node_id> nid_;
		NodeData data_;

		friend class ListGraph;
	};

	struct Edge {
		template<typename... Args>
		Edge(const NodeHandle& fst,
		     const NodeHandle& snd,
		     edge_id eid,
		     Args&&... args) : valid_(true), fst_(fst), snd_(snd),
		    eid_(std::make_unique<edge_id>(eid)),
		    data_(std::forward<Args>(args)...) {
			handle_.id_ = eid_.get();
		}
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
			swap(fst_, edge.fst_);
			swap(snd_, edge.snd_);
			swap(handle_, edge.handle_);
			swap(last_valid_, edge.last_valid_);
			swap(valid_, edge.valid_);
		}
		const EdgeHandle &getHandle() const {
			return handle_;
		}
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

	using edge_range_iterator = typename std::vector<Edge>::iterator;
	using cedge_range_iterator = typename std::vector<Edge>::const_iterator;

	class ConstEdgeIterator;
	class EdgeIterator {
		ListGraph *graph_;
		edge_range_iterator cit_;
		edge_range_iterator eit_;
	public:
		void find_next() {
			while (cit_ != eit_ && !graph_->hasEdge(cit_->getHandle())) {
				EdgeHandle eh = cit_->getHandle();
				++cit_;
				graph_->removeEdge(eh);
			}
		}
		void increment() {
			if (cit_ != eit_)
				++cit_;
			find_next();
		}

	public:
		using value_type = const EdgeHandle;
		using reference = const EdgeHandle&;
		using pointer = const EdgeHandle*;
		using difference_type = std::ptrdiff_t;
		using iterator_category = std::forward_iterator_tag;

		EdgeIterator() : graph_(nullptr), cit_(nullptr), eit_(nullptr) {}
		EdgeIterator(ListGraph *graph,
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
		reference operator*() {return cit_->getHandle();}
		pointer operator->() {return &cit_->getHandle();}
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
		const ListGraph *graph_;
		cedge_range_iterator cit_;
		cedge_range_iterator eit_;

		void find_next() {
			while (cit_ != eit_ && !graph_->hasEdge(cit_->getHandle()))
				++cit_;
		}
		void increment() {
			if (cit_ != eit_)
				++cit_;
			find_next();
		}
	public:
		using value_type = const EdgeHandle;
		using reference = const EdgeHandle&;
		using pointer = const EdgeHandle*;
		using difference_type = std::ptrdiff_t;
		using iterator_category = std::forward_iterator_tag;

		ConstEdgeIterator() : graph_(nullptr), cit_(nullptr), eit_(nullptr) {}
		ConstEdgeIterator(const ListGraph *graph,
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
		reference operator*() {return cit_->getHandle();}
		pointer operator->() {return &cit_->getHandle();}
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

	class NodeIterator {
		using node_iterator = typename std::vector<Node>::const_iterator;
		node_iterator cit_;
	public:
		using value_type = const NodeHandle;
		using reference = const NodeHandle&;
		using pointer = const NodeHandle*;
		using difference_type = std::ptrdiff_t;
		using iterator_category = std::forward_iterator_tag;

		NodeIterator() = default;
		NodeIterator(node_iterator cit) : cit_(cit) {}
		NodeIterator& operator++() {
			++cit_;
			return *this;
		}
		NodeIterator operator++(int) {
			auto cp(*this);
			++cit_;
			return cp;
		}
		reference operator*() {
			return cit_->getHandle();
		}
		pointer operator->() {
			return &cit_->getHandle();
		}
		bool operator==(const NodeIterator &it) const {
			return cit_ == it.cit_;
		}
		bool operator!=(const NodeIterator &it) const {
			return !(*this == it);
		}
	};

	template<typename T>
	class ListWrapper {
		ListGraph *graph_;
		std::list<T> list_;
		using list_iterator = typename std::list<T>::const_iterator;

		class WrapIterator {
			using traits = std::iterator_traits<list_iterator>;
			ListGraph *graph_;
			list_iterator cit_;
			list_iterator eit_;

			void find_next() {
				while (cit_ != eit_ && !graph_->hasEdge(*cit_)) {
					EdgeHandle eh = *cit_;
					++cit_;
					graph_->removeEdge(eh);
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

			WrapIterator(ListGraph *graph,
			             list_iterator bit,
			             list_iterator eit) : graph_(graph), cit_(bit), eit_(eit) {
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

		ListWrapper(ListGraph *graph) : graph_(graph) {}
		ListWrapper() : graph_(nullptr) {}
		iterator begin() const {return iterator(graph_, list_.begin(), list_.end());}
		iterator end() const {return iterator(graph_, list_.end(), list_.end());}
		size_type size() const {
			size_type valid = 0;
			for (auto it = list_.begin(); it != list_.end(); ++it) {
				if (graph_->hasEdge(*it))
					++valid;
			}
			return valid;
		}
		bool empty() const {return size() == 0;}

	private:
		void push_back(const T& val) {list_.push_back(val);}
		void push_back(T&& val) {list_.push_back(std::move(val));}
		void erase(list_iterator it) {list_.erase(it);}

		template<typename, typename>
		friend class ListGraph;
	};

	std::vector<Node> nodes_;
	std::vector<Edge> edges_;
	std::unordered_map<node_id, ListWrapper<EdgeHandle>> edges_map_;
	size_t valid_edges_;

public:
	static const bool directedTag = false;
	static const bool weightedTag = edge_traits<EdgeData>::weighted;
	static const bool traversableTag = node_traits<NodeData>::color && node_traits<NodeData>::predecessor;
	static const bool pathTag = traversableTag && node_traits<NodeData>::distance;
	static const bool heuristicpathTag = pathTag && node_traits<NodeData>::location;

	using node_handle = NodeHandle;
	using edge_handle = EdgeHandle;
	using adj_range = ListWrapper<EdgeHandle>;
	using node_data = NodeData;
	using edge_data = EdgeData;
	using node_iterator = NodeIterator;
	using const_node_iterator = NodeIterator;
	using edge_iterator = EdgeIterator;
	using const_edge_iterator = ConstEdgeIterator;
	using weight_type = typename edge_traits<EdgeData>::weight_type;
	using distance_type = typename node_traits<NodeData>::distance_type;

	ListGraph() : valid_edges_(0) {}
	ListGraph(size_t nonodes) : valid_edges_(0) {
		while(nonodes--)
			addNode();
	}
	ListGraph(const ListGraph&) = delete;
	ListGraph& operator=(const ListGraph&) = delete;

	// Methods
	bool hasEdge(const NodeHandle &nha, const NodeHandle &nhb) const {
		if (*nha.id_ >= nodes_.size() && *nhb.id_ >= nodes_.size())
			return false;
		auto &list = edges_map_.find(*nha.id_)->second;
		auto eit = std::find_if(list.begin(), list.end(), [this, &nhb](const EdgeHandle &eh) {
			return nhb == edges_[*eh.id_].fst_ || nhb == edges_[*eh.id_].snd;
		});
		return eit != list.end();
	}
	bool hasEdge(const EdgeHandle& eh) const {
		if (*eh.id_ < edges_.size() && edges_[*eh.id_].valid_)
			return true;
		return false;
	}
	bool hasNode(const NodeHandle &nh) const {
		if (*nh.id_ < nodes_.size())
			return true;
		return false;
	}
	size_t degree(const NodeHandle &nh) const {
		assert(*nh.id_ < nodes_.size());
		return edges_map_.find(*nh.id_)->second.size();
	}
	EdgeData &getEdge(const EdgeHandle& eh) {
		assert(*eh.id_ < edges_.size() && edges_[*eh.id_].valid_);
		return edges_[*eh.id_].data_;
	}
	const EdgeData &getEdge(const EdgeHandle& eh) const {
		assert(*eh.id_ < edges_.size() && edges_[*eh.id_].valid_);
		return edges_[*eh.id_].data_;
	}
	NodeData &getNode(const NodeHandle& nh) {
		assert(*nh.id_ < nodes_.size());
		return nodes_[*nh.id_].data_;
	}
	const NodeData &getNode(const NodeHandle& nh) const {
		assert(*nh.id_ < nodes_.size());
		return nodes_[*nh.id_].data_;
	}
	NodeHandle getOther(const EdgeHandle &eh, const NodeHandle &nh) const {
		assert(*eh.id_ < edges_.size() && *nh.id_ < nodes_.size() && edges_[*eh.id_].valid_);
		return edges_[*eh.id_].fst_ == nh? edges_[*eh.id_].snd_ : edges_[*eh.id_].fst_;
	}
	std::pair<NodeHandle, NodeHandle> getBoth(const EdgeHandle &eh) const {
		assert(*eh.id_ < edges_.size() && edges_[*eh.id_].valid_);
		return std::make_pair(edges_[*eh.id_].fst_, edges_[*eh.id_].snd_);
	}
	NodeData &getOtherNode(const EdgeHandle &eh, const NodeHandle &nh) {
		assert(*eh.id_ < edges_.size() && *nh.id_ < nodes_.size() &&
		       edges_[*eh.id_].valid_);
		const NodeHandle &other = getOther(eh, nh);
		return nodes_[*other.id_].data_;
	}
	const NodeData &getOtherNode(const EdgeHandle &eh, const NodeHandle &nh) const {
		assert(*eh.id_ < edges_.size() && *nh.id_ < nodes_.size() &&
		       edges_[*eh.id_].valid_);
		const NodeHandle &other = getOther(eh, nh);
		return nodes_[*other.id_].data_;
	}
	adj_range &operator[](const NodeHandle &nh) {
		assert(*nh.id_ < nodes_.size());
		return edges_map_[*nh.id_];
	}
	const adj_range &operator[](const NodeHandle &nh) const {
		assert(*nh.id_ < nodes_.size());
		return edges_map_.find(*nh.id_)->second;
	}
	node_id nodeCount() const {
		return nodes_.size();
	}
	edge_id edgeCount() const {
		return valid_edges_;
	}
	node_iterator beginNode() {return node_iterator(nodes_.begin());}
	const_node_iterator beginNode() const {return const_node_iterator(nodes_.begin());}
	const_node_iterator cbeginNode() const {return const_node_iterator(nodes_.cbegin());}
	node_iterator endNode() {return node_iterator(nodes_.end());}
	const_node_iterator endNode() const {return const_node_iterator(nodes_.end());}
	const_node_iterator cendNode() const {return const_node_iterator(nodes_.cend());}

	edge_iterator beginEdge() {return edge_iterator(this, edges_.begin(), edges_.end());}
	const_edge_iterator beginEdge() const {return const_edge_iterator(this, edges_.begin(), edges_.end());}
	const_edge_iterator cbeginEdge() const {return const_edge_iterator(this, edges_.cbegin(), edges_.cend());}
	edge_iterator endEdge() {return edge_iterator(this, edges_.end(), edges_.end());}
	const_edge_iterator endEdge() const {return const_edge_iterator(this, edges_.end(), edges_.end());}
	const_edge_iterator cendEdge() const {return const_edge_iterator(this, edges_.cend(), edges_.cend());}

	template<typename EData = EdgeData>
	std::enable_if_t<EData::weighted, void>
	setWeight(const EdgeHandle& eh,
	          typename EData::weight_type weight) {
		assert(*eh.id_ < edges_.size());
		auto &data = getEdge(eh);
		data.weight_ = weight;
	}
	template<typename EData = EdgeData>
	std::enable_if_t<EData::weighted, typename EData::weight_type>
	getWeight(const EdgeHandle& eh) const {
		assert(*eh.id_ < edges_.size());
		auto &data = getEdge(eh);
		return data.weight_;
	}

	template<typename... Args>
	NodeHandle addNode(Args&&... args) {
		node_id nid = nodes_.size();
		Node &node = nodes_.emplace_back(nid, std::forward<Args>(args)...);
		NodeHandle nh = node.getHandle();
		edges_map_.emplace(*nh.id_, this);
		return nh;
	}
	template<typename... Args>
	EdgeHandle addEdge(const NodeHandle &nha,
	                   const NodeHandle &nhb,
	                   Args&&... args) {
		assert(*nha.id_ < nodes_.size() && *nhb.id_ < nodes_.size());
		edge_id eid = edges_.size();
		Edge& edge = edges_.emplace_back(nha, nhb, eid, std::forward<Args>(args)...);
		edges_map_[*nha.id_].push_back(edge.getHandle());
		edges_map_[*nhb.id_].push_back(edge.getHandle());
		++valid_edges_;
		return edge.getHandle();
	}
	template<typename... Args, typename EData = EdgeData>
	std::enable_if_t<EData::weighted, EdgeHandle>
	addEdge(const NodeHandle &nha,
	        const NodeHandle &nhb,
	        typename EData::weight_type weight,
	        Args&&... args) {
		assert(*nha.id_ < nodes_.size() && *nhb.id_ < nodes_.size());
		edge_id eid = edges_.size();
		Edge& edge = edges_.emplace_back(nha, nhb, eid, weight, std::forward<Args>(args)...);
		edges_map_[*nha.id_].push_back(edge.getHandle());
		edges_map_[*nhb.id_].push_back(edge.getHandle());
		++valid_edges_;
		return edge.getHandle();
	}
	void removeNode(const NodeHandle &nh) {
		assert(*nh.id_ < nodes_.size());
		/* Go through all outgoing edges, mark them invalid and set last_valid
		 * NodeHandle to remaining valid node. If some edge is already invalid,
		 * it is remove instantly. O(V) */
		for (auto& eh : edges_map_[*nh.id_].list_) {
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
		edges_map_.erase(*nh.id_);
		Node &bnode = nodes_.back();
		if (bnode.getHandle() != nh) {
			auto nh_map = edges_map_.extract(*bnode.nid_);
			nh_map.key() = *nh.id_;
			edges_map_.insert(std::move(nh_map));
			nodes_[*nh.id_].swap(bnode);
		}
		nodes_.pop_back();
	}
	void removeEdge(const EdgeHandle &eh) {
		assert(*eh.id_ < edges_.size());
		if (edges_[*eh.id_].valid_) {
			/* Remove EdgeHandles from lists of two adjacent nodes. O(V)*/
			NodeHandle &fst = edges_[*eh.id_].fst_;
			NodeHandle &snd = edges_[*eh.id_].snd_;
			auto &fst_list = edges_map_[*fst.id_].list_;
			auto &snd_list = edges_map_[*snd.id_].list_;
			auto it = std::find_if(fst_list.begin(), fst_list.end(), [&eh](const EdgeHandle &list_eh) {
				return eh == list_eh;
			});
			edges_map_[*fst.id_].erase(it);
			it = std::find_if(snd_list.begin(), snd_list.end(), [&eh](const EdgeHandle & list_eh) {
				return eh == list_eh;
			});
			edges_map_[*snd.id_].erase(it);
			--valid_edges_;
		} else {
			/* Remove EdgeHandle from list of remaining valid node. O(V)*/
			NodeHandle &valid = edges_[*eh.id_].last_valid_;
			auto &list = edges_map_[*valid.id_].list_;
			auto it = std::find_if(list.begin(), list.end(), [&eh](const EdgeHandle &list_eh){
				return eh == list_eh;
			});
			edges_map_[*valid.id_].erase(it);
		}
		/* Remove corresponding Edge from edges_ vector O(1)*/
		Edge &bedge = edges_.back();
		if (bedge.getHandle() != eh)
			edges_[*eh.id_].swap(edges_.back());
		edges_.pop_back();
	}
};

template<typename NodeData, typename EdgeData>
struct graph_traits<ListGraph<NodeData, EdgeData>> {
	static const bool directedTag = ListGraph<NodeData, EdgeData>::directedTag;
	static const bool weightedTag = ListGraph<NodeData, EdgeData>::weightedTag;
	static const bool traversableTag = ListGraph<NodeData, EdgeData>::traversableTag;
	static const bool pathTag = ListGraph<NodeData, EdgeData>::pathTag;
	static const bool heuristicpathTag = ListGraph<NodeData, EdgeData>::heuristicpathTag;
	using node_handle = typename ListGraph<NodeData, EdgeData>::node_handle;
	using edge_handle = typename ListGraph<NodeData, EdgeData>::edge_handle;
	using node_data = NodeData;
	using edge_data = EdgeData;
	using node_iterator = typename ListGraph<NodeData, EdgeData>::node_iterator;
	using const_node_iterator = typename ListGraph<NodeData, EdgeData>::const_node_iterator;
	using edge_iterator = typename ListGraph<NodeData, EdgeData>::edge_iterator;
	using const_edge_iterator = typename ListGraph<NodeData, EdgeData>::const_edge_iterator;
	using adj_range = typename ListGraph<NodeData, EdgeData>::adj_range;
	using adj_iterator = typename adj_range::iterator;
	using const_adj_iterator = typename adj_range::const_iterator;
	using weight_type = typename ListGraph<NodeData, EdgeData>::weight_type;
	using distance_type = typename ListGraph<NodeData, EdgeData>::distance_type;
};

}
#endif

#ifndef LIST_GRAPH_H
#define LIST_GRAPH_H
#include <vector>
#include <list>
#include <unordered_map>
#include <cassert>
#include <algorithm>
#include <memory>
#include <set>
#include <utility>
#include <boost/range.hpp>

#include "NodeData.h"
#include "EdgeData.h"
#include "NodeTraits.h"
#include "EdgeTraits.h"
#include "GraphTraits.h"
#include "Handle.h"
#include "GraphIterators.h"

namespace graphlib {

template<typename NodeData,
         typename EdgeData>
class ListGraph {
	struct Node {
		template<typename... Args>
		explicit Node(node_id nid, Args&&... args) :
		    nid_(std::make_unique<node_id>(nid)),
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
		using handle_type = NodeHandle;

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

	class EdgeIterator;
	class ConstEdgeIterator;
	using EdgeIteratorTraits = EdgeTraits<erange_iterator>;
	using EdgeIteratorBase = ForwardGraphIterator<ListGraph, EdgeIterator, EdgeIteratorTraits>;

	class EdgeIterator : public EdgeIteratorBase {
		using base_iterator = EdgeIteratorBase;
		using etraits = EdgeIteratorTraits;
	public:
		using value_type = typename etraits::value_type;
		using reference = typename etraits::reference;
		using pointer = typename etraits::pointer;
		using difference_type = typename etraits::difference_type;
		using iterator_category = typename etraits::iterator_category;

		EdgeIterator() = default;
		EdgeIterator(ListGraph *graph,
		             erange_iterator bit,
		             erange_iterator eit) : base_iterator(graph, bit, eit) {}
	bool operator==(const ConstEdgeIterator &it) const {
		return graph_ == it.graph_ && cit_ == it.cit_;
	}
	bool operator!=(const ConstEdgeIterator &it) const {
		return !(*this == it);
	}
	private:
	    using EdgeIteratorBase::graph_;
	    using EdgeIteratorBase::cit_;
	    using EdgeIteratorBase::eit_;
	    void find_next() {
			while (cit_ != eit_ && !graph_->hasEdge(cit_->getHandle())) {
				EdgeHandle eh = cit_->getHandle();
				++cit_;
				graph_->removeEdge(eh);
			}
		}
		reference dereference() const {
			return cit_->getHandle();
		}
		template<typename, typename, typename>
		friend class ForwardGraphIterator;
		friend class ConstEdgeIterator;
	};

	using ConstEdgeIteratorTraits = EdgeTraits<cerange_iterator>;
	using ConstEdgeIteratorBase = ForwardGraphIterator<const ListGraph, ConstEdgeIterator, ConstEdgeIteratorTraits>;

	class ConstEdgeIterator : public ConstEdgeIteratorBase {
		using base_iterator = ConstEdgeIteratorBase;
		using etraits = ConstEdgeIteratorTraits;
		using ConstEdgeIteratorBase::graph_;
		using ConstEdgeIteratorBase::cit_;
		using ConstEdgeIteratorBase::eit_;

	public:
		using value_type = typename etraits::value_type;
		using reference = typename etraits::reference;
		using pointer = typename etraits::pointer;
		using difference_type = typename etraits::difference_type;
		using iterator_category = typename etraits::iterator_category;

		ConstEdgeIterator() = default;
		ConstEdgeIterator(const ListGraph *graph,
		                  cerange_iterator bit,
		                  cerange_iterator eit) : base_iterator(graph, bit, eit) {}
		explicit ConstEdgeIterator(const EdgeIterator &it) : base_iterator(it.graph_, it.cit_, it.eit_) {}
		ConstEdgeIterator& operator=(const EdgeIterator &it) {
			graph_ = it.graph_;
			cit_ = it.cit_;
			eit_ = it.eit_;
			return *this;
		}
		bool operator==(const EdgeIterator &it) const {
			return graph_ == it.graph_ && cit_ == it.cit_;
		}
		bool operator!=(const EdgeIterator &it) const {
			return !(*this == it);
		}
	private:
		void find_next() {
			while (cit_ != eit_ && !graph_->hasEdge(cit_->getHandle()))
				++cit_;
		}
		reference dereference() const {
			return cit_->getHandle();
		}
		template<typename, typename, typename>
		friend class ForwardGraphIterator;
		friend class EdgeIterator;
	};

	template<typename T>
	class ListWrapper {
		ListGraph *graph_;
		std::list<T> list_;
		using list_iterator = typename std::list<T>::const_iterator;

		class WrapIterator;
		using WrapIteratorTraits = EdgeTraits<list_iterator>;
		using WrapIteratorBase = ForwardGraphIterator<ListGraph, WrapIterator, WrapIteratorTraits>;

		class WrapIterator : public WrapIteratorBase {
			using base_iterator = WrapIteratorBase;
			using etraits = WrapIteratorTraits;
			using WrapIteratorBase::graph_;
			using WrapIteratorBase::cit_;
			using WrapIteratorBase::eit_;
		public:
			using value_type = typename etraits::value_type;
			using reference = typename etraits::reference;
			using pointer = typename etraits::pointer;
			using difference_type = typename etraits::difference_type;
			using iterator_category = typename etraits::iterator_category;

			WrapIterator() = default;
			WrapIterator(ListGraph *graph,
			             list_iterator bit,
			             list_iterator eit) : base_iterator(graph, bit, eit) {}
		private:
			void find_next() {
				while (cit_ != eit_ && !graph_->hasEdge(*(cit_))) {
					EdgeHandle eh = *(cit_);
					++cit_;
					graph_->removeEdge(eh);
				}
			}
			reference dereference() const {
				return *cit_;
			}
			template<typename, typename, typename>
			friend class ForwardGraphIterator;
		};

	public:
		using value_type = typename std::list<T>::value_type;
		using reference = typename std::list<T>::reference;
		using const_reference = typename std::list<T>::const_reference;
		using iterator = WrapIterator;
		using const_iterator = WrapIterator;
		using difference_type = typename std::iterator_traits<list_iterator>::difference_type;
		using size_type = typename std::list<T>::size_type;

		ListWrapper() : graph_(nullptr) {}
		explicit ListWrapper(ListGraph *graph) : graph_(graph) {}
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
	std::vector<std::unique_ptr<ListWrapper<EdgeHandle>>> edges_map_;
	size_t valid_edges_{0};

public:
	static const bool directedTag = false;
	static const bool weightedTag = edge_traits<EdgeData>::weighted;
	static const bool traversableTag = node_traits<NodeData>::color && node_traits<NodeData>::predecessor;
	static const bool pathTag = traversableTag && node_traits<NodeData>::distance;
	static const bool heuristicpathTag = pathTag && node_traits<NodeData>::location;

	using node_handle = NodeHandle;
	using edge_handle = EdgeHandle;
	using adj_range = ListWrapper<EdgeHandle>;
	using adj_iterator = typename ListWrapper<EdgeHandle>::iterator;
	using const_adj_iterator = typename ListWrapper<EdgeHandle>::const_iterator;
	using node_data = NodeData;
	using edge_data = EdgeData;
	using node_iterator = ForwardIterator<NodeTraits<cnrange_iterator>>;
	using const_node_iterator = ForwardIterator<NodeTraits<cnrange_iterator>>;
	using edge_iterator = EdgeIterator;
	using const_edge_iterator = ConstEdgeIterator;
	using weight_type = typename edge_traits<EdgeData>::weight_type;
	using distance_type = typename node_traits<NodeData>::distance_type;
	using location_type = typename node_traits<NodeData>::location_type;
	using priority_type = typename node_traits<NodeData>::priority_type;

	ListGraph() = default;
	explicit ListGraph(size_t nonodes) {
		while(nonodes--)
			addNode();
	}
	ListGraph(const ListGraph&) = delete;
	ListGraph& operator=(const ListGraph&) = delete;

	bool hasEdge(const NodeHandle &nha, const NodeHandle &nhb) const {
		if (*nha.id_ >= nodes_.size() && *nhb.id_ >= nodes_.size())
			return false;
		auto &list = *edges_map_[*nha.id_];
		auto eit = std::find_if(list.begin(), list.end(), [&, this](const EdgeHandle &eh) {
			return nhb == getOther(eh, nha);
		});
		return eit != list.end();
	}
	bool hasEdge(const EdgeHandle& eh) const {
		return (*eh.id_ < edges_.size() &&
		        edges_[*eh.id_].valid_) ? true : false;
	}
	bool hasNode(const NodeHandle &nh) const {
		return (*nh.id_ < nodes_.size()) ? true : false;
	}
	size_t degree(const NodeHandle &nh) const {
		assert(*nh.id_ < nodes_.size());
		return edges_map_[*nh.id_]->size();
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
		return *edges_map_[*nh.id_];
	}
	const adj_range &operator[](const NodeHandle &nh) const {
		assert(*nh.id_ < nodes_.size());
		return *edges_map_[*nh.id_];
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

	boost::iterator_range<node_iterator> nodes() {
		return boost::make_iterator_range(beginNode(), endNode());
	}
	boost::iterator_range<const_node_iterator> nodes() const {
		return boost::make_iterator_range(beginNode(), endNode());
	}
	boost::iterator_range<const_node_iterator> cnodes() const {
		return boost::make_iterator_range(cbeginNode(), cendNode());
	}

	boost::iterator_range<edge_iterator> edges() {
		return boost::make_iterator_range(beginEdge(), endEdge());
	}
	boost::iterator_range<const_edge_iterator> edges() const {
		return boost::make_iterator_range(beginEdge(), endEdge());
	}
	boost::iterator_range<const_edge_iterator> cedges() const {
		return boost::make_iterator_range(cbeginEdge(), cendEdge());
	}

	template<typename EData = EdgeData>
	std::enable_if_t<EData::weighted, void>
	setWeight(const EdgeHandle& eh,
	          typename edge_traits<EData>::weight_type weight) {
		assert(*eh.id_ < edges_.size());
		auto &data = getEdge(eh);
		data.weight_ = weight;
	}
	template<typename EData = EdgeData>
	std::enable_if_t<EData::weighted>
	modWeight(const EdgeHandle& eh,
	          typename edge_traits<EData>::weight_type weight) {
		assert(*eh.id_ < edges_.size());
		auto &edata = getEdge(eh);
		edata.weight_ += weight;
	}
	template<typename EData = EdgeData>
	std::enable_if_t<EData::weighted, typename EData::weight_type>
	getWeight(const EdgeHandle& eh) const {
		assert(*eh.id_ < edges_.size());
		auto &data = getEdge(eh);
		return data.weight_;
	}
	template<typename Graph = ListGraph<NodeData, EdgeData>>
	std::enable_if_t<Graph::traversableTag, Color>
	getNodeColor(const node_handle &nh) const {
		assert(*nh.id_ < nodes_.size());
		auto &data = getNode(nh);
		return data.color_;
	}
	template<typename Graph = ListGraph<NodeData, EdgeData>>
	std::enable_if_t<Graph::traversableTag>
	setNodeColor(const node_handle &nh,
	             Color color) {
		assert(*nh.id_ < nodes_.size());
		auto &data = getNode(nh);
		data.color_ = color;
	}
	template<typename Graph = ListGraph<NodeData, EdgeData>>
	std::enable_if_t<Graph::traversableTag, node_handle>
	getNodePred(const typename graph_traits<Graph>::node_handle &nh) const {
		assert(*nh.id_ < nodes_.size());
		auto &data = getNode(nh);
		return data.pred_;
	}
	template<typename Graph = ListGraph<NodeData, EdgeData>>
	std::enable_if_t<Graph::traversableTag>
	setNodePred(const typename graph_traits<Graph>::node_handle &nh,
	            const typename graph_traits<Graph>::node_handle &pred) {
		assert(*nh.id_ < nodes_.size());
		auto &data = getNode(nh);
		data.pred_ = pred;
	}
	template<typename Graph = ListGraph<NodeData, EdgeData>>
	std::enable_if_t<Graph::pathTag,
	                 typename graph_traits<Graph>::distance_type>
	getNodeDist(const typename graph_traits<Graph>::node_handle &nh) const {
		assert(*nh.id_ < nodes_.size());
		auto &data = getNode(nh);
		return data.dist_;
	}
	template<typename Graph = ListGraph<NodeData, EdgeData>>
	std::enable_if_t<Graph::pathTag>
	setNodeDist(const typename graph_traits<Graph>::node_handle &nh,
	            typename graph_traits<Graph>::distance_type dist) {
		assert(*nh.id_ < nodes_.size());
		auto &data = getNode(nh);
		data.dist_ = dist;
	}
	template<typename Graph = ListGraph<NodeData, EdgeData>>
	std::enable_if_t<Graph::heuristicpathTag,
	                 const typename graph_traits<Graph>::location_type &>
	getNodeLoc(const typename graph_traits<Graph>::node_handle &nh) const {
		assert(*nh.id_ < nodes_.size());
		auto &data = getNode(nh);
		return data.loc_;
	}
	template<typename Graph = ListGraph<NodeData, EdgeData>>
	std::enable_if_t<Graph::heuristicpathTag>
	setNodeLoc(const typename graph_traits<Graph>::node_handle &nh,
	           const typename graph_traits<Graph>::location_type &loc) {
		assert(*nh.id_ < nodes_.size());
		auto &data = getNode(nh);
		data.loc_ = loc;
	}
	template<typename Graph = ListGraph<NodeData, EdgeData>>
	std::enable_if_t<Graph::heuristicpathTag,
	                 typename graph_traits<Graph>::priority_type>
	getNodePrio(const typename graph_traits<Graph>::node_handle &nh) const {
		assert(*nh.id_ < nodes_.size());
		auto &data = getNode(nh);
		return data.prio_;
	}
	template<typename Graph = ListGraph<NodeData, EdgeData>>
	std::enable_if_t<Graph::heuristicpathTag>
	setNodePrio(const typename graph_traits<Graph>::node_handle &nh,
	            typename graph_traits<Graph>::priority_type prio) {
		assert(*nh.id_ < nodes_.size());
		auto &data = getNode(nh);
		data.prio_ = prio;
	}
	template<typename... Args>
	NodeHandle addNode(Args&&... args) {
		node_id nid = nodes_.size();
		Node &node = nodes_.emplace_back(nid, std::forward<Args>(args)...);
		NodeHandle nh = node.getHandle();
		edges_map_.emplace_back(std::make_unique<ListWrapper<EdgeHandle>>(this));
		return nh;
	}
	template<typename... Args>
	EdgeHandle addEdge(const NodeHandle &nha,
	                   const NodeHandle &nhb,
	                   Args&&... args) {
		assert(*nha.id_ < nodes_.size() && *nhb.id_ < nodes_.size());
		edge_id eid = edges_.size();
		Edge& edge = edges_.emplace_back(nha, nhb, eid, std::forward<Args>(args)...);
		edges_map_[*nha.id_]->push_back(edge.getHandle());
		edges_map_[*nhb.id_]->push_back(edge.getHandle());
		++valid_edges_;
		return edge.getHandle();
	}
	void removeNode(const NodeHandle &nh) {
		assert(*nh.id_ < nodes_.size());
		/* Go through all outgoing edges, mark them invalid and set last_valid
		 * NodeHandle to remaining valid node. If some edge is already invalid,
		 * it is remove instantly. O(V) */
		for (auto& eh : edges_map_[*nh.id_]->list_) {
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
	void removeEdge(const EdgeHandle &eh) {
		assert(*eh.id_ < edges_.size());
		if (edges_[*eh.id_].valid_) {
			/* Remove EdgeHandles from lists of two adjacent nodes. O(V)*/
			NodeHandle &fst = edges_[*eh.id_].fst_;
			NodeHandle &snd = edges_[*eh.id_].snd_;
			auto &fst_list = edges_map_[*fst.id_]->list_;
			auto &snd_list = edges_map_[*snd.id_]->list_;
			auto it = std::find_if(fst_list.begin(), fst_list.end(), [&eh](const EdgeHandle &list_eh) {
				return eh == list_eh;
			});
			edges_map_[*fst.id_]->erase(it);
			it = std::find_if(snd_list.begin(), snd_list.end(), [&eh](const EdgeHandle & list_eh) {
				return eh == list_eh;
			});
			edges_map_[*snd.id_]->erase(it);
			--valid_edges_;
		} else {
			/* Remove EdgeHandle from list of remaining valid node. O(V)*/
			NodeHandle &valid = edges_[*eh.id_].last_valid_;
			auto &list = edges_map_[*valid.id_]->list_;
			auto it = std::find_if(list.begin(), list.end(), [&eh](const EdgeHandle &list_eh){
				return eh == list_eh;
			});
			edges_map_[*valid.id_]->erase(it);
		}
		/* Remove corresponding Edge from edges_ vector O(1)*/
		Edge &bedge = edges_.back();
		if (bedge.getHandle() != eh)
			edges_[*eh.id_].swap(edges_.back());
		edges_.pop_back();
	}
	int exportGraph(std::string path) const {
		if (!path.empty() && (path[0] != '/' || path.compare(0, 2, "./")))
			path.insert(0, "./");
		else if (path.empty()) {
			std::cerr << "path string is empty" << std::endl;
			return -1;
		}
		std::ofstream ofile(path);
		if (ofile.is_open()) {
			ofile << "graph {\n";
			for (auto eh : edges()) {
				auto [fst, snd] = getBoth(eh);
				ofile << fst.getId();
				ofile << " -- ";
				ofile << snd.getId() << ";\n";
			}
			ofile << "}";
		} else {
			std::cerr << "Failed to open " << path << std::endl;
			return -1;
		}
		ofile.close();
		return 0;
	}
	int exportShortestPath(std::string path, node_handle target) const {
		if (!path.empty() && (path[0] != '/' || path.compare(0, 2, "./")))
			path.insert(0, "./");
		else if (path.empty()) {
			std::cerr << "path string is empty" << std::endl;
			return -1;
		}
		std::ofstream ofile(path);
		if (ofile.is_open()) {
			std::set<std::pair<node_handle, node_handle>> shedges;
			while (getNodePred(target) != node_handle()) {
				node_handle pred = getNodePred(target);
				shedges.insert(std::make_pair(target, pred));
				target = pred;
			}
			ofile << "graph {\n";
			for (auto eh : edges()) {
				auto [fst, snd] = getBoth(eh);
				ofile << fst.getId() << " -- " << snd.getId();
				if (shedges.find(std::make_pair(fst, snd)) != shedges.end() ||
				        shedges.find(std::make_pair(snd, fst)) != shedges.end())
				    ofile << "[color=red,pendwidth=3.0]";
				ofile << ";\n";
			}
			ofile << '}';
		} else {
			std::cerr << "Failed to open " << path << std::endl;
			return -1;
		}
		ofile.close();
		return 0;
	}
};

template<typename NodeData, typename EdgeData>
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

} //namespace graphlib
#endif

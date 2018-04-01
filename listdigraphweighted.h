namespace graphlib {

class ListDiGraphWeighted {
	struct Node;
	struct Edge;
	template<class>
	struct Iterator;

public:
	using node_id = std::size_t;
	using edge_id = std::size_t;
	using weight_type = long int;
	using edge_range = std::list<edge_id>;
	using node_iterator = Iterator<typename std::vector<Node>::iterator>;
	using const_node_iterator = Iterator<typename std::vector<Node>::const_iterator>;
	using edge_iterator = Iterator<typename std::vector<Edge>::iterator>;
	using const_edge_iterator = Iterator<typename std::vector<Edge>::const_iterator>;

	static const bool directedTag = true;
	static const bool weightedTag = true;
	static const unsigned int defaultWeight = 1;

private:
	std::vector<Node> nodes_;
	std::vector<Edge> edges_;
	std::list<node_id> free_nid_;
	std::list<edge_id> free_eid_;
	std::unordered_map<node_id, std::list<edge_id>> edges_map_;

	struct Edge {
		node_id src_nid_;
		node_id tg_nid_;
		weight_type weight_;
		bool valid_{true};
		Edge(node_id src_nid, node_id tg_nid, weight_type weight) :
		    src_nid_(src_nid), tg_nid_(tg_nid), weight_(weight) {}
		bool operator==(const Edge& edge) const {
			return src_nid_ == edge.src_nid_ &&
			        tg_nid_ == edge.tg_nid_ &&
			        valid_  == edge.valid_ &&
			        weight_ == edge.weight_;
		}
		bool operator!=(const Edge& edge) const {
			return !(*this == edge);
		}
		bool operator<(const Edge& edge) const {
			return weight_ < edge.weight_;
		}
	};

	struct Node {
		enum class Color {WHITE, GRAY, BLACK};
		Color color_{Color::WHITE};
		long int dist_{0};
		node_id pred_{static_cast<node_id>(-1)};
		edge_id inEdges_{0};
		bool valid_{true};

		bool operator==(const Node& node) const {
			return color_ == node.color_ &&
			        dist_ == node.dist_ &&
			        pred_ == node.pred_ &&
			        inEdges_ == node.inEdges_ &&
			        valid_ == node.valid_;
		}
		bool operator!=(const Node& node) const {
			return !(*this == node);
		}
	};

	template<typename OIterator>
	struct Iterator {
	private:
		using traits = std::iterator_traits<OIterator>;
		OIterator cit_{nullptr};
		OIterator eit_{nullptr};
		void increment() {
			++cit_;
			while(cit_ != eit_ && cit_->valid_)
				++cit_;
		}
		template<typename>
		friend struct Iterator;
	public:
		using value_type = typename traits::value_type;
		using reference = typename traits::reference;
		using pointer = typename traits::pointer;
		using difference_type = typename traits::difference_type;
		using iterator_category = std::forward_iterator_tag;

		Iterator() = default;
		template<typename It>
		Iterator(const It& it) : cit_(it.cit_), eit_(it.eit_) {}
		Iterator(OIterator bit, OIterator eit) : cit_(bit), eit_(eit) {
			while (cit_ != eit_ && !cit_->valid_)
				++cit_;
		}
		Iterator& operator++() {
			increment();
			return *this;
		}
		Iterator operator++(int) {
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
		friend bool operator==(const Iterator& it1, const Iterator& it2) {
			return it1.cit_ == it2.cit_;
		}
		friend bool operator!=(const Iterator& it1, const Iterator& it2) {
			return !(it1 == it2);
		}
	};

public:
	ListDiGraphWeighted() = default;
	ListDiGraphWeighted(size_t nonodes) : nodes_(nonodes) {}

	bool hasEdge(node_id a, node_id b) const {
		if (a >= nodes_.size() || !nodes_[a].valid_ ||
		        b >= nodes_.size() || !nodes_[b].valid_)
			return false;
		auto& list = edges_map_.find(a)->second;
		auto it = std::find_if(list.begin(), list.end(), [this, b](edge_id c) {
			return this->edges_[c].tg_nid_ == b;
		});
		return it != list.end();
	}
	bool hasEdge(edge_id a) const {
		return a < edges_.size() && edges_[a].valid_;
	}
	Edge& getEdge(edge_id a) {
		assert(a < edges_.size() && edges_[a].valid_);
		return edges_[a];
	}
	const Edge& getEdge(edge_id a) const {
		assert(a < edges_.size() && edges_[a].valid_);
		return edges_[a];
	}
	Node& getNode(node_id a) {
		assert(a < nodes_.size() && nodes_[a].valid_);
		return nodes_[a];
	}
	const Node& getNode(node_id a) const {
		assert(a < nodes_.size() && nodes_[a].valid_);
		return nodes_[a];
	}
	node_id addNode() {
		if (free_nid_.empty()) {
			node_id id = nodes_.size();
			nodes_.emplace_back();
			edges_map_.emplace(id, std::list<edge_id>());
			return id;
		} else {
			node_id id = free_nid_.front();
			free_nid_.pop_front();
			nodes_[id].color_ = Node::Color::WHITE;
			nodes_[id].dist_ = 0;
			nodes_[id].pred_ = static_cast<node_id>(-1);
			nodes_[id].inEdges_ = 0;
			nodes_[id].valid_ = true;
			return id;
		}
	}
	edge_id addEdge(node_id a, node_id b, weight_type c = defaultWeight) {
		assert (a < nodes_.size() && b < nodes_.size() &&
		        nodes_[a].valid_ && nodes_[b].valid_);
		++nodes_[b].inEdges_;
		if (free_eid_.empty()) {
			edge_id id = edges_.size();
			edges_.emplace_back(a, b, c);
			edges_map_.emplace(id, std::list<edge_id>());
			return id;
		} else {
			edge_id id = free_eid_.front();
			free_eid_.pop_front();
			edges_[id].src_nid_ = a;
			edges_[id].tg_nid_ = b;
			edges_[id].weight_ = c;
			edges_[id].valid_ = true;
			return id;
		}
	}
	node_id inNodeDegree(node_id a) const {
		assert(a < nodes_.size() && nodes_[a].valid_);
		return nodes_[a].inEdges_;
	}
	node_id outNodeDegree(node_id a) const {
		assert(a < nodes_.size() && nodes_[a].valid_);
		return edges_map_.find(a)->second.size();
	}
	node_id getSource(edge_id a) const {
		assert(a < edges_.size() && edges_[a].valid_);
		return edges_[a].src_nid_;
	}
	Node& getSourceNode(edge_id a) {
		assert(a < edges_.size() && edges_[a].valid_);
		return nodes_[edges_[a].src_nid_];
	}
	const Node& getSourceNode(edge_id a) const {
		assert(a < edges_.size() && edges_[a].valid_);
		return nodes_[edges_[a].src_nid_];
	}
	node_id getTarget(edge_id a) const {
		assert(a < edges_.size() && edges_[a].valid_);
		return edges_[a].tg_nid_;
	}
	Node& getTargetNode(edge_id a) {
		assert(a < edges_.size() && edges_[a].valid_);
		return nodes_[edges_[a].tg_nid_];
	}
	const Node& getTargetNode(edge_id a) const {
		assert(a < edges_.size() && edges_[a].valid_);
		return nodes_[edges_[a].tg_nid_];
	}
	edge_range& operator[](node_id a) {
		assert(a < nodes_.size() && nodes_[a].valid_);
		return edges_map_[a];
	}
	const edge_range& operator[](node_id a) const {
		assert(a < nodes_.size() && nodes_[a].valid_);
		return edges_map_.find(a)->second;
	}
	weight_type getWeight(edge_id a) const {
		assert(a < edges_.size() && edges_[a].valid_);
		return edges_[a].weight_;
	}
	void setWeight(edge_id a, weight_type w) {
		assert(a < edges_.size() && edges_[a].valid_);
		edges_[a].weight_ = w;
	}
	void removeEdge(edge_id a) {
		assert(a < edges_.size() && edges_[a].valid_);
		--nodes_[edges_[a].tg_nid_].inEdges_;
		edges_[a].valid_ = false;
		free_eid_.push_back(a);
	}
	void removeNode(node_id a) {
		assert(a < nodes_.size() && nodes_[a].valid_);
		for(auto e : edges_map_[a]) {
			--nodes_[edges_[e].tg_nid_].inEdges_;
			removeEdge(e);
		}
		edges_map_[a].clear();
		free_nid_.push_back(a);
		nodes_[a].valid_ = false;
		for (node_id t = a; t < nodes_.size(); ++t)
			for(auto e : edges_map_[t])
				if (edges_[e].tg_nid_ == a)
					removeEdge(e);
	}
	node_id nodeCount() const {
		return nodes_.size() - free_nid_.size();
	}
	edge_id edgeCount() const {
		return edges_.size() - free_eid_.size();
	}
	node_iterator beginNode() {return node_iterator(nodes_.begin(), nodes_.end());}
	const_node_iterator beginNode() const {return const_node_iterator(nodes_.begin(), nodes_.end());}
	const_node_iterator cbeginNode() const {return const_node_iterator(nodes_.begin(), nodes_.end());}
	node_iterator endNode() {return node_iterator(nodes_.end(), nodes_.end());}
	const_node_iterator endNode() const {return const_node_iterator(nodes_.end(), nodes_.end());}
	const_node_iterator cendNode() const {return const_node_iterator(nodes_.end(), nodes_.end());}

	edge_iterator beginEdge() {return edge_iterator(edges_.begin(), edges_.end());}
	const_edge_iterator beginEdge() const {return const_edge_iterator(edges_.begin(), edges_.end());}
	const_edge_iterator cbeginEdge() const {return const_edge_iterator(edges_.begin(), edges_.end());}
	edge_iterator endEdge() {return edge_iterator(edges_.end(), edges_.end());}
	const_edge_iterator endEdge() const {return const_edge_iterator(edges_.end(), edges_.end());}
	const_edge_iterator cendEdge() const {return const_edge_iterator(edges_.end(), edges_.end());}
};

template<>
struct graph_traits<ListDiGraphWeighted> {
	static const bool directedTag = ListDiGraphWeighted::directedTag;
	static const bool weightedTag = ListDiGraphWeighted::weightedTag;
	using node_id = typename ListDiGraphWeighted::node_id;
	using edge_id = typename ListDiGraphWeighted::edge_id;
	using weight_type = typename ListDiGraphWeighted::weight_type;
	using node_iterator = typename ListDiGraphWeighted::node_iterator;
	using edge_iterator = typename ListDiGraphWeighted::edge_iterator;
	using const_node_iterator = typename ListDiGraphWeighted::const_node_iterator;
	using const_edge_iterator = typename ListDiGraphWeighted::const_edge_iterator;
	using edge_range = typename ListDiGraphWeighted::edge_range;
};
}

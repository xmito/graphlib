namespace graphlib {
	namespace concepts {

	    /* Graph represents undirected unweighted graph concept, that has
		 * following methods and aliases*/
	    class Graph {
		private:
			/* Graph should implement struct Node, that is default constructible,
			 * copy constructible, copy assignable*/
			struct Node {
				Node() {}
				Node(const Node&) {}
				Node& operator=(const Node&) {}
				bool operator==(const Node&) const {return true;}
				bool operator!=(const Node&) const {return true;}
			};

			/* Graph should implement Edge struct that is constructible,
			 * copy constructible and copy assignable*/
			struct Edge {
				Edge(node_id, node_id) {}
				Edge(const Edge&) {}
				Edge& operator=(const Edge&) {}
				bool operator==(const Edge&) const {return true;}
				bool operator!=(const Edge&) const {return true;}
			};

			/* Graph struct should provide implementation of iterators and
			 * constant iterators, that can be used to iterate over nodes
			 * and edges. These should fulfill Forward Iterator category.
			 * Their naming can be arbitrary because we also define aliases*/
			struct NodeIterator {/* Forward Iterator */};
			struct ConstNodeIterator {/* Forward Iterator */};
			struct EdgeIterator {/* Forward Iterator */};
			struct ConstEdgeIterator {/* Forward Iterator */};

			/* EdgeRange should be a range, that can be used to
			 * iterate over edges of one node. It should internally implement
			 * begin and end methods. It can be named arbitrarily, because the Graph
			 * concept also defines edge_storage alias */
			struct EdgeRange {};

		public:
			/* It should have node_id and edge_id aliases, that represent
			 * unique identifiers for nodes and edges. These can (not necessarily)
			 * be used to address containers holding nodes and edges.*/
			using node_id = std::size_t;
			using edge_id = std::size_t;
			/* A graph conforming to Graph should have static bools that
			 * identify the kind of graph. For Graph, it should have
			 * directedTag and weightedTag. If weightedTag is false, edges
			 * have implicit weight of one.*/
			static const bool directedTag = false;
			static const bool weightedTag = false;

			using node_iterator = NodeIterator;
			using const_node_iterator = ConstNodeIterator;
			using edge_iterator = EdgeIterator;
			using const_edge_iterator = ConstEdgeIterator;

			using edge_storage = EdgeRange;

			/**** GRAPH METHODS AND CONSTRUCTORS ****
			 ***************************************/
			// Grap should be default constructible
			Graph() {}

			// Graph should be constructible with initial number of nodes
			Graph(size_t nonodes) {}

			/* hasEdge method verifies that Graph contains edge with
			 * source/target node_id or edge_id*/
			bool hasEdge(node_id, node_id) const {return true;}
			bool hasEdge(edge_id) const {return true;}

			/* getNode and getEdge return a reference to Node and Edge structs
			 * according to specified function argument node_id/edge_id*/
			Node& getNode(node_id) {}
			const Node& getNode(node_id) const {}
			Edge& getEdge(edge_id) {}
			const Edge& getEdge(edge_id) const {}

			// getNodes method returns source and target node_ids for edge with edge_id
			std::pair<node_id, node_id> getNodes(edge_id) const {
				return std::make_pair(node_id(), node_id());
			}

			// addNode and addEdge methods add a node and edge respectively
			node_id addNode() {return node_id();}
			edge_id addEdge(node_id, node_id) {return edge_id();}

			// nodeDegree returns a count of incident edges
			node_id nodeDegree(node_id) const {return node_id();}

			/* nodeCount and edgeCount methods return current number of
			 * nodes and edges in Graph */
			node_id nodeCount() const {return node_id();}
			edge_id edgeCount() const {return edge_id();}

			/* removeNode and removeEdge methods are not necessary for Graph
			 * concept, but their inclusion depends on use of certain imple-
			 * mentation of graph*/
			void removeNode(node_id) {}
			void removeEdge(edge_id) {}

			/* operator[] return reference or constant reference to a range,
			 * which can be used to iterate over edges adjacent to node with node_id*/
			edge_storage& operator[](node_id) {}
			const edge_storage& operator[](node_id) const {}

			/* To iterate over all nodes and edges in Graph, implementation class
			 * should provide following methods, that allow to retrieve corresponding
			 * forward iterators */
			node_iterator beginNode() {return node_iterator();}
			const_node_iterator beginNode() const {return const_node_iterator();}
			const_node_iterator cbeginNode() const {return const_node_iterator();}
			node_iterator endNode() {return node_iterator();}
			const_node_iterator endNode() const {return const_node_iterator();}
			const_node_iterator cendNode() const {return const_node_iterator();}
		};
	}
}

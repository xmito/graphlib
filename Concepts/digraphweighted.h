namespace graphlib {
    namespace concepts {

		/* DiGraphWeighted represents directed weighted graph, that has
		 * following methods and aliases*/
		class DiGraphWeighted {
		private:
			/* DiGraphWeighted should implement class Node, that is default constructible,
			 * copy constructible, copy assignable*/
			class Node {
				Node() {}
				Node(const Node&) {}
				Node& operator=(const Node&) {}
				bool operator==(const Node&) const {return true;}
				bool operator!=(const Node&) const {return true;}
			};

			/* DiGraphWeighted should implement Edge class that is default constructible,
			 * copy constructible and copy assignable*/
			class Edge {
				Edge(node_id, node_id, weight_type) {}
				Edge(const Edge&) {}
				Edge& operator=(const Edge&) {}
				bool operator==(const Edge&) const {return true;}
				bool operator!=(const Edge&) const {return true;}
				bool operator<(const Edge&) const {return true;}
			};

			/* DiGraphWeighted struct should provide implementation of iterators and
			 * constant iterators, that can be used to iterate over nodes
			 * and edges. These should fulfill Forward Iterator category.
			 * Their naming can be arbitrary because we also define aliases*/
			struct NodeIterator {/* Forward Iterator */};
			struct ConstNodeIterator {/* Forward Iterator */};
			struct EdgeIterator {/* Forward Iterator */};
			struct ConstEdgeIterator {/* Forward Iterator */};

			/* EdgeRange should be a range, that can be used to iterate over
			 * edges of one node. It should internally implement begin and
			 * end methods. It can be named arbitratily, because DiGraphWeighted
			 * also defines edge_storage alias */
			struct EdgeRange {};

		public:
			/* It should have node_id and edge_id aliases, that represent
			 * unique identifiers for nodes and edges. These can be used to
			 * address containers holding nodes and edges. In addition, it
			 * must have weight_type alias for weighted edges*/
			using node_id = std::size_t;
			using edge_id = std::size_t;
			using weight_type = long int;

			/* A graph conforming to DiGraphWeighted should have static bools that
			 * identify the kind of graph. For DiGraphWeighted, it should have
			 * directedTag and weightedTag. Additionaly, it should have
			 * static unsigned int that represents default weight*/
			static const bool directedTag = true;
			static const bool weightedTag = true;
			static const unsigned int defaultWeight = 1;

			using node_iterator = NodeIterator;
			using const_node_iterator = ConstNodeIterator;
			using edge_iterator = EdgeIterator;
			using const_edge_iterator = ConstEdgeIterator;

			using edge_storage = EdgeRange;

			/**** DIGRAPHWEIGHTED METHODS AND CONSTRUCTORS ****
			 *************************************************/
			// DiGraphWeighted should be default constructible
			DiGraphWeighted() {}

			// DiGraphWeighted should be constructible with initial number of nodes
			DiGraphWeighted(size_t nonodes) {}

			/* hasEdge method verifies that DiGraphWeighted contains edge with
			 * source/target node_id or edge_id*/
			bool hasEdge(node_id, node_id) const {return true;}
			bool hasEdge(edge_id) const {return true;}

			/* getNode and getEdge methods return reference or constant reference
			 * to Node and Edge structs */
			Node& getNode(node_id) {}
			const Node& getNode(node_id) const {}
			Edge& getEdge(edge_id) {}
			const Edge& getEdge(edge_id) const {}

			// addNode and addEdge add node or edge respectively
			node_id addNode() {return node_id();}
			edge_id addEdge(node_id, node_id, weight_type = defaultWeight) {
				return edge_id();
			}

			/* inNodeDegree returns a count of edges, that are directed to
			 * specific node with node_id */
			node_id inNodeDegree(node_id) const {return node_id();}

			/* outNodeDegree returns a count of edges, that start in node
			 * with node_id */
			node_id outNodeDegree(node_id) const {return node_id();}

			/* getSource and getSourceNode methods return node_id or reference
			 * to source node of edge with edge_id */
			node_id getSource(edge_id) const {return node_id();}
			Node& getSourceNode(edge_id) {}
			const Node& getSourceNode(edge_id) const {}

			/* getTarget and getTargetNode methods return node_id or reference
			 * to target node of edge with edge_id */
			node_id getTarget(edge_id) const {return node_id();}
			Node& getTargetNode(edge_id) {}
			const Node& getTargetNode(edge_id) const {}

			// setWeight sets weight of edge with edge_id
			void setWeight(edge_id, weight_type) {return;}

			// getWeight is getter for edge_weight of edge with edge_id
			weight_type getWeight(edge_id) const {return weight_type();}

			/* removeNode and removeEdge methods are not necessary for DiGraphWeighted,
			 * unless it is used with johnson algorithm*/
			void removeNode(node_id) {}
			void removeEdge(edge_id) {}

			// nodeCount and edgeCount return current number of nodes and edges in graph
			node_id nodeCount() const {return node_id();}
			edge_id edgeCount() const {return edge_id();}

			/* operator[] returns a reference or constant reference to a range,
			 * which can be used to iterate over edges adjacent to node with node_id */
			edge_storage& operator[](node_id) {}
			const edge_storage& operator[](node_id) const {}

			/* To iterate over all nodes and edges in DiGraphWeighted, implemented class
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

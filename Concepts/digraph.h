namespace graphlib {
    namespace concepts {

	/* DiGraph represents directed unweighted graph concept, that has
	 * following methods and aliases*/
	    class DiGraph {
		private:
			/* DiGraph should implement class Node, that is default constructible,
			 * copy constructible, copy assignable*/
			class Node {
				Node() {}
				Node(const Node&) {}
				Node& operator=(const Node&) {}
				bool operator==(const Node&) const {return true;}
				bool operator!=(const Node&) const {return true;}
			};

			/* DiGraph should implement Edge class that is constructible,
			 * copy constructible and copy assignable*/
			class Edge {
				Edge(node_id, node_id) {}
				Edge(const Edge&) {}
				Edge& operator=(const Edge&) {}
				bool operator==(const Edge&) const {return true;}
				bool operator!=(const Edge&) const {return true;}
			};

			/* DiGraph struct should provide implementation of iterators and
			 * constant iterators, that can be used to iterate over nodes
			 * and edges. These should fulfill Forward Iterator category.
			 * Their naming can be arbitrary because we also define aliases*/
			struct NodeIterator {/* Forward Iterator */};
			struct ConstNodeIterator {/* Forward Iterator */};
			struct EdgeIterator {/* Forward Iterator */};
			struct ConstEdgeIterator {/* Forward Iterator */};

			/* EdgeRange should be a range, that can be used to iterate over
			 * edges of one node. It should internally implement begin and
			 * end methods It can be named arbitratily, becase the DiGraph
			 * concept defines also alias edge_storage */
			struct EdgeRange {};

		public:
			/* It should have node_id and edge_id aliases, that represent
			 * unique identifiers for nodes and edges. These can be used to
			 * address containers holding nodes and edges.*/
			using node_id = std::size_t;
			using edge_id = std::size_t;

			/* A graph conforming to DiGraph should have static bools that
			 * identify the kind of graph. For DiGraph, it should have
			 * directedTag and weightedTag. If weightedTag is false, edges
			 * have implicit weight of one.*/
			static const bool directedTag = true;
			static const bool weightedTag = false;

			using node_iterator = NodeIterator;
			using const_node_iterator = ConstNodeIterator;
			using edge_iterator = EdgeIterator;
			using const_edge_iterator = ConstEdgeIterator;

			using edge_storage = EdgeRange;

			/**** DIGRAPH METHODS AND CONSTRUCTORS ****
			 *****************************************/
			// DiGrap should be default constructible
			DiGraph() {}

			// DiGraph should be constructible with initial number of nodes
			DiGraph(size_t nonodes) {}

			/* hasEdge method verifies that DiGraph contains edge with
			 * source/target node_id or edge_id*/
			bool hasEdge(node_id, node_id) const {return true;}
			bool hasEdge(edge_id) const {return true;}

			/* getNode and getEdge return a reference or constant reference
			 * to Node and Edge structs */
			Node& getNode(node_id) {}
			const Node& getNode(node_id) const {}
			Edge& getEdge(edge_id) {}
			const Edge& getEdge(edge_id) const {}

			// addNode and addEdge methods add a node and edge respectively
			node_id addNode() {return node_id();}
			edge_id addEdge(node_id, node_id) {return edge_id();}

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
			 * to target node of edge with edge_id*/
			node_id getTarget(edge_id) const {return node_id();}
			Node& getTargetNode(edge_id) {}
			const Node& getTargetNode(edge_id) const {}

			/* removeNode and removeEdge methods are not necessary for DiGraph,
			 * unless it is used with johnson algorithm*/
			void removeNode(node_id) {}
			void removeEdge(edge_id) {}

			/* nodeCount and edgeCount methods return a number of nodes and edges
			 * in DiGraph */
			node_id nodeCount() const {return node_id();}
			edge_id edgeCount() const {return edge_id();}

			/* operator[] returns a reference or constant reference to a range,
			 * which can be used to iterate over edges adjacent to node with node_id*/
			edge_storage& operator[](node_id) {}
			const edge_storage& operator[](node_id) const {}

			/* To iterate over all nodes and edges in DiGraph, implemented class
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

list(APPEND CMAKE_MODULE_PATH
	${CMAKE_CURRENT_LIST_DIR}/bellman_ford
	${CMAKE_CURRENT_LIST_DIR}/dag
	${CMAKE_CURRENT_LIST_DIR}/dijkstra)
include(bellman_ford)
include(dijkstra)
include(dag)

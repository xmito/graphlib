#ifndef FLOYDWARSHALL_H
#define FLOYDWARSHALL_H

#include <type_traits>
#include <climits>

#include "Matrix.h"
#include "NodeData.h"
#include "graph_traits.h"

namespace graphlib {

/* Assumes directed weighted graphs with no negative weight cycles*/
template <typename Graph,
          typename = std::enable_if_t<Graph::directedTag &&
                                      Graph::weightedTag && Graph::pathTag>>
Matrix<typename graph_traits<Graph>::distance_type>
floydWarshall(const Graph &graph) {

    using node_handle = typename graph_traits<Graph>::node_handle;
    using distance_type = typename graph_traits<Graph>::distance_type;

    Matrix<distance_type> resMatrix(graph.nodeCount(), graph.nodeCount(),
                                    std::numeric_limits<distance_type>::max());
    for (auto &snh : graph.nodes()) {
        for (auto &eh : graph[snh]) {
            node_handle tnh = graph.getTarget(eh);
            resMatrix[snh.getId()][tnh.getId()] = graph.getWeight(eh);
        }
        resMatrix[snh.getId()][snh.getId()] = 0;
    }

    for (auto &nh : graph.nodes()) {
        auto k = nh.getId();

        for (auto &nh1 : graph.nodes()) {
            auto i = nh1.getId();

            for (auto &nh2 : graph.nodes()) {
                auto j = nh2.getId();

                if (resMatrix[i][k] !=
                        std::numeric_limits<distance_type>::max() &&
                    resMatrix[k][j] !=
                        std::numeric_limits<distance_type>::max()) {
                    if (resMatrix[i][j] > resMatrix[i][k] + resMatrix[k][j]) {
                        resMatrix[i][j] = resMatrix[i][k] + resMatrix[k][j];
                    }
                }
            }
        }
    }

    return resMatrix;
}

} // namespace graphlib

#endif

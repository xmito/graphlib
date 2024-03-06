#ifndef GRAPHVIZ_EXPORT_H
#define GRAPHVIZ_EXPORT_H
#include "graph_traits.h"
#include <exception>
#include <set>
#include <sstream>
#include <string>
#include <utility>

namespace graphlib {

class path_error : public std::runtime_error {
  public:
    explicit path_error(const std::string &what) : std::runtime_error(what) {}
};

inline void normalizePath(std::string &path) {
    if (!path.empty() && (path[0] != '/' || path.compare(0, 2, "./")))
        path.insert(0, "./");
    else if (path.empty())
        throw path_error("Path string is empty!");
}

template <typename Graph>
void exportGraph(const Graph &graph, std::ostream &out) {
    std::string pre = Graph::directedTag ? "digraph" : "graph";
    std::string indent("    ");
    out << pre << " {\n";
    for (auto &eh : graph.edges()) {
        if constexpr (Graph::directedTag) {
            out << indent << graph.getSource(eh).getId();
            out << " -> ";
            out << graph.getTarget(eh).getId();
        } else {
            auto [fst, snd] = graph.getBoth(eh);
            out << indent << fst.getId();
            out << " -- ";
            out << snd.getId();
        }
        if (Graph::weightedTag)
            out << "[label=" << graph.getWeight(eh) << "]";
        out << ";\n";
    }
    out << "}";
}

template <typename Graph>
std::string exportGraph(const Graph &graph) {
    std::stringstream ss;
    exportGraph(graph, ss);
    return ss.str();
}

template <typename Graph>
int exportGraph(const Graph &graph, std::string path) {
    try {
        normalizePath(path);
    } catch (const path_error &ex) {
        std::cerr << ex.what() << std::endl;
        return -1;
    }
    std::ofstream ofile(path);
    if (ofile.is_open())
        exportGraph(graph, ofile);
    else {
        std::cerr << "Failed to open " << path << std::endl;
        return -1;
    }
    ofile.close();
    return 0;
}

template <typename Graph>
void exportShortestPath(const Graph &graph,
                        typename graph_traits<Graph>::node_handle target,
                        std::ostream &out) {
    using node_handle = typename graph_traits<Graph>::node_handle;
    std::string pre = Graph::directedTag ? "digraph" : "graph";
    std::string indent("    ");
    std::set<std::pair<node_handle, node_handle>> shedges;
    while (graph.getNodePred(target) != node_handle()) {
        node_handle pred = graph.getNodePred(target);
        shedges.insert(std::make_pair(pred, target));
        target = pred;
    }
    out << pre << " {\n";
    for (auto &eh : graph.edges()) {
        if constexpr (Graph::directedTag) {
            node_handle src = graph.getSource(eh);
            node_handle tg = graph.getTarget(eh);
            out << indent << src.getId() << " -> " << tg.getId();
            if (shedges.find(std::make_pair(src, tg)) != shedges.end())
                out << "[color=red,pendwidth=3.0]";
        } else {
            auto [fst, snd] = graph.getBoth(eh);
            out << indent << fst.getId() << " -- " << snd.getId();
            if (shedges.find(std::make_pair(fst, snd)) != shedges.end() ||
                shedges.find(std::make_pair(snd, fst)) != shedges.end())
                out << "[color=red,pendwidth=3.0]";
        }
        if (graph.weightedTag)
            out << "[label=" << graph.getWeight(eh) << "]";
        out << ";\n";
    }
    out << "}";
}

template <typename Graph>
std::string
exportShortestPath(const Graph &graph,
                   typename graph_traits<Graph>::node_handle target) {
    std::stringstream ss;
    exportShortestPath(graph, target, ss);
    return ss.str();
}

template <typename Graph>
int exportShortestPath(const Graph &graph,
                       typename graph_traits<Graph>::node_handle target,
                       std::string path) {
    try {
        normalizePath(path);
    } catch (const path_error &ex) {
        std::cerr << ex.what() << std::endl;
        return -1;
    }
    std::ofstream ofile(path);
    if (ofile.is_open())
        exportShortestPath(graph, target, ofile);
    else {
        std::cerr << "Failed to open" << path << std::endl;
        return -1;
    }
    ofile.close();
    return 0;
}

} // namespace graphlib

#endif

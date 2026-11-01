// Directed road graph stored in Compressed Sparse Row (CSR) form.
//
// CSR keeps every edge in three flat arrays instead of one heap-allocated
// list per node. Node u's outgoing edges are targets[offsets[u] .. offsets[u+1]).
// Flat arrays are cache-friendly and cost ~8 bytes per edge, which is what
// lets a multi-million-node state road network fit comfortably in memory.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace route {

using NodeId = uint32_t;
constexpr NodeId kNoNode = UINT32_MAX;

struct Edge {
  NodeId from;
  NodeId to;
  float meters;
};

struct Graph {
  // Node coordinates (degrees).
  std::vector<double> lat, lon;

  // Forward adjacency: outgoing edges.
  std::vector<uint32_t> offsets;  // size n + 1
  std::vector<NodeId> targets;    // size m
  std::vector<float> weights;     // size m, edge length in meters

  // Reverse adjacency: incoming edges. Needed to search backward from the
  // goal (bidirectional search, landmark preprocessing) on one-way streets.
  std::vector<uint32_t> roffsets;
  std::vector<NodeId> rtargets;
  std::vector<float> rweights;

  // 1 if the node is in the largest strongly connected component, i.e. the
  // part of the map where every node can reach every other node.
  std::vector<uint8_t> in_main_scc;
  size_t main_scc_size = 0;

  size_t num_nodes() const { return lat.size(); }
  size_t num_edges() const { return targets.size(); }

  // Builds roffsets/rtargets/rweights from the forward arrays.
  void build_reverse();
  // Marks the largest strongly connected component (iterative Kosaraju).
  void compute_main_scc();
  // Nearest node to a coordinate, restricted to the main SCC.
  NodeId nearest_node(double lat, double lon) const;
  // Shortest edge u -> v, or a negative value if there is none.
  double edge_weight(NodeId u, NodeId v) const;
};

// Builds a CSR graph from coordinates and an unordered edge list.
Graph build_graph(std::vector<double> lat, std::vector<double> lon,
                  const std::vector<Edge>& edges);

// Binary file format written by tools/*.py (see README).
Graph load_graph(const std::string& path);
void save_graph(const Graph& g, const std::string& path);

// Great-circle distance in meters.
double haversine_m(double lat1, double lon1, double lat2, double lon2);

}  // namespace route

// Shortest-path search: BFS, UCS, bidirectional UCS, A*, and ALT.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "graph.h"

namespace route {

enum class Algorithm { kBFS, kUCS, kBiUCS, kAStar, kALT };

const std::vector<Algorithm>& all_algorithms();
std::string algorithm_name(Algorithm a);  // "bfs", "ucs", "biucs", "astar", "alt"
bool parse_algorithm(const std::string& s, Algorithm* out);

struct SearchResult {
  bool found = false;
  double meters = 0;            // total route length
  std::vector<NodeId> path;     // source ... target
  uint64_t expanded = 0;        // nodes taken off the frontier and processed
  double millis = 0;            // wall-clock search time
  std::vector<NodeId> explored; // expanded nodes, in order (capped, for drawing)
};

// ALT preprocessing: exact shortest distances to and from a few landmark
// nodes. By the triangle inequality, for any landmark L:
//   d(v, t) >= d(L, t) - d(L, v)   and   d(v, t) >= d(v, L) - d(t, L)
// so the largest of these differences is a lower bound on the remaining
// distance: an admissible A* heuristic, usually far tighter than a straight
// line because it accounts for the actual road network.
class Landmarks {
 public:
  // Picks k landmarks with "farthest" selection: each new landmark is the
  // node farthest (by road) from the ones already chosen, so they spread out
  // toward the edges of the map, where they give the best bounds.
  void build(const Graph& g, int k, uint32_t seed);
  double lower_bound(NodeId v, NodeId t) const;
  int count() const { return static_cast<int>(nodes_.size()); }
  const std::vector<NodeId>& nodes() const { return nodes_; }
  size_t memory_bytes() const { return (from_.size() + to_.size()) * sizeof(float); }

 private:
  size_t n_ = 0;
  std::vector<NodeId> nodes_;
  std::vector<float> from_;  // from_[i * n + v] = d(L_i, v)
  std::vector<float> to_;    // to_[i * n + v]   = d(v, L_i)
};

// Exact distances from one source to every node (plain Dijkstra over a CSR
// adjacency). Unreachable nodes get +infinity.
std::vector<double> distances_from(const std::vector<uint32_t>& offsets,
                                   const std::vector<NodeId>& targets,
                                   const std::vector<float>& weights,
                                   NodeId source);

// Runs queries against one graph. Holds per-node scratch arrays that are
// reused across queries; see the "stamp" note in search.cpp.
class Router {
 public:
  explicit Router(const Graph& g);
  void set_landmarks(const Landmarks* lm) { landmarks_ = lm; }
  bool has_landmarks() const { return landmarks_ && landmarks_->count() > 0; }

  // record_explored: how many expanded nodes to keep for visualization.
  SearchResult route(Algorithm a, NodeId s, NodeId t, size_t record_explored = 0);

 private:
  struct Side {
    std::vector<double> dist;
    std::vector<NodeId> parent;
    std::vector<uint32_t> seen;     // == stamp: dist/parent valid this query
    std::vector<uint32_t> settled;  // == stamp: node expanded this query
  };

  void bfs(NodeId s, NodeId t, SearchResult& r, size_t cap);
  void best_first(NodeId s, NodeId t, bool use_heuristic, bool use_landmarks,
                  SearchResult& r, size_t cap);
  void bidirectional(NodeId s, NodeId t, SearchResult& r, size_t cap);

  void next_stamp();
  bool seen(const Side& sd, NodeId v) const { return sd.seen[v] == stamp_; }
  double dist(const Side& sd, NodeId v) const;
  void reach(Side& sd, NodeId v, double d, NodeId parent);
  double heuristic(NodeId v, NodeId t, bool use_landmarks) const;
  void finish_path(SearchResult& r, NodeId s, NodeId t);

  const Graph& g_;
  const Landmarks* landmarks_ = nullptr;
  Side fwd_, bwd_;
  uint32_t stamp_ = 0;
};

}  // namespace route

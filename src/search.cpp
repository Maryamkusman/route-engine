#include "search.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <random>
#include <stdexcept>

namespace route {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();

// Heuristics are scaled down by a hair. Edge weights are stored as 32-bit
// floats, so rounding could otherwise make a perfect lower bound overshoot the
// true distance by a few millimeters, which would technically break A*'s
// optimality guarantee. Shrinking the estimate by 0.001% costs nothing in
// practice and keeps the heuristic admissible.
constexpr double kHeuristicSlack = 0.99999;

struct QueueEntry {
  double key;  // priority: g for UCS, g + h for A*/ALT
  double g;    // cost from the source when this entry was pushed
  NodeId v;
  bool operator>(const QueueEntry& o) const { return key > o.key; }
};
using MinQueue =
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>>;

double elapsed_ms(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
      .count();
}
}  // namespace

const std::vector<Algorithm>& all_algorithms() {
  static const std::vector<Algorithm> kAll = {Algorithm::kBFS, Algorithm::kUCS,
                                              Algorithm::kBiUCS, Algorithm::kAStar,
                                              Algorithm::kALT};
  return kAll;
}

std::string algorithm_name(Algorithm a) {
  switch (a) {
    case Algorithm::kBFS: return "bfs";
    case Algorithm::kUCS: return "ucs";
    case Algorithm::kBiUCS: return "biucs";
    case Algorithm::kAStar: return "astar";
    case Algorithm::kALT: return "alt";
  }
  return "?";
}

bool parse_algorithm(const std::string& s, Algorithm* out) {
  for (Algorithm a : all_algorithms())
    if (algorithm_name(a) == s) {
      *out = a;
      return true;
    }
  return false;
}

// ---------------------------------------------------------------------------
// One-to-all Dijkstra (used for landmark preprocessing)

std::vector<double> distances_from(const std::vector<uint32_t>& offsets,
                                   const std::vector<NodeId>& targets,
                                   const std::vector<float>& weights,
                                   NodeId source) {
  std::vector<double> dist(offsets.size() - 1, kInf);
  MinQueue pq;
  dist[source] = 0;
  pq.push({0, 0, source});
  while (!pq.empty()) {
    QueueEntry e = pq.top();
    pq.pop();
    if (e.g > dist[e.v]) continue;  // stale entry
    for (uint32_t i = offsets[e.v]; i < offsets[e.v + 1]; ++i) {
      double nd = e.g + weights[i];
      if (nd < dist[targets[i]]) {
        dist[targets[i]] = nd;
        pq.push({nd, nd, targets[i]});
      }
    }
  }
  return dist;
}

// ---------------------------------------------------------------------------
// Landmarks

void Landmarks::build(const Graph& g, int k, uint32_t seed) {
  n_ = g.num_nodes();
  nodes_.clear();
  from_.assign(static_cast<size_t>(k) * n_, 0.f);
  to_.assign(static_cast<size_t>(k) * n_, 0.f);
  if (k <= 0 || g.main_scc_size == 0) return;

  // Start from a random node in the main component; the first landmark is
  // the node farthest from it.
  std::mt19937 rng(seed);
  std::uniform_int_distribution<NodeId> pick(0, static_cast<NodeId>(n_ - 1));
  NodeId start;
  do start = pick(rng);
  while (!g.in_main_scc[start]);

  std::vector<double> min_dist =
      distances_from(g.offsets, g.targets, g.weights, start);
  for (int i = 0; i < k; ++i) {
    // Farthest-node selection: the node whose distance to the nearest
    // already-chosen landmark (or the start node) is largest.
    NodeId best = kNoNode;
    double best_d = -1;
    for (NodeId v = 0; v < n_; ++v)
      if (g.in_main_scc[v] && std::isfinite(min_dist[v]) && min_dist[v] > best_d) {
        best_d = min_dist[v];
        best = v;
      }
    // Stop early on tiny maps where every node is already a landmark.
    if (best == kNoNode || (i > 0 && best_d <= 0)) break;
    nodes_.push_back(best);

    std::vector<double> from = distances_from(g.offsets, g.targets, g.weights, best);
    std::vector<double> to = distances_from(g.roffsets, g.rtargets, g.rweights, best);
    for (NodeId v = 0; v < n_; ++v) {
      from_[i * n_ + v] = static_cast<float>(from[v]);
      to_[i * n_ + v] = static_cast<float>(to[v]);
      if (i == 0) min_dist[v] = from[v];
      else min_dist[v] = std::min(min_dist[v], from[v]);
    }
  }
  from_.resize(nodes_.size() * n_);
  to_.resize(nodes_.size() * n_);
}

double Landmarks::lower_bound(NodeId v, NodeId t) const {
  double best = 0;
  for (size_t i = 0; i < nodes_.size(); ++i) {
    const float* from = &from_[i * n_];
    const float* to = &to_[i * n_];
    double a = static_cast<double>(from[t]) - from[v];  // d(L,t) - d(L,v)
    double b = static_cast<double>(to[v]) - to[t];      // d(v,L) - d(t,L)
    if (std::isfinite(a)) best = std::max(best, a);
    if (std::isfinite(b)) best = std::max(best, b);
  }
  return best;
}

// ---------------------------------------------------------------------------
// Router

// The "stamp" trick: a query must start with every node's distance at
// infinity. Clearing arrays of millions of entries before each query would
// cost more than the search itself on a short route. Instead each query gets
// a new stamp number, and a node's entry only counts if its stamp matches.
// Resetting is then O(1).

Router::Router(const Graph& g) : g_(g) {
  size_t n = g.num_nodes();
  for (Side* sd : {&fwd_, &bwd_}) {
    sd->dist.assign(n, kInf);
    sd->parent.assign(n, kNoNode);
    sd->seen.assign(n, 0);
    sd->settled.assign(n, 0);
  }
}

void Router::next_stamp() {
  if (++stamp_ == 0) {  // wrapped after ~4 billion queries
    for (Side* sd : {&fwd_, &bwd_}) {
      std::fill(sd->seen.begin(), sd->seen.end(), 0);
      std::fill(sd->settled.begin(), sd->settled.end(), 0);
    }
    stamp_ = 1;
  }
}

double Router::dist(const Side& sd, NodeId v) const {
  return sd.seen[v] == stamp_ ? sd.dist[v] : kInf;
}

void Router::reach(Side& sd, NodeId v, double d, NodeId parent) {
  sd.seen[v] = stamp_;
  sd.dist[v] = d;
  sd.parent[v] = parent;
}

double Router::heuristic(NodeId v, NodeId t, bool use_landmarks) const {
  // Straight-line distance never exceeds driving distance, because every
  // edge is at least as long as the straight line between its endpoints.
  double h = haversine_m(g_.lat[v], g_.lon[v], g_.lat[t], g_.lon[t]);
  if (use_landmarks) h = std::max(h, landmarks_->lower_bound(v, t));
  return h * kHeuristicSlack;
}

SearchResult Router::route(Algorithm a, NodeId s, NodeId t, size_t record_explored) {
  if (s >= g_.num_nodes() || t >= g_.num_nodes())
    throw std::out_of_range("node id out of range");
  if (a == Algorithm::kALT && !has_landmarks())
    throw std::logic_error("ALT needs landmarks; build them first");

  SearchResult r;
  auto start = std::chrono::steady_clock::now();
  switch (a) {
    case Algorithm::kBFS: bfs(s, t, r, record_explored); break;
    case Algorithm::kUCS: best_first(s, t, false, false, r, record_explored); break;
    case Algorithm::kAStar: best_first(s, t, true, false, r, record_explored); break;
    case Algorithm::kALT: best_first(s, t, true, true, r, record_explored); break;
    case Algorithm::kBiUCS: bidirectional(s, t, r, record_explored); break;
  }
  r.millis = elapsed_ms(start);

  // Report the route's length by summing its actual edges, so every
  // algorithm's answer is measured the same way.
  if (r.found) {
    r.meters = 0;
    for (size_t i = 1; i < r.path.size(); ++i)
      r.meters += g_.edge_weight(r.path[i - 1], r.path[i]);
  }
  return r;
}

void Router::finish_path(SearchResult& r, NodeId s, NodeId t) {
  r.found = true;
  for (NodeId v = t; v != kNoNode; v = fwd_.parent[v]) r.path.push_back(v);
  std::reverse(r.path.begin(), r.path.end());
  (void)s;
}

// BFS: explores in rings of "number of road segments from the start". The
// first time it sees the target, it has found the route with the fewest
// segments, which is NOT the shortest route when segments differ in length.
void Router::bfs(NodeId s, NodeId t, SearchResult& r, size_t cap) {
  next_stamp();
  std::vector<NodeId> queue{s};
  reach(fwd_, s, 0, kNoNode);
  bool done = (s == t);
  for (size_t head = 0; head < queue.size() && !done; ++head) {
    NodeId u = queue[head];
    r.expanded++;
    if (r.explored.size() < cap) r.explored.push_back(u);
    for (uint32_t i = g_.offsets[u]; i < g_.offsets[u + 1]; ++i) {
      NodeId v = g_.targets[i];
      if (seen(fwd_, v)) continue;
      reach(fwd_, v, fwd_.dist[u] + 1, u);
      if (v == t) {  // early exit: hop count is final when first seen
        done = true;
        break;
      }
      queue.push_back(v);
    }
  }
  if (done) finish_path(r, s, t);
}

// UCS, A* and ALT share one loop; they differ only in the priority key.
//   UCS:  key = g(v)                 (expand the closest node first)
//   A*:   key = g(v) + straight-line distance to the target
//   ALT:  key = g(v) + max(straight line, landmark lower bound)
void Router::best_first(NodeId s, NodeId t, bool use_heuristic, bool use_landmarks,
                        SearchResult& r, size_t cap) {
  next_stamp();
  MinQueue pq;
  reach(fwd_, s, 0, kNoNode);
  pq.push({use_heuristic ? heuristic(s, t, use_landmarks) : 0, 0, s});

  while (!pq.empty()) {
    QueueEntry e = pq.top();
    pq.pop();
    NodeId u = e.v;
    // Lazy deletion: instead of a decrease-key operation, we push a new
    // entry whenever we find a shorter path and skip outdated ones here.
    if (e.g > fwd_.dist[u]) continue;
    fwd_.settled[u] = stamp_;
    r.expanded++;
    if (r.explored.size() < cap) r.explored.push_back(u);
    if (u == t) {
      finish_path(r, s, t);
      return;
    }
    for (uint32_t i = g_.offsets[u]; i < g_.offsets[u + 1]; ++i) {
      NodeId v = g_.targets[i];
      double nd = e.g + g_.weights[i];
      if (nd < dist(fwd_, v)) {
        reach(fwd_, v, nd, u);
        double key = use_heuristic ? nd + heuristic(v, t, use_landmarks) : nd;
        pq.push({key, nd, v});
      }
    }
  }
}

// Bidirectional UCS: one search grows forward from s, one grows backward
// from t over the reversed edges, always advancing whichever frontier is
// closer. mu tracks the best complete route seen where the two searches
// touch. The subtle part is stopping: the first meeting point is not always
// on the best route, so we keep going until
//   (closest forward frontier) + (closest backward frontier) >= mu,
// at which point no undiscovered route can beat mu.
void Router::bidirectional(NodeId s, NodeId t, SearchResult& r, size_t cap) {
  next_stamp();
  if (s == t) {
    reach(fwd_, s, 0, kNoNode);
    finish_path(r, s, t);
    return;
  }
  MinQueue qf, qb;
  reach(fwd_, s, 0, kNoNode);
  reach(bwd_, t, 0, kNoNode);
  qf.push({0, 0, s});
  qb.push({0, 0, t});
  double mu = kInf;
  NodeId meet = kNoNode;

  while (!qf.empty() && !qb.empty()) {
    if (qf.top().key + qb.top().key >= mu) break;
    bool forward = qf.top().key <= qb.top().key;
    Side& me = forward ? fwd_ : bwd_;
    Side& other = forward ? bwd_ : fwd_;
    MinQueue& q = forward ? qf : qb;
    const auto& offs = forward ? g_.offsets : g_.roffsets;
    const auto& tgts = forward ? g_.targets : g_.rtargets;
    const auto& wts = forward ? g_.weights : g_.rweights;

    QueueEntry e = q.top();
    q.pop();
    NodeId u = e.v;
    if (e.g > me.dist[u]) continue;
    me.settled[u] = stamp_;
    r.expanded++;
    if (r.explored.size() < cap) r.explored.push_back(u);

    for (uint32_t i = offs[u]; i < offs[u + 1]; ++i) {
      NodeId v = tgts[i];
      double nd = e.g + wts[i];
      if (nd < dist(me, v)) {
        reach(me, v, nd, u);
        q.push({nd, nd, v});
        double through = nd + dist(other, v);
        if (through < mu) {
          mu = through;
          meet = v;
        }
      }
    }
  }
  if (meet == kNoNode) return;

  // Stitch: s ... meet from forward parents, meet ... t from backward parents.
  r.found = true;
  for (NodeId v = meet; v != kNoNode; v = fwd_.parent[v]) r.path.push_back(v);
  std::reverse(r.path.begin(), r.path.end());
  for (NodeId v = bwd_.parent[meet]; v != kNoNode; v = bwd_.parent[v])
    r.path.push_back(v);
}

}  // namespace route

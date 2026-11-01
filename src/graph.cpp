#include "graph.h"

#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace route {

namespace {
constexpr char kMagic[4] = {'R', 'T', 'E', 'G'};
constexpr uint32_t kVersion = 1;
constexpr double kEarthRadiusM = 6371008.8;
constexpr double kDegToRad = M_PI / 180.0;

// Counting sort of edges by one endpoint into CSR arrays.
void to_csr(size_t n, const std::vector<Edge>& edges, bool by_from,
            std::vector<uint32_t>& offsets, std::vector<NodeId>& targets,
            std::vector<float>& weights) {
  offsets.assign(n + 1, 0);
  for (const Edge& e : edges) offsets[(by_from ? e.from : e.to) + 1]++;
  for (size_t i = 0; i < n; ++i) offsets[i + 1] += offsets[i];
  targets.resize(edges.size());
  weights.resize(edges.size());
  std::vector<uint32_t> cursor(offsets.begin(), offsets.end() - 1);
  for (const Edge& e : edges) {
    NodeId key = by_from ? e.from : e.to;
    uint32_t pos = cursor[key]++;
    targets[pos] = by_from ? e.to : e.from;
    weights[pos] = e.meters;
  }
}
}  // namespace

double haversine_m(double lat1, double lon1, double lat2, double lon2) {
  double p1 = lat1 * kDegToRad, p2 = lat2 * kDegToRad;
  double dp = p2 - p1, dl = (lon2 - lon1) * kDegToRad;
  double a = std::sin(dp / 2) * std::sin(dp / 2) +
             std::cos(p1) * std::cos(p2) * std::sin(dl / 2) * std::sin(dl / 2);
  return 2 * kEarthRadiusM * std::asin(std::min(1.0, std::sqrt(a)));
}

Graph build_graph(std::vector<double> lat, std::vector<double> lon,
                  const std::vector<Edge>& edges) {
  Graph g;
  g.lat = std::move(lat);
  g.lon = std::move(lon);
  to_csr(g.num_nodes(), edges, true, g.offsets, g.targets, g.weights);
  g.build_reverse();
  g.compute_main_scc();
  return g;
}

void Graph::build_reverse() {
  size_t n = num_nodes();
  std::vector<Edge> edges;
  edges.reserve(num_edges());
  for (NodeId u = 0; u < n; ++u)
    for (uint32_t i = offsets[u]; i < offsets[u + 1]; ++i)
      edges.push_back({u, targets[i], weights[i]});
  to_csr(n, edges, false, roffsets, rtargets, rweights);
}

void Graph::compute_main_scc() {
  // Kosaraju's algorithm, written iteratively so a long chain of road
  // segments can't overflow the call stack.
  size_t n = num_nodes();
  std::vector<uint8_t> seen(n, 0);
  std::vector<NodeId> order;
  order.reserve(n);
  std::vector<std::pair<NodeId, uint32_t>> stack;  // (node, next edge index)

  // Pass 1: record nodes in order of DFS finish time on the forward graph.
  for (NodeId root = 0; root < n; ++root) {
    if (seen[root]) continue;
    seen[root] = 1;
    stack.push_back({root, offsets[root]});
    while (!stack.empty()) {
      auto& [u, i] = stack.back();
      if (i < offsets[u + 1]) {
        NodeId v = targets[i++];
        if (!seen[v]) {
          seen[v] = 1;
          stack.push_back({v, offsets[v]});
        }
      } else {
        order.push_back(u);
        stack.pop_back();
      }
    }
  }

  // Pass 2: flood fill the reverse graph in reverse finish order. Each fill
  // is exactly one strongly connected component.
  std::vector<uint32_t> comp(n, UINT32_MAX);
  std::vector<size_t> sizes;
  std::vector<NodeId> todo;
  for (auto it = order.rbegin(); it != order.rend(); ++it) {
    if (comp[*it] != UINT32_MAX) continue;
    uint32_t id = static_cast<uint32_t>(sizes.size());
    sizes.push_back(0);
    comp[*it] = id;
    todo.push_back(*it);
    while (!todo.empty()) {
      NodeId u = todo.back();
      todo.pop_back();
      sizes[id]++;
      for (uint32_t i = roffsets[u]; i < roffsets[u + 1]; ++i) {
        NodeId v = rtargets[i];
        if (comp[v] == UINT32_MAX) {
          comp[v] = id;
          todo.push_back(v);
        }
      }
    }
  }

  uint32_t best = 0;
  for (uint32_t c = 1; c < sizes.size(); ++c)
    if (sizes[c] > sizes[best]) best = c;
  in_main_scc.assign(n, 0);
  main_scc_size = sizes.empty() ? 0 : sizes[best];
  for (size_t u = 0; u < n; ++u) in_main_scc[u] = comp[u] == best;
}

NodeId Graph::nearest_node(double qlat, double qlon) const {
  // Linear scan with an equirectangular approximation: about 2 ms per
  // million nodes, plenty fast for interactive clicks on a map.
  double cos_lat = std::cos(qlat * kDegToRad);
  NodeId best = kNoNode;
  double best_d = std::numeric_limits<double>::infinity();
  for (NodeId u = 0; u < num_nodes(); ++u) {
    if (!in_main_scc[u]) continue;
    double dy = lat[u] - qlat, dx = (lon[u] - qlon) * cos_lat;
    double d = dx * dx + dy * dy;
    if (d < best_d) {
      best_d = d;
      best = u;
    }
  }
  return best;
}

double Graph::edge_weight(NodeId u, NodeId v) const {
  double best = -1;
  for (uint32_t i = offsets[u]; i < offsets[u + 1]; ++i)
    if (targets[i] == v && (best < 0 || weights[i] < best)) best = weights[i];
  return best;
}

// File layout (little-endian):
//   char[4] "RTEG", uint32 version, uint64 n, uint64 m,
//   double lat[n], double lon[n], uint32 offsets[n+1],
//   uint32 targets[m], float weights[m]
Graph load_graph(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot open " + path);
  char magic[4];
  uint32_t version;
  uint64_t n, m;
  in.read(magic, 4);
  in.read(reinterpret_cast<char*>(&version), sizeof version);
  in.read(reinterpret_cast<char*>(&n), sizeof n);
  in.read(reinterpret_cast<char*>(&m), sizeof m);
  if (!in || std::memcmp(magic, kMagic, 4) != 0 || version != kVersion)
    throw std::runtime_error(path + " is not a route-engine graph file");

  Graph g;
  auto read_vec = [&](auto& v, size_t count) {
    v.resize(count);
    in.read(reinterpret_cast<char*>(v.data()), count * sizeof(v[0]));
  };
  read_vec(g.lat, n);
  read_vec(g.lon, n);
  read_vec(g.offsets, n + 1);
  read_vec(g.targets, m);
  read_vec(g.weights, m);
  if (!in) throw std::runtime_error(path + " is truncated");
  if (g.offsets[n] != m) throw std::runtime_error(path + " has bad offsets");
  g.build_reverse();
  g.compute_main_scc();
  return g;
}

void save_graph(const Graph& g, const std::string& path) {
  std::ofstream out(path, std::ios::binary);
  if (!out) throw std::runtime_error("cannot write " + path);
  uint64_t n = g.num_nodes(), m = g.num_edges();
  out.write(kMagic, 4);
  out.write(reinterpret_cast<const char*>(&kVersion), sizeof kVersion);
  out.write(reinterpret_cast<const char*>(&n), sizeof n);
  out.write(reinterpret_cast<const char*>(&m), sizeof m);
  auto write_vec = [&](const auto& v) {
    out.write(reinterpret_cast<const char*>(v.data()), v.size() * sizeof(v[0]));
  };
  write_vec(g.lat);
  write_vec(g.lon);
  write_vec(g.offsets);
  write_vec(g.targets);
  write_vec(g.weights);
}

}  // namespace route

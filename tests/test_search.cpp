// Correctness tests. The core idea: UCS (Dijkstra) is simple enough to trust,
// so every faster algorithm must return a route of exactly the same length on
// thousands of random queries. Plus small hand-built graphs for edge cases.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "../src/graph.h"
#include "../src/search.h"

using namespace route;

static int g_failures = 0;
#define CHECK(cond, ...)                                              \
  do {                                                                \
    if (!(cond)) {                                                    \
      std::fprintf(stderr, "FAIL %s:%d: %s  ", __FILE__, __LINE__, #cond); \
      std::fprintf(stderr, __VA_ARGS__);                              \
      std::fprintf(stderr, "\n");                                     \
      ++g_failures;                                                   \
    }                                                                 \
  } while (0)

static bool near(double a, double b) { return std::fabs(a - b) <= 1e-6 * b + 0.01; }

// A path is valid if it starts at s, ends at t, and every step is a real edge.
static bool valid_path(const Graph& g, const SearchResult& r, NodeId s, NodeId t) {
  if (r.path.empty() || r.path.front() != s || r.path.back() != t) return false;
  for (size_t i = 1; i < r.path.size(); ++i)
    if (g.edge_weight(r.path[i - 1], r.path[i]) < 0) return false;
  return true;
}

// A road-like grid: jittered intersections ~120 m apart, curvy edges (longer
// than the straight line), some one-way streets, and some missing streets so
// that parts of the map are disconnected.
static Graph random_city(int rows, int cols, uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u01(0, 1);
  std::vector<double> lat, lon;
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) {
      lat.push_back(42.28 + r * 0.0011 + (u01(rng) - 0.5) * 0.0004);
      lon.push_back(-83.74 + c * 0.0015 + (u01(rng) - 0.5) * 0.0005);
    }
  std::vector<Edge> edges;
  auto add = [&](int a, int b) {
    if (u01(rng) < 0.12) return;  // missing street
    float len = static_cast<float>(haversine_m(lat[a], lon[a], lat[b], lon[b]) *
                                   (1.0 + 0.5 * u01(rng)));
    double kind = u01(rng);
    if (kind < 0.15) edges.push_back({(NodeId)a, (NodeId)b, len});       // one-way
    else if (kind < 0.30) edges.push_back({(NodeId)b, (NodeId)a, len});  // one-way
    else {
      edges.push_back({(NodeId)a, (NodeId)b, len});
      edges.push_back({(NodeId)b, (NodeId)a, len});
    }
  };
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) {
      int id = r * cols + c;
      if (c + 1 < cols) add(id, id + 1);
      if (r + 1 < rows) add(id, id + cols);
    }
  return build_graph(lat, lon, edges);
}

static void test_random_queries_match_ucs() {
  Graph g = random_city(60, 60, 7);
  Landmarks lm;
  lm.build(g, 6, 3);
  Router router(g);
  router.set_landmarks(&lm);
  std::mt19937 rng(11);
  std::uniform_int_distribution<NodeId> pick(0, g.num_nodes() - 1);
  int found = 0, missing = 0;
  for (int q = 0; q < 2000; ++q) {
    // Any two nodes, including unreachable pairs and s == t.
    NodeId s = pick(rng), t = q % 100 == 0 ? s : pick(rng);
    SearchResult base = router.route(Algorithm::kUCS, s, t);
    base.found ? ++found : ++missing;
    if (base.found) CHECK(valid_path(g, base, s, t), "ucs path %u->%u", s, t);
    for (Algorithm a : all_algorithms()) {
      SearchResult r = router.route(a, s, t);
      std::string name = algorithm_name(a);
      CHECK(r.found == base.found, "%s found=%d ucs=%d for %u->%u", name.c_str(),
            r.found, base.found, s, t);
      if (!r.found || !base.found) continue;
      CHECK(valid_path(g, r, s, t), "%s invalid path %u->%u", name.c_str(), s, t);
      if (a == Algorithm::kBFS) {
        // BFS minimizes segments, not meters.
        CHECK(r.path.size() <= base.path.size(), "bfs used more segments %u->%u", s, t);
        CHECK(r.meters >= base.meters - 0.01, "bfs beat ucs?? %u->%u", s, t);
      } else {
        CHECK(near(r.meters, base.meters), "%s %.3f vs ucs %.3f for %u->%u",
              name.c_str(), r.meters, base.meters, s, t);
      }
    }
  }
  CHECK(found > 1000 && missing > 0, "want both reachable and unreachable pairs: %d/%d",
        found, missing);
  std::printf("random queries: %d reachable, %d unreachable, all checked\n", found, missing);
}

static void test_heuristics_do_less_work() {
  Graph g = random_city(80, 80, 21);
  Landmarks lm;
  lm.build(g, 8, 5);
  Router router(g);
  router.set_landmarks(&lm);
  std::vector<NodeId> pool;
  for (NodeId v = 0; v < g.num_nodes(); ++v)
    if (g.in_main_scc[v]) pool.push_back(v);
  std::mt19937 rng(2);
  std::uniform_int_distribution<size_t> pick(0, pool.size() - 1);
  uint64_t ucs = 0, astar = 0, alt = 0;
  for (int q = 0; q < 300; ++q) {
    NodeId s = pool[pick(rng)], t = pool[pick(rng)];
    ucs += router.route(Algorithm::kUCS, s, t).expanded;
    astar += router.route(Algorithm::kAStar, s, t).expanded;
    alt += router.route(Algorithm::kALT, s, t).expanded;
  }
  CHECK(astar < ucs, "A* should expand fewer nodes than UCS (%llu vs %llu)",
        (unsigned long long)astar, (unsigned long long)ucs);
  CHECK(alt < astar, "ALT should expand fewer nodes than A* (%llu vs %llu)",
        (unsigned long long)alt, (unsigned long long)astar);
  std::printf("work: ucs %llu, astar %llu, alt %llu expanded nodes\n",
              (unsigned long long)ucs, (unsigned long long)astar, (unsigned long long)alt);
}

// Two routes from 0 to 3: 0->1->3 (2 segments, long) and 0->2->4->3 (3
// segments, short). BFS must pick the first, UCS the second.
static void test_bfs_is_not_shortest() {
  std::vector<double> lat = {42.0, 42.0, 42.001, 42.002, 42.001};
  std::vector<double> lon = {-83.0, -82.99, -83.0, -83.0, -82.999};
  std::vector<Edge> e = {{0, 1, 5000}, {1, 3, 5000}, {0, 2, 120}, {2, 4, 150}, {4, 3, 150}};
  Graph g = build_graph(lat, lon, e);
  Router router(g);
  SearchResult bfs = router.route(Algorithm::kBFS, 0, 3);
  SearchResult ucs = router.route(Algorithm::kUCS, 0, 3);
  CHECK(bfs.path == std::vector<NodeId>({0, 1, 3}), "bfs path");
  CHECK(ucs.path == std::vector<NodeId>({0, 2, 4, 3}), "ucs path");
  CHECK(near(ucs.meters, 420), "ucs meters %.1f", ucs.meters);
}

// One-way streets: 0->1 exists, 1->0 does not.
static void test_one_way() {
  std::vector<double> lat = {42.0, 42.001};
  std::vector<double> lon = {-83.0, -83.0};
  Graph g = build_graph(lat, lon, {{0, 1, 120}});
  Router router(g);
  for (Algorithm a : {Algorithm::kBFS, Algorithm::kUCS, Algorithm::kBiUCS, Algorithm::kAStar}) {
    CHECK(router.route(a, 0, 1).found, "%s 0->1", algorithm_name(a).c_str());
    CHECK(!router.route(a, 1, 0).found, "%s 1->0 should fail", algorithm_name(a).c_str());
  }
}

// The bidirectional stopping rule. The two searches first touch at node 1
// on the route 0 -> 1 -> 2 (1000 m), but the shortest route is
// 0 -> 3 -> 4 -> 2 (900 m). Stopping at the first meeting would be wrong.
static void test_bidirectional_first_meeting_is_not_best() {
  std::vector<double> lat(5, 42.0), lon(5, -83.0);
  for (int i = 0; i < 5; ++i) lat[i] += i * 1e-5;
  std::vector<Edge> e = {{0, 1, 500}, {1, 2, 500}, {0, 3, 300}, {3, 4, 300}, {4, 2, 300}};
  Graph g = build_graph(lat, lon, e);
  Router router(g);
  SearchResult r = router.route(Algorithm::kBiUCS, 0, 2);
  CHECK(near(r.meters, 900), "biucs %.1f, want 900", r.meters);
  CHECK(r.path == std::vector<NodeId>({0, 3, 4, 2}), "biucs path");
}

static void test_graph_file_round_trip() {
  Graph g = random_city(10, 10, 1);
  std::string path = "/tmp/route_engine_test.bin";
  save_graph(g, path);
  Graph h = load_graph(path);
  CHECK(h.num_nodes() == g.num_nodes() && h.num_edges() == g.num_edges(), "sizes");
  CHECK(h.targets == g.targets && h.weights == g.weights, "edges");
  CHECK(h.main_scc_size == g.main_scc_size, "scc");
}

int main() {
  test_bfs_is_not_shortest();
  test_one_way();
  test_bidirectional_first_meeting_is_not_best();
  test_graph_file_round_trip();
  test_random_queries_match_ucs();
  test_heuristics_do_less_work();
  if (g_failures) {
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
  }
  std::printf("all tests passed\n");
  return 0;
}

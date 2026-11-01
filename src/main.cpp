// route_engine: command-line entry point.
//
//   route_engine bench <graph.bin> [--queries N] [--landmarks K] [--seed S]
//       Random queries across the map; prints a comparison table and checks
//       every algorithm against UCS.
//   route_engine query <graph.bin> <source> <target> [algorithm]
//       One query by node id, all algorithms if none is named.
//   route_engine serve <graph.bin> [--landmarks K]
//       Line protocol on stdin/stdout, used by the web demo:
//         info
//         route <lat1> <lon1> <lat2> <lon2> <algorithm> <max_explored>
//       Each reply is one line of JSON.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "graph.h"
#include "search.h"

using namespace route;

namespace {

struct Options {
  int queries = 1000;
  int landmarks = 8;
  uint32_t seed = 42;
};

double seconds_since(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

Graph load_or_die(const std::string& path) {
  auto t0 = std::chrono::steady_clock::now();
  Graph g = load_graph(path);
  std::fprintf(stderr,
               "loaded %s: %zu nodes, %zu edges, main component %zu nodes (%.1fs)\n",
               path.c_str(), g.num_nodes(), g.num_edges(), g.main_scc_size,
               seconds_since(t0));
  return g;
}

void build_landmarks(const Graph& g, Landmarks& lm, int k, uint32_t seed) {
  if (k <= 0) return;
  auto t0 = std::chrono::steady_clock::now();
  lm.build(g, k, seed);
  std::fprintf(stderr, "built %d landmarks in %.1fs (%.0f MB)\n", lm.count(),
               seconds_since(t0), lm.memory_bytes() / 1e6);
}

double percentile(std::vector<double> v, double p) {
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  size_t i = static_cast<size_t>(p * (v.size() - 1));
  return v[i];
}

int bench(const std::string& path, const Options& opt) {
  Graph g = load_or_die(path);
  Landmarks lm;
  build_landmarks(g, lm, opt.landmarks, opt.seed);
  Router router(g);
  router.set_landmarks(&lm);

  std::vector<NodeId> pool;
  for (NodeId v = 0; v < g.num_nodes(); ++v)
    if (g.in_main_scc[v]) pool.push_back(v);
  std::mt19937 rng(opt.seed);
  std::uniform_int_distribution<size_t> pick(0, pool.size() - 1);
  std::vector<std::pair<NodeId, NodeId>> queries;
  for (int i = 0; i < opt.queries; ++i) queries.push_back({pool[pick(rng)], pool[pick(rng)]});

  std::vector<Algorithm> algos;
  for (Algorithm a : all_algorithms())
    if (a != Algorithm::kALT || router.has_landmarks()) algos.push_back(a);

  std::map<Algorithm, std::vector<double>> ms, expanded, stretch;
  std::map<Algorithm, int> mismatches;
  double total_km = 0;
  for (auto [s, t] : queries) {
    SearchResult base = router.route(Algorithm::kUCS, s, t);
    total_km += base.meters / 1000;
    for (Algorithm a : algos) {
      SearchResult r = a == Algorithm::kUCS ? base : router.route(a, s, t);
      ms[a].push_back(r.millis);
      expanded[a].push_back(static_cast<double>(r.expanded));
      stretch[a].push_back(base.meters > 0 ? r.meters / base.meters : 1.0);
      bool optimal = std::fabs(r.meters - base.meters) <= 1e-6 * base.meters + 0.01;
      if (a != Algorithm::kBFS && (!r.found || !optimal)) mismatches[a]++;
    }
  }

  std::printf("\n%d random queries, average route %.1f km\n\n", opt.queries,
              total_km / opt.queries);
  std::printf("%-8s %12s %12s %16s %14s %12s\n", "algo", "median ms", "p95 ms",
              "mean expanded", "vs UCS nodes", "route len");
  double ucs_expanded = 0;
  for (double x : expanded[Algorithm::kUCS]) ucs_expanded += x;
  for (Algorithm a : algos) {
    double mean_exp = 0, mean_stretch = 0;
    for (double x : expanded[a]) mean_exp += x;
    for (double x : stretch[a]) mean_stretch += x;
    double frac = mean_exp / ucs_expanded;
    mean_exp /= opt.queries;
    mean_stretch /= opt.queries;
    char len[32];
    if (a == Algorithm::kBFS)
      std::snprintf(len, sizeof len, "+%.1f%%", (mean_stretch - 1) * 100);
    else
      std::snprintf(len, sizeof len, mismatches[a] ? "%d WRONG" : "optimal",
                    mismatches[a]);
    std::printf("%-8s %12.2f %12.2f %16.0f %13.1f%% %12s\n", algorithm_name(a).c_str(),
                percentile(ms[a], 0.5), percentile(ms[a], 0.95), mean_exp, frac * 100,
                len);
  }
  int bad = 0;
  for (auto& [a, c] : mismatches) bad += c;
  std::printf("\n%s\n", bad ? "FAILED: some optimal algorithms disagreed with UCS"
                            : "All optimal algorithms matched UCS on every query.");
  return bad ? 1 : 0;
}

int query(const std::string& path, NodeId s, NodeId t, const std::string& only) {
  Graph g = load_or_die(path);
  Landmarks lm;
  if (only.empty() || only == "alt") build_landmarks(g, lm, 8, 42);
  Router router(g);
  router.set_landmarks(&lm);
  for (Algorithm a : all_algorithms()) {
    if (!only.empty() && algorithm_name(a) != only) continue;
    if (a == Algorithm::kALT && !router.has_landmarks()) continue;
    SearchResult r = router.route(a, s, t);
    if (!r.found) {
      std::printf("%-6s no route\n", algorithm_name(a).c_str());
      continue;
    }
    std::printf("%-6s %10.1f m  %6zu nodes on path  %9llu expanded  %8.2f ms\n",
                algorithm_name(a).c_str(), r.meters, r.path.size(),
                static_cast<unsigned long long>(r.expanded), r.millis);
  }
  return 0;
}

void append_coords(std::ostringstream& out, const Graph& g, const std::vector<NodeId>& ids) {
  out << '[';
  for (size_t i = 0; i < ids.size(); ++i) {
    if (i) out << ',';
    out << '[' << g.lat[ids[i]] << ',' << g.lon[ids[i]] << ']';
  }
  out << ']';
}

int serve(const std::string& path, const Options& opt) {
  Graph g = load_or_die(path);
  Landmarks lm;
  build_landmarks(g, lm, opt.landmarks, opt.seed);
  Router router(g);
  router.set_landmarks(&lm);
  std::fprintf(stderr, "ready\n");

  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream in(line);
    std::string cmd;
    in >> cmd;
    std::ostringstream out;
    out.precision(7);
    if (cmd == "info") {
      double lat_sum = 0, lon_sum = 0;
      double min_lat = 90, max_lat = -90, min_lon = 180, max_lon = -180;
      for (NodeId v = 0; v < g.num_nodes(); ++v) {
        lat_sum += g.lat[v];
        lon_sum += g.lon[v];
        min_lat = std::min(min_lat, g.lat[v]);
        max_lat = std::max(max_lat, g.lat[v]);
        min_lon = std::min(min_lon, g.lon[v]);
        max_lon = std::max(max_lon, g.lon[v]);
      }
      out << "{\"nodes\":" << g.num_nodes() << ",\"edges\":" << g.num_edges()
          << ",\"landmarks\":" << lm.count() << ",\"bounds\":[[" << min_lat << ','
          << min_lon << "],[" << max_lat << ',' << max_lon << "]]}";
    } else if (cmd == "route") {
      double lat1, lon1, lat2, lon2;
      std::string name;
      size_t cap = 0;
      Algorithm a;
      in >> lat1 >> lon1 >> lat2 >> lon2 >> name >> cap;
      if (!in || !parse_algorithm(name, &a)) {
        out << "{\"error\":\"usage: route lat1 lon1 lat2 lon2 algorithm max_explored\"}";
      } else if (a == Algorithm::kALT && !router.has_landmarks()) {
        out << "{\"error\":\"ALT is off: the server was started with --landmarks 0\"}";
      } else {
        NodeId s = g.nearest_node(lat1, lon1), t = g.nearest_node(lat2, lon2);
        SearchResult r = router.route(a, s, t, cap);
        out << "{\"algorithm\":\"" << name << "\",\"found\":" << (r.found ? "true" : "false")
            << ",\"meters\":" << r.meters << ",\"expanded\":" << r.expanded
            << ",\"ms\":" << r.millis << ",\"path\":";
        append_coords(out, g, r.path);
        out << ",\"explored\":";
        append_coords(out, g, r.explored);
        out << '}';
      }
    } else {
      out << "{\"error\":\"unknown command\"}";
    }
    std::cout << out.str() << '\n' << std::flush;
  }
  return 0;
}

int usage() {
  std::fprintf(stderr,
               "usage:\n"
               "  route_engine bench <graph.bin> [--queries N] [--landmarks K] [--seed S]\n"
               "  route_engine query <graph.bin> <source> <target> [bfs|ucs|biucs|astar|alt]\n"
               "  route_engine serve <graph.bin> [--landmarks K]\n");
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) return usage();
  std::string cmd = argv[1], path = argv[2];
  Options opt;
  std::vector<std::string> positional;
  for (int i = 3; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--queries" && i + 1 < argc) opt.queries = std::stoi(argv[++i]);
    else if (arg == "--landmarks" && i + 1 < argc) opt.landmarks = std::stoi(argv[++i]);
    else if (arg == "--seed" && i + 1 < argc) opt.seed = std::stoul(argv[++i]);
    else positional.push_back(arg);
  }
  try {
    if (cmd == "bench") return bench(path, opt);
    if (cmd == "serve") return serve(path, opt);
    if (cmd == "query" && positional.size() >= 2)
      return query(path, std::stoul(positional[0]), std::stoul(positional[1]),
                   positional.size() > 2 ? positional[2] : "");
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return usage();
}

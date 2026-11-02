#!/usr/bin/env python3
"""Generate a synthetic road-like graph for testing and benchmarking without
downloading map data.

A jittered grid of intersections around Ann Arbor, ~120 m apart, with curvy
streets (each edge 0-50% longer than the straight line), some one-way
streets, and some missing streets. A 1000 x 1000 grid has 1M intersections.

    python tools/synth_graph.py --rows 1000 --cols 1000 data/synthetic.bin
"""

import argparse

import numpy as np

from graph_file import write_graph

EARTH_RADIUS_M = 6371008.8


def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = np.radians(lat1), np.radians(lat2)
    dp, dl = p2 - p1, np.radians(lon2 - lon1)
    a = np.sin(dp / 2) ** 2 + np.cos(p1) * np.cos(p2) * np.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * np.arcsin(np.minimum(1.0, np.sqrt(a)))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("output")
    ap.add_argument("--rows", type=int, default=300)
    ap.add_argument("--cols", type=int, default=300)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    R, C = args.rows, args.cols

    r, c = np.meshgrid(np.arange(R), np.arange(C), indexing="ij")
    lat = (42.10 + r * 0.0011 + (rng.random((R, C)) - 0.5) * 0.0004).ravel()
    lon = (-84.00 + c * 0.0015 + (rng.random((R, C)) - 0.5) * 0.0005).ravel()
    ids = np.arange(R * C).reshape(R, C)

    a = np.concatenate([ids[:, :-1].ravel(), ids[:-1, :].ravel()])
    b = np.concatenate([ids[:, 1:].ravel(), ids[1:, :].ravel()])
    keep = rng.random(a.size) >= 0.08            # 8% of streets missing
    a, b = a[keep], b[keep]
    meters = haversine_m(lat[a], lon[a], lat[b], lon[b]) * (1 + 0.5 * rng.random(a.size))

    kind = rng.random(a.size)
    fwd = kind >= 0.10                           # 10% one-way b -> a
    bwd = (kind < 0.10) | (kind >= 0.20)         # 10% one-way a -> b
    src = np.concatenate([a[fwd], b[bwd]]).astype(np.uint32)
    dst = np.concatenate([b[fwd], a[bwd]]).astype(np.uint32)
    w = np.concatenate([meters[fwd], meters[bwd]]).astype(np.float32)
    write_graph(args.output, lat, lon, src, dst, w)
    print(f"{R * C:,} nodes, {src.size:,} edges -> {args.output}")


if __name__ == "__main__":
    main()

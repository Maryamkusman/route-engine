#!/usr/bin/env python3
"""Convert an OpenStreetMap extract (.osm.pbf or .osm) into a route-engine graph.

Keeps drivable roads, respects one-way streets, and weights every edge by its
length in meters. Usage:

    pip install osmium numpy
    python tools/osm_to_graph.py michigan-latest.osm.pbf data/michigan.bin

Download extracts from https://download.geofabrik.de/north-america/us/michigan.html
For a faster start, cut out a region first with osmium-tool, e.g. the
Ann Arbor / Detroit area:

    osmium extract -b -84.0,42.1,-82.9,42.6 michigan-latest.osm.pbf -o se-michigan.osm.pbf
"""

import argparse
import math
import struct
import sys
import time
from array import array

import numpy as np
import osmium

# Road types a car can use. Footways, cycleways, tracks etc. are skipped.
DRIVABLE = {
    "motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link",
    "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified",
    "residential", "living_street", "service", "road",
}
NO_ACCESS = {"no", "private"}
EARTH_RADIUS_M = 6371008.8


def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def direction(tags):
    """+1 = one-way forward, -1 = one-way reversed, 0 = both directions."""
    oneway = tags.get("oneway", "")
    if oneway in ("yes", "true", "1"):
        return 1
    if oneway == "-1":
        return -1
    if oneway == "no":
        return 0
    if tags.get("junction") in ("roundabout", "circular"):
        return 1
    if tags.get("highway") in ("motorway", "motorway_link"):
        return 1
    return 0


class RoadHandler(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.index = {}             # OSM node id -> compact id
        self.lat = array("d")
        self.lon = array("d")
        self.src = array("I")
        self.dst = array("I")
        self.w = array("f")
        self.ways = 0

    def node_id(self, n):
        i = self.index.get(n.ref)
        if i is None:
            i = len(self.lat)
            self.index[n.ref] = i
            self.lat.append(n.location.lat)
            self.lon.append(n.location.lon)
        return i

    def way(self, w):
        tags = w.tags
        if tags.get("highway") not in DRIVABLE:
            return
        if tags.get("access") in NO_ACCESS or tags.get("motor_vehicle") in NO_ACCESS:
            return
        if tags.get("area") == "yes":
            return
        nodes = [n for n in w.nodes if n.location.valid()]
        if len(nodes) < 2:
            return
        d = direction(tags)
        self.ways += 1
        prev = nodes[0]
        prev_id = self.node_id(prev)
        for n in nodes[1:]:
            cur_id = self.node_id(n)
            meters = haversine_m(prev.location.lat, prev.location.lon,
                                 n.location.lat, n.location.lon)
            if d >= 0:
                self.src.append(prev_id); self.dst.append(cur_id); self.w.append(meters)
            if d <= 0:
                self.src.append(cur_id); self.dst.append(prev_id); self.w.append(meters)
            prev, prev_id = n, cur_id


def write_graph(path, lat, lon, src, dst, w):
    n, m = len(lat), len(src)
    order = np.argsort(src, kind="stable")
    counts = np.bincount(src, minlength=n)
    offsets = np.zeros(n + 1, dtype=np.uint32)
    np.cumsum(counts, out=offsets[1:])
    with open(path, "wb") as f:
        f.write(b"RTEG")
        f.write(struct.pack("<IQQ", 1, n, m))
        f.write(np.asarray(lat, dtype="<f8").tobytes())
        f.write(np.asarray(lon, dtype="<f8").tobytes())
        f.write(offsets.astype("<u4").tobytes())
        f.write(dst[order].astype("<u4").tobytes())
        f.write(w[order].astype("<f4").tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help=".osm.pbf or .osm file")
    ap.add_argument("output", help="graph file to write, e.g. data/michigan.bin")
    args = ap.parse_args()

    t0 = time.time()
    h = RoadHandler()
    # locations=True makes osmium attach coordinates to each way's nodes.
    h.apply_file(args.input, locations=True, idx="flex_mem")
    if not h.src:
        sys.exit("no drivable roads found in " + args.input)
    src = np.frombuffer(h.src, dtype=np.uint32)
    dst = np.frombuffer(h.dst, dtype=np.uint32)
    w = np.frombuffer(h.w, dtype=np.float32)
    write_graph(args.output, h.lat, h.lon, src, dst, w)
    print(f"{h.ways:,} roads -> {len(h.lat):,} nodes, {len(src):,} edges "
          f"in {time.time() - t0:.1f}s -> {args.output}")


if __name__ == "__main__":
    main()

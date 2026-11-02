"""Writes the route-engine binary graph format (see README). Shared by the
OpenStreetMap converter and the synthetic generator; needs only numpy."""

import struct

import numpy as np


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

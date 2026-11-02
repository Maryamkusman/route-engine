# Route Engine

Shortest-path routing on real road networks, with five search algorithms
implemented from scratch in C++ and benchmarked against each other:
BFS, UCS (Dijkstra), bidirectional UCS, A*, and ALT (A* with landmarks).

Every optimal algorithm is checked against UCS on thousands of random
queries, and a web demo draws each search's route and the intersections it
explored.
<img width="1770" height="869" alt="image" src="https://github.com/user-attachments/assets/8170df81-224c-4655-8be8-6f1de7604012" />

https://route-engine-ukcj.onrender.com 

## Results

300 random queries on a 1M-intersection synthetic road network
(average route 95 km; single CPU core):

| Algorithm         | Median ms | Nodes explored vs UCS | Route        |
|-------------------|----------:|----------------------:|--------------|
| BFS               |      27.5 |                 99.6% | 8.8% longer  |
| UCS (Dijkstra)    |      99.5 |                  100% | optimal      |
| Bidirectional UCS |      70.6 |                 70.4% | optimal      |
| A*                |      43.9 |                 38.4% | optimal      |
| ALT, 8 landmarks  |       8.5 |                  6.2% | optimal      |

ALT preprocessing took 2.9 s and 64 MB. Rerun `bench` on real map data for
real-world numbers.

## Build and test

Requires a C++17 compiler and CMake 3.16+.

    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j
    ./build/test_search

## Get a graph

Synthetic (no download, good for a first run):

    pip install numpy
    python tools/synth_graph.py --rows 1000 --cols 1000 data/synthetic-1m.bin

Real OpenStreetMap data:

    pip install osmium numpy
    # Michigan extract from https://download.geofabrik.de/north-america/us/michigan.html
    osmium extract -b -84.0,42.1,-82.9,42.6 michigan-latest.osm.pbf -o data/se-michigan.osm.pbf
    python tools/osm_to_graph.py data/se-michigan.osm.pbf data/se-michigan.bin

The full state works too; the converter needs several GB of RAM for it.

## Run

    ./build/route_engine bench data/se-michigan.bin --queries 1000 --landmarks 8
    ./build/route_engine query data/se-michigan.bin 12345 67890

    pip install flask
    python web/app.py data/se-michigan.bin     # http://localhost:8000

## Deploy

The Dockerfile builds the engine, runs the tests, and serves the demo with
gunicorn on `$PORT` (default 8000). It uses `data/map.bin` if present,
otherwise a synthetic map.

    docker build -t route-engine .
    docker run -p 8000:8000 route-engine

A 1M-node map needs roughly 250 MB of RAM, so free 512 MB hosting tiers fit.
Run one gunicorn worker: each worker loads its own copy of the graph.

## Layout

    src/graph.*        CSR road graph, binary file format, SCC, nearest node
    src/search.*       BFS, UCS, bidirectional UCS, A*, ALT + landmarks
    src/main.cpp       bench / query / serve commands
    tests/             correctness tests (every algorithm vs. UCS)
    tools/             OpenStreetMap converter, synthetic graph generator
    web/               Flask server + Leaflet map

## Graph file format

Little-endian: `"RTEG"`, uint32 version (1), uint64 n, uint64 m,
double lat[n], double lon[n], uint32 offsets[n+1], uint32 targets[m],
float meters[m]. Node u's outgoing edges are `targets[offsets[u]..offsets[u+1])`.

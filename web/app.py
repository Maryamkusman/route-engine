#!/usr/bin/env python3
"""Web demo: a map where you click two points and watch each algorithm search.

The C++ engine runs as a long-lived child process (`route_engine serve`), so
the graph and landmarks load once. Flask forwards each request to it over
stdin/stdout.

    pip install flask
    python web/app.py data/synthetic-1m.bin
    # open http://localhost:8000
"""

import argparse
import json
import os
import subprocess
import threading

from flask import Flask, abort, jsonify, request, send_from_directory
from werkzeug.exceptions import HTTPException

HERE = os.path.dirname(os.path.abspath(__file__))
ALGORITHMS = ("bfs", "ucs", "biucs", "astar", "alt")


class Engine:
    def __init__(self, binary, graph, landmarks):
        self.proc = subprocess.Popen(
            [binary, "serve", graph, "--landmarks", str(landmarks)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.lock = threading.Lock()  # one request at a time over the pipe

    def ask(self, line):
        with self.lock:
            self.proc.stdin.write(line + "\n")
            self.proc.stdin.flush()
            reply = self.proc.stdout.readline()
        if not reply:
            raise RuntimeError("route_engine exited; check its output above")
        return json.loads(reply)


def parse_point(text):
    try:
        lat, lon = (float(x) for x in text.split(","))
    except (AttributeError, ValueError):
        abort(400, "points look like ?from=42.28,-83.74")
    return lat, lon


def create_app(engine):
    app = Flask(__name__)

    @app.get("/")
    def index():
        return send_from_directory(HERE, "index.html")

    @app.get("/api/info")
    def info():
        return jsonify(engine.ask("info"))

    @app.get("/api/route")
    def route():
        lat1, lon1 = parse_point(request.args.get("from"))
        lat2, lon2 = parse_point(request.args.get("to"))
        algo = request.args.get("algo", "astar")
        if algo not in ALGORITHMS:
            abort(400, f"algo must be one of {', '.join(ALGORITHMS)}")
        explored = max(0, min(request.args.get("explored", 8000, type=int), 50000))
        return jsonify(engine.ask(f"route {lat1} {lon1} {lat2} {lon2} {algo} {explored}"))

    # Errors as JSON, so the page can show the message instead of an HTML page.
    @app.errorhandler(HTTPException)
    def http_error(e):
        return jsonify(error=e.description), e.code

    return app


def app_from_env():
    """Entry point for a production server, configured by environment:

        ROUTE_GRAPH=data/map.bin gunicorn -w 1 --threads 8 --chdir web "app:app_from_env()"

    Use one worker: each worker starts its own engine and its own copy of the
    graph. Threads are fine, since the engine serves one request at a time.
    """
    engine = Engine(os.environ.get("ROUTE_ENGINE", os.path.join(HERE, "..", "build", "route_engine")),
                    os.environ["ROUTE_GRAPH"], int(os.environ.get("ROUTE_LANDMARKS", 8)))
    engine.ask("info")
    return create_app(engine)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("graph", help="graph file, e.g. data/synthetic-1m.bin")
    ap.add_argument("--engine", default=os.path.join(HERE, "..", "build", "route_engine"))
    ap.add_argument("--landmarks", type=int, default=8)
    ap.add_argument("--port", type=int, default=8000)
    args = ap.parse_args()
    if not os.path.exists(args.engine):
        ap.exit(1, f"no engine at {args.engine}; build it first (see README)\n")
    engine = Engine(args.engine, args.graph, args.landmarks)
    engine.ask("info")  # blocks until the graph and landmarks are loaded
    create_app(engine).run(port=args.port, threaded=True)


if __name__ == "__main__":
    main()

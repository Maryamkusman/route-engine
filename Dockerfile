# Builds the engine and a map, then serves the web demo.
#
#   docker build -t route-engine .
#   docker run -p 8000:8000 route-engine        # http://localhost:8000
#
# Uses data/map.bin if you put one there (e.g. from tools/osm_to_graph.py);
# otherwise generates a synthetic map.

FROM python:3.12-slim AS build
RUN apt-get update && apt-get install -y --no-install-recommends g++ cmake make \
    && rm -rf /var/lib/apt/lists/*
RUN pip install --no-cache-dir numpy
WORKDIR /src
COPY CMakeLists.txt .
COPY src src
COPY tests tests
COPY tools tools
COPY data data
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXE_LINKER_FLAGS="-static-libstdc++ -static-libgcc" \
    && cmake --build build -j \
    && ./build/test_search
ARG SYNTH_SIZE=600
RUN [ -f data/map.bin ] || python tools/synth_graph.py --rows $SYNTH_SIZE --cols $SYNTH_SIZE data/map.bin

FROM python:3.12-slim
RUN pip install --no-cache-dir flask gunicorn
WORKDIR /app
COPY --from=build /src/build/route_engine build/route_engine
COPY --from=build /src/data/map.bin data/map.bin
COPY web web
ENV ROUTE_GRAPH=/app/data/map.bin ROUTE_ENGINE=/app/build/route_engine ROUTE_LANDMARKS=8 PORT=8000
EXPOSE 8000
# Engine load and landmark building happen before the server accepts requests,
# hence the long timeout.
CMD gunicorn -w 1 --threads 8 --timeout 120 --chdir web -b 0.0.0.0:$PORT "app:app_from_env()"

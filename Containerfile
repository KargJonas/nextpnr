# The environment the whole flow runs in: yosys, nextpnr-generic with the
# slg47910 uarch, and the Python bitstream tools. Built by `make image`.
FROM docker.io/library/ubuntu:24.04

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        build-essential cmake git ca-certificates python3 python3-dev \
        libboost-dev libboost-filesystem-dev libboost-program-options-dev \
        libboost-thread-dev libboost-iostreams-dev libeigen3-dev \
        yosys \
    && rm -rf /var/lib/apt/lists/*

# nextpnr at the commit the uarch was developed against.
ARG NEXTPNR_COMMIT=7de80fbb9c0fc5bc2e9b680f409579795de6b6c7
COPY nextpnr/upstream.patch /src/upstream.patch
RUN git clone -q https://github.com/YosysHQ/nextpnr.git /src/nextpnr \
    && cd /src/nextpnr && git checkout -q ${NEXTPNR_COMMIT} \
    && git apply /src/upstream.patch

# The uarch, built into that tree.
COPY nextpnr/slg47910 /src/nextpnr/generic/viaduct/slg47910
RUN cd /src/nextpnr \
    && cmake -S . -B build -DARCH=generic -DBUILD_GUI=OFF -DBUILD_PYTHON=OFF \
             -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j"$(nproc)" --target nextpnr-generic \
    && install -m 755 build/nextpnr-generic /usr/local/bin/

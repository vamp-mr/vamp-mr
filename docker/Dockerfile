FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    git \
    ninja-build \
    pkg-config \
    python3 \
    python3-dev \
    python3-pip \
    libstdc++6 \
    libboost-all-dev \
    libeigen3-dev \
    libjsoncpp-dev \
    libompl-dev \
    libprotobuf-dev \
    libtbb-dev \
    libyaml-cpp-dev \
    libeigen3-dev \
    protobuf-compiler \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /root/vamp-mr
COPY . .

RUN ./scripts/setup/install_vamp_mr.sh --prefix /usr/local --type Release --with-vamp --no-sudo

CMD ["bash"]


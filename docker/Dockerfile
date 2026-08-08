FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    libgtest-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY . .

RUN cmake -S . -B build -G Ninja
RUN cmake --build build
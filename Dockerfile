FROM ubuntu:24.04

# Install dependencies
RUN  apt-get update -qq && \
     apt-get install -y \
     libgl-dev \
     cmake \
     build-essential \
     libopenscenegraph-dev \
     libgdal-dev \
     libgeos-dev \
     libsqlite3-dev \
     protobuf-compiler \
     libprotobuf-dev \
     libtinyxml-dev \
     libglew-dev \
     && rm -rf /var/lib/apt/lists/*

COPY . /code
RUN cd /code && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && make install && ldconfig && \
    cd / && \
    rm -fr /code
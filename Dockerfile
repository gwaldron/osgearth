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
     libglx-mesa0 \
     libgl1-mesa-dri \
     mesa-utils \
     xvfb \
     x11-utils \
     xauth \
     x11-xserver-utils \
     libglu1-mesa \
     && rm -rf /var/lib/apt/lists/*

COPY . /code
RUN cd /code && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DOSGEARTH_BUILD_PROCEDURAL_NODEKIT=ON .. && \
    make -j$(nproc) && make install && ldconfig && \
    cd / && \
    rm -fr /code

# https://gist.github.com/whateverforever/53efa2a76e7772da0693e734e4307fa2
# use custom init handler so that xvfb-run is executed properly
# alternatively use docker --init, but that doesnt work in k8s
ENV TINI_VERSION v0.19.0
ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini /tini
RUN chmod +x /tini
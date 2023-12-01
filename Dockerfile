# Build a Docker image to compile and build openscenegraph and osgearth with GL3

ARG UID=1000
ARG GID=1000
ARG HOME=/home/user

FROM ubuntu:focal AS devel

ARG DEBIAN_FRONTEND=noninteractive
ARG HOME

# Install Dependencies
RUN sed -i '/deb-src/s/^# //' /etc/apt/sources.list

RUN apt update && apt upgrade -y                                                         && \
    apt -y install software-properties-common apt-transport-https

RUN add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"      && \
    apt update && apt upgrade -y

RUN apt install -y wget build-essential autoconf locate apt-file libspdlog-dev              \
    git-all fuse libgl1-mesa-dev psmisc libpq-dev libssl-dev openssl libffi-dev             \
    zlib1g-dev libdbus-1-3 desktop-file-utils                                               \
    libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libilmbase-dev                              \
    libxcb-render-util0 libxcb-xinerama0 libxcb-composite0 libxcb-cursor0                   \
    libxcb-damage0 libxcb-dpms0 libxcb-dri2-0 libxcb-dri3-0 libxcb-ewmh2                    \
    libxcb-glx0 libxcb-present0 libxcb-randr0 libxcb-record0 libxcb-render0                 \
    libxcb-res0 libxcb-screensaver0 libxcb-shape0 libxcb-shm0 libxcb-sync1                  \
    libxcb-util1 libfontconfig libfontconfig1 libxcb-xkb1 libxkbcommon-x11-0                \
    libegl1-mesa-dev unixodbc-dev curl unzip tar libnss3 libxcomposite1                     \
    libxcursor-dev libxtst-dev libxrandr-dev libgtk3.0-cil-dev libcurl4-openssl-dev         \
    libomp-dev libstdc++6 ninja-build libboost-all-dev gdal-bin gdal-data                   \
    libgdal-dev libgeos-dev libgeos++-dev librocksdb-dev freeglut3-dev libglu1-mesa-dev     \
    libx11-xcb-dev '^libxcb.*-dev' libxrender-dev libxi-dev libxkbcommon-dev libglew-dev    \
    libxkbcommon-x11-dev libxinerama1 libxrandr2 protobuf-compiler libprotobuf-dev          \
    ffmpeg libavformat-dev libavdevice-dev libavcodec-dev libavutil-dev libswscale-dev      \
    libsdl1.2-dev libsdl2-dev libgtkgl2.0-dev libgtkglext1-dev libjasper-dev libasio-dev    \
    dcmtk libvncserver-dev xfonts-scalable libocct-data-exchange-dev libocct-draw-dev       \
    libocct-foundation-dev libocct-modeling-algorithms-dev libocct-modeling-data-dev        \
    libocct-ocaf-dev libocct-visualization-dev libzip-dev libblosc-dev libpoco-dev          \
    libgl-dev libsqlite3-dev nano libgstreamer1.0-0 gstreamer1.0-plugins-base               \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev    \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly            \
    gstreamer1.0-libav gstreamer1.0-x gstreamer1.0-tools gstreamer1.0-gl gstreamer1.0-gtk3  \
    libgomp1 --fix-missing

# Update gcc for correct c++17 support
ARG GCC=9
RUN echo "Install GCC ${GCC}"                                               && \
    apt -y install g++-${GCC}                                               && \
    update-alternatives                                                        \
    --install /usr/bin/gcc gcc /usr/bin/gcc-${GCC} 60                          \
    --slave /usr/bin/g++ g++ /usr/bin/g++-${GCC}                               \
    --slave /usr/bin/gcc-ar gcc-ar /usr/bin/gcc-ar-${GCC}                      \
    --slave /usr/bin/gcc-nm gcc-nm /usr/bin/gcc-nm-${GCC}                      \
    --slave /usr/bin/gcc-ranlib gcc-ranlib /usr/bin/gcc-ranlib-${GCC}       && \
    update-alternatives --config gcc

# Update cmake
ARG CMAKE=3.26.3
RUN wget -c -nv https://github.com/Kitware/CMake/releases/download/v${CMAKE}/cmake-${CMAKE}-Linux-x86_64.sh     && \
    sh cmake-${CMAKE}-Linux-x86_64.sh --prefix=/usr/local --exclude-subdir                                      && \
    rm cmake-${CMAKE}-Linux-x86_64.sh

ENV LD_LIBRARY_PATH /usr/local/lib/:${LD_LIBRARY_PATH}

# Install Autodesk FBX
RUN mkdir -p ${HOME}/Downloads && cd ${HOME}/Downloads && \
    wget -q --user-agent="Mozilla" https://damassets.autodesk.net/content/dam/autodesk/www/adn/fbx/2020-0-1/fbx202001_fbxsdk_linux.tar.gz && \
    tar xzvf fbx202001_fbxsdk_linux.tar.gz && \
    chmod a+x fbx202001_fbxsdk_linux

RUN mkdir -p /usr/fbx202001_fbxsdk_linux && \
    sh -c '/bin/echo -e "yes\nn" | ${HOME}/Downloads/fbx202001_fbxsdk_linux /usr/fbx202001_fbxsdk_linux' && \
    rm -rf ${HOME}/Downloads

ARG fbx_include="/usr/fbx202001_fbxsdk_linux/include"
ARG fbx_lib_release="/usr/fbx202001_fbxsdk_linux/lib/gcc/x64/release/libfbxsdk.a"
ARG fbx_lib_debug="/usr/fbx202001_fbxsdk_linux/lib/gcc/x64/debug/libfbxsdk.a"
ARG fbx_xml_lib=libxml2.so
ARG fbx_zlib_lib=libz.so

# Install OpenSceneGraph dependancies
RUN apt update && apt upgrade -y && apt build-dep -y openscenegraph

ARG OSG_DIR=/opt/OpenSceneGraph
WORKDIR ${HOME}/gitrepo
RUN git clone https://github.com/openscenegraph/OpenSceneGraph.git                      && \
    mkdir OpenSceneGraph/build && cd OpenSceneGraph/build                               && \
    cmake -DOSG_GL3_AVAILABLE=ON -DOSG_GL1_AVAILABLE=OFF -DOSG_GL2_AVAILABLE=OFF           \
          -DOSG_GLES1_AVAILABLE=OFF -DOSG_GLES2_AVAILABLE=OFF -DOSG_GLES3_AVAILABLE=OFF    \
          -DOSG_GL_DISPLAYLISTS_AVAILABLE=OFF -DOSG_GL_FIXED_FUNCTION_AVAILABLE=OFF        \
          -DOSG_GL_MATRICES_AVAILABLE=OFF -DOSG_GL_VERTEX_ARRAY_FUNCS_AVAILABLE=OFF        \
          -DOSG_GL_VERTEX_FUNCS_AVAILABLE=OFF -DOPENGL_PROFILE=GL3                         \
          -DOSG_GL_CONTEXT_VERSION=4.6                                                     \
          -DFBX_INCLUDE_DIR="$fbx_include" -DFBX_LIBRARY="$fbx_lib_release"                \
          -DFBX_LIBRARY_DEBUG="$fbx_lib_debug" -DFBX_XML2_LIBRARY="$fbx_xml_lib"           \
          -DFBX_ZLIB_LIBRARY="$fbx_zlib_lib"                                               \
          -DCMAKE_INSTALL_PREFIX=${OSG_DIR} -DCMAKE_BUILD_TYPE=Release ..               && \
    cmake --build . --parallel 4 --target install
ENV LD_LIBRARY_PATH /opt/OpenSceneGraph/lib:$LD_LIBRARY_PATH

# Install Draco
ARG DRACO_DIR=/opt/Draco
WORKDIR ${HOME}/gitrepo
RUN git clone https://github.com/google/draco.git                               && \
    mkdir draco/build && cd draco/build                                         && \
    cmake -DCMAKE_INSTALL_PREFIX=${DRACO_DIR} -DCMAKE_BUILD_TYPE=Release ..     && \
    cmake --build . --parallel 4 --target install
ENV LD_LIBRARY_PATH ${DRACO_DIR}/lib:$LD_LIBRARY_PATH

# Install OsgEarth
ARG OSGEARTH_DIR=/opt/osgearth
COPY . ${HOME}/gitrepo/osgearth
WORKDIR ${HOME}/gitrepo
RUN sed -i 's/OPENGL_gl_LIBRARY/OPENGL_LIBRARY/' ${HOME}/gitrepo/osgearth/src/applications/osgearth_imgui/CMakeLists.txt              && \
    sed -i 's/OPENGL_gl_LIBRARY/OPENGL_LIBRARY/' ${HOME}/gitrepo/osgearth/src/applications/osgearth_pick/CMakeLists.txt               && \
    sed -i 's/OPENGL_gl_LIBRARY/OPENGL_LIBRARY/' ${HOME}/gitrepo/osgearth/src/applications/osgearth_collecttriangles/CMakeLists.txt

RUN mkdir -p ${HOME}/gitrepo/osgearth/build && cd ${HOME}/gitrepo/osgearth/build                                        && \                                   
    cmake -DOSG_DIR=${OSG_DIR} -DOpenGL_GL_PREFERENCE=GLVND                                                                \
          -DCMAKE_INSTALL_PREFIX=${OSGEARTH_DIR} -DCMAKE_BUILD_TYPE=Release ..                                          && \
    cmake --build . --parallel 4 --target install

ENV OSG_DIR ${OSG_DIR}
ENV OSGEARTH_DIR ${OSGEARTH_DIR}

# Delete all source files
# RUN rm -rf ${HOME}/gitrepo

# Clean apt cache
RUN apt clean && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

# Create user
ARG UID
ARG GID
RUN groupadd -g "${GID}" user && \
    useradd --create-home --home-dir ${HOME} --no-log-init -u "${UID}" -g "${GID}" user &&\
    chown -R user:user ${HOME}

# Switch to user
USER user

RUN mkdir -p ${HOME}/xdg
ENV XDG_RUNTIME_DIR=${HOME}/xdg
ENV __NV_PRIME_RENDER_OFFLOAD 1
ENV __GLX_VENDOR_LIBRARY_NAME nvidia
ENV LD_LIBRARY_PATH /usr/local/lib64:/opt/OpenSceneGraph/lib:/opt/osgearth/lib64:$LD_LIBRARY_PATH
ENV PATH /usr/local/lib64:/opt/OpenSceneGraph/bin:/opt/osgearth/bin:$PATH

WORKDIR ${HOME}
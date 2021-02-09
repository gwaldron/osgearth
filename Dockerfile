FROM ubuntu:16.04

RUN apt-get update && apt-get install -y curl openssl \
    && curl https://bintray.com/user/downloadSubjectPublicKey?username=bintray | apt-key add - \
    && echo "deb http://dl.bintray.com/jasonbeverage/pelicanmapping xenial main" | tee -a /etc/apt/sources.list \
    && apt-get update -qq \
    && apt-get install -y python-software-properties software-properties-common \
    && add-apt-repository ppa:ubuntugis/ppa --yes \
    && apt-get update -qq \
    && apt-get install -y \
       cmake \
       openscenegraph=3.6.3 \
       gdal-bin \
       libgdal-dev \
       libgeos-dev \
       libsqlite3-dev \
       protobuf-compiler \
       libprotobuf-dev \
       libpoco-dev
COPY . /code
RUN cd /code && cmake -DCMAKE_BUILD_TYPE=Release . && make -j2 && make install && ldconfig
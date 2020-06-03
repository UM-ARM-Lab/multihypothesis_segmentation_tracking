#!/usr/bin/env bash

# Install OctoMap from source
SRC_DIR=~/local/src_repos
mkdir -p ${SRC_DIR}
cd ${SRC_DIR}
git clone --branch devel --depth 1 https://github.com/OctoMap/octomap.git
cd octomap/octomap
mkdir build
cd build
cmake \
	-DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_INSTALL_PREFIX=~/local \
	-DBUILD_TESTING=OFF \
	-DOCTOMAP_OMP=ON \
	..
make -j$(nproc)
make install

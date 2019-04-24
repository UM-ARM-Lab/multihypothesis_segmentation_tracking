#!/usr/bin/env bash

# Install OctoMap from source
mkdir -p ~/local/src_repos
cd ~/local/src_repos
git clone --branch devel --depth 1 https://github.com/OctoMap/octomap.git
cd octomap/octomap
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=~/local -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DOCTOMAP_OMP=ON ..
make -j$(nproc)
make install
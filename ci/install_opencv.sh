#!/usr/bin/env bash

# Install OpenCV from source
mkdir -p ~/local/src_repos
cd ~/local/src_repos
git clone --branch 3.4.5 --depth 1 https://github.com/opencv/opencv.git
git clone --branch 3.4.5 --depth 1 https://github.com/opencv/opencv_contrib.git
cd opencv
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=~/local -DCMAKE_BUILD_TYPE=Release -DOPENCV_EXTRA_MODULES_PATH=~/local/src_repos/opencv_contrib/modules -DOPENCV_ENABLE_NONFREE=ON -DWITH_CUDA=ON -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_JAVA=OFF ..
make -j$(nproc)
make install
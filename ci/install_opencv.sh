#!/usr/bin/env bash

# Install OpenCV from source
SRC_DIR=~/local/src_repos
mkdir -p ${SRC_DIR}
cd ${SRC_DIR}
git clone --branch 3.4.9 --depth 1 https://github.com/opencv/opencv.git
git clone --branch 3.4.9 --depth 1 https://github.com/opencv/opencv_contrib.git
cd opencv
mkdir build
cd build
cmake \
	-DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_INSTALL_PREFIX=~/local \
	-DOPENCV_EXTRA_MODULES_PATH=${SRC_DIR}/opencv_contrib/modules \
	-DOPENCV_ENABLE_NONFREE=ON \
	-DWITH_CUDA=ON \
	-DBUILD_opencv_cudacodec=OFF \
	-DBUILD_EXAMPLES=OFF \
	-DBUILD_TESTS=OFF \
	-DBUILD_PERF_TESTS=OFF \
	-DBUILD_JAVA=OFF \
	-DCPACK_BINARY_DEB:BOOL=ON \
	..
make -j$(nproc)
make install

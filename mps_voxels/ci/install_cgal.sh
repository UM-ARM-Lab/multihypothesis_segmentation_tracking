#!/usr/bin/env bash

# Install CGAL from source
SRC_DIR=~/local/src_repos
mkdir -p ${SRC_DIR}
cd ${SRC_DIR}
git clone --branch releases/CGAL-4.14.3 --depth 1 https://github.com/CGAL/cgal.git
cd cgal
mkdir build
cd build
cmake \
	-DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_INSTALL_PREFIX=~/local \
	-DWITH_CGAL_ImageIO=OFF \
	-DWITH_CGAL_Qt5=OFF \
	-DWITH_Eigen3=ON \
	-DWITH_GMP=OFF \
	-DWITH_MPFR=OFF \
	..
make -j$(nproc)
make install
echo "export CGAL_DIR=~/local/lib/cmake/CGAL/" >> ~/.bashrc

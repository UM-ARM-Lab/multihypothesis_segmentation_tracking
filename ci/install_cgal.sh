#!/usr/bin/env bash

# Install CGAL from source
mkdir -p ~/local/src_repos
cd ~/local/src_repos
git clone --branch releases/CGAL-4.13 --depth 1 https://github.com/CGAL/cgal.git
cd cgal
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=~/local -DCMAKE_BUILD_TYPE=Release -DWITH_CGAL_ImageIO=OFF -DWITH_CGAL_Qt5=OFF -DWITH_Eigen3=ON -DWITH_GMP=OFF -DWITH_MPFR=OFF ..
make -j$(nproc)
make install
echo "export CGAL_DIR=~/local/lib/cmake/CGAL/" >> ~/.bashrc
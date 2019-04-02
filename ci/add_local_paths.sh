#!/usr/bin/env bash

# Create local build/install environment
mkdir -p ~/local/src_repos

# Add the paths to your .bashrc
echo "export PATH=$PATH:~/local/bin" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/local/lib" >> ~/.bashrc
echo "export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:~/local/lib/pkgconfig" >> ~/.bashrc
echo "export CMAKE_PREFIX_PATH=~/local:$CMAKE_PREFIX_PATH" >> ~/.bashrc

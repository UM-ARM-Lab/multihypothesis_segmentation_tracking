#!/bin/bash
declare -a versionList=("kinetic") # "lunar") # "melodic")

# Create the versioned dockerfiles
#for v in "${versionList[@]}"; do
#    sed "s/%ROS_VERSION%/$v/g" Dockerfile > Dockerfile_$v
#done

cd ..
# Build and tag
for v in "${versionList[@]}"; do
    docker build $1 -f ci/Dockerfile -t um_arm_lab/test_mps_voxels:$v --build-arg ROS_VERSION=$v --build-arg SSH_PRIVATE_KEY="$(cat ~/.ssh/id_rsa)" .
done
cd -

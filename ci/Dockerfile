#-----------------------------------------------------
# Private Repo Aquisition
# * Takes a provided ssh key to clone private repos
# * The actual image copies the cloned repos
# * This image and the copied ssh keys are not exposed
#-----------------------------------------------------

# Begin with a base build
ARG ROS_VERSION=kinetic
ARG CUDA_VERSION=9.1-devel-ubuntu16.04
ARG PACKAGE_NAME=mps_voxels
FROM ros:${ROS_VERSION} as getter

# Install missing debian packages
RUN apt-get -yq update && \
    DEBIAN_FRONTEND=noninteractive apt-get -yqq install \
    ssh \
    git && \
    rm -rf /var/lib/apt/lists/*

# Set home directory
ENV HOME /root

# Inject credentials at build time
ARG SSH_PRIVATE_KEY
RUN mkdir /root/.ssh/
RUN echo "${SSH_PRIVATE_KEY}" > /root/.ssh/id_rsa && \
    chmod 600 /root/.ssh/*

# Get keys for target host
RUN touch /root/.ssh/known_hosts
RUN ssh-keyscan github.com >> /root/.ssh/known_hosts

# Set up ROS workspace and clone prerequisites
# TODO: Use wstool merge
RUN cd && \
    mkdir -p catkin_ws/src
ADD ./.rosinstall /root/catkin_ws/src/.rosinstall
RUN cd ~/catkin_ws/src && wstool status && wstool up



#-----------------------------------------------------
# Main image
# * Uses cloned artifacts from "getter"
#-----------------------------------------------------

# Begin with a base build
FROM nvidia/cuda:${CUDA_VERSION}
# Initial ARG is outside of build stage, so inherit here
# https://docs.docker.com/engine/reference/builder/#understand-how-arg-and-from-interact
ARG ROS_VERSION
ARG PACKAGE_NAME

LABEL maintainer="Andrew Price<pricear@umich.edu>"

# Set home directory
ENV HOME /root
ENV ROS_DISTRO ${ROS_VERSION}

# Set bash as sh
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Install ROS Baseline
RUN apt-get -yq update && \
    DEBIAN_FRONTEND=noninteractive apt-get -yqq install \
    dirmngr \
    gnupg2 \
    lsb-release && \
    rm -rf /var/lib/apt/lists/*
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get -yq update && \
    DEBIAN_FRONTEND=noninteractive apt-get -yqq install \
    ros-${ROS_VERSION}-ros-base \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential && \
    rm -rf /var/lib/apt/lists/*

# Set up ROS workspace and clone sources
RUN cd && \
    mkdir -p catkin_ws/src && cd catkin_ws/src && \
    source /opt/ros/${ROS_VERSION}/setup.bash && \
    catkin_init_workspace

# Copy over the cloned source dependencies and package for rosdep
# * This keeps the apt stuff from getting called every time any file in this repo is changed.
COPY --from=getter /root/catkin_ws/src /root/catkin_ws/src/
RUN mkdir -p ~/catkin_ws/src/${PACKAGE_NAME}/
ADD ./package.xml /root/catkin_ws/src/${PACKAGE_NAME}/

# Pre-load the set of apt files to prevent rosdep from running one-at-a-time
ADD ci/apt.txt /
# RUN cd ~/catkin_ws && \
    # rosdep update && \
    # export ROS_PACKAGE_PATH=$(pwd)/src/mps_core:$ROS_PACKAGE_PATH && \
    # rosdep install --simulate --from-paths src --ignore-src -r | awk '/sudo -H apt-get install /{print$5}' >> /apt.txt && \
    # cat /apt.txt

# Install missing rosdeps and apt packages
    # DEBIAN_FRONTEND=noninteractive apt-get -yqq upgrade && \
RUN apt-get -yq update && \
    DEBIAN_FRONTEND=noninteractive xargs -a /apt.txt apt-get -yqq install && \
    cd ~/catkin_ws && \
    rosdep init && \
    rosdep update && \
    rosdep install --as-root "apt:false pip:false" --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Get keys for target host
RUN mkdir /root/.ssh/ && \
    touch /root/.ssh/known_hosts && \
    ssh-keyscan github.com >> /root/.ssh/known_hosts

# Enable cache for repeated builds
RUN export PATH=/usr/lib/ccache:$PATH

# Install packages from source
ADD ci/add_local_paths.sh /scripts/
RUN /scripts/add_local_paths.sh
ADD ci/install_cgal.sh /scripts/
RUN /scripts/install_cgal.sh
ADD ci/install_opencv.sh /scripts/
RUN /scripts/install_opencv.sh
ADD ci/install_octomap.sh /scripts/
RUN /scripts/install_octomap.sh
ENV CGAL_DIR=~/local/lib/cmake/CGAL/
ENV OCTOMAP_DIR=~/local/share/octomap/
ENV OpenCV_DIR=~/local/share/OpenCV/

# Hack for annoying dependency layout
RUN cd ~/catkin_ws/src/mps_core/mps_launch_files && \
    touch CATKIN_IGNORE

# Build and test the prerequisites
RUN cd ~/catkin_ws && \
    rm ~/catkin_ws/src/${PACKAGE_NAME}/package.xml && \
    source ~/.bashrc && \
    source /opt/ros/`rosversion -d`/setup.bash && \
    catkin_make && \
    source ~/catkin_ws/devel/setup.bash && \
    catkin_make run_tests && \
    catkin_test_results

# ADD ./ /root/catkin_ws/src/${PACKAGE_NAME}

# # Build and test the project
# RUN cd ~/catkin_ws && \
#     source ~/.bashrc && \
#     source /opt/ros/`rosversion -d`/setup.bash && \
#     catkin_make && \
#     source ~/catkin_ws/devel/setup.bash && \
#     catkin_make run_tests && \
#     catkin_test_results


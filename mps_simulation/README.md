# mps_simulation

## Installation on a Clean System

Assuming you already have ROS installed and configured, and your desired workspace is `catkin_ws`:

If you want to install things only to the local user, add the following to your `~/.bashrc`/`~/.zshrc` file:
```
# Local install dir
export PATH=$PATH:~/local/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/local/lib
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:~/local/lib/pkgconfig
export CMAKE_PREFIX_PATH=~/local:$CMAKE_PREFIX_PATH
```


Install CGAL:
```
mkdir -p ~/local/src && cd ~/local/src
git clone --depth 1 --branch releases/CGAL-4.14.2 https://github.com/CGAL/cgal.git
cd ~/local/src/cgal
mkdir -p build/release
cd build/release
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=~/local ../..
make -j$(nproc)
make install
```

Clone the current package and its source dependencies
```
cd ~/catkin_ws/src
git clone git@github.com:UM-ARM-Lab/mps_simulation.git
```

```
cd ~/catkin_ws
wstool init src
wstool merge -t src src/mps_simulation/.rosinstall
wstool update -t src
```

Install the broader ROS dependencies:
```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Running Manipulation Demo


```roscore```

```roslaunch mps_simulation test_domain.launch world:=experiment_world```

```rosrun mps_voxels matlab_action_wrapper.py __name:=segment_rgbd _pkg:=mps_msgs _action:=SegmentRGBD _request:=/segment_rgbd/request _response:=/segment_rgbd/response```

```rosrun mps_voxels explorer _image_transport:=raw follow_joint_trajectory:=dual_arm_controller/follow_joint_trajectory```

In Matlab:

```start_segmentation```
or
```rosshutdown; cd ~; start_segmentation```

Start Clion, and run gazebo_segmentation_gt

To run SiamMask tracker, refer to [SiamMask](https://github.com/UM-ARM-Lab/SiamMask).

To run Shape Completion, refer to [Shape Completion](https://github.com/UM-ARM-Lab/mps_shape_completion). 

```ssh arprice@armtyphoon.local``` Password: `VisitDemo`

```export ROS_MASTER_URI=http://armistice.local:11311```

```rosrun mps_shape_completion shape_completion_node.py```
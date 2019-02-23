# mps_voxels

Utilities for dealing with volumetric data for the MPS project.

## Running Shape Completion from Bagged Data

Start up a ROS core

    roscore

Open up RViz so you can see what's up

    rviz -d $(rospack find mps_voxels)/config/shape_completion.rviz

Run the OctoMap server to generate a Octree-based collision map:

    rosrun octomap_server octomap_server_node cloud_in:=/kinect2_victor_head/qhd/points _frame_id:=table_surface _resolution:=0.01 _filter_ground:=false

NB: `cloud_in` should match the Kinect data source (usually `kinect2_victor_head` or `kinect2_roof`).
Next, run the shape completion node on a machine with CUDA, etc. (`ssh armtyphoon.local`, `ssh armlab@odin.local`, etc.)

    rosrun mps_voxels shape_completion_node.py

Run the "manager" node to extract the point cloud as voxels and forward to shape completion:

    rosrun mps_voxels shape_completion_manager

Finally, play back some recorded data:

    rosbag play --clock -d 3 -r 0.3 $(rospack find ap_umich_data)/ycb_separate_head_2018-07-09-19-56-40.bag

In this case we're playing data from https://gitlab.eecs.umich.edu/pricear/ap_umich_data/ at a rate slower than realtime (`-r 0.3`)

## Demo Setup

### Important Commands

In the following, we assume that we are running the ROS Master from `armflare` with an IP address of `10.10.10.190`.

```
roscore
```

```
ssh armlab@realtime.local
export ROS_MASTER_URI=http://armflare.local:11311
roslaunch victor_hardware_interface dual_arm_lcm_bridge.launch
```

```
ssh armlab@loki.local
export ROS_MASTER_URI=http://armflare.local:11311
roslaunch mps_launch_files kinect_vicon_real_robot.launch pov:=victor_head
```

```
roslaunch victor_moveit_config planning_context.launch load_robot_description:=true
rviz -d $(rospack find mps_voxels)/config/demo.rviz
```

```
ssh -X shapereconstruction@odin
export ROS_MASTER_URI=http://10.10.10.190:11311
export ROS_IP=10.10.11.99
matlab -nodesktop
start_segmentation
```

```
ssh armtyphoon.local
export ROS_MASTER_URI=http://armflare.local:11311
cd ~/catkin_ws/src/Shape-Completion-API
./shape_completion_node.py
```

```
rosrun mps_voxels matlab_action_wrapper.py __name:=segment_rgbd _pkg:=mps_msgs _action:=SegmentRGBD _request:=/segment_rgbd/request _response:=/segment_rgbd/response
```

```
rosrun or_victor ros_trajectory_forwarder.py tf:=or_tf tf_static:=or_tf_static
```

### Other Commands of Note

```
rosrun or_victor move_to_impedance_switch.py
```

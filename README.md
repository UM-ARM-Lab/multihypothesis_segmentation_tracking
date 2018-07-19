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
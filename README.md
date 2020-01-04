# mps_interactive_segmentation


```roscore```

```roslaunch mps_interactive_segmentation test_domain.launch world:=experiment_world```

```rosrun mps_voxels matlab_action_wrapper.py __name:=segment_rgbd _pkg:=mps_msgs _action:=SegmentRGBD _request:=/segment_rgbd/request _response:=/segment_rgbd/response```

```rosrun mps_voxels explorer _image_transport:=raw follow_joint_trajectory:=dual_arm_controller/follow_joint_trajectory```

In Matlab:

```start_segmentation```
or
```rosshutdown; cd ~; start_segmentation```

Start Clion, and run gazebo_segmentation_gt

To run SiamMask tracker, refer to [SimaMask](https://github.com/UM-ARM-Lab/SiamMask).
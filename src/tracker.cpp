//
// Created by arprice on 8/13/18.
//

#include "mps_voxels/Tracker.h"

#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include "mps_voxels/colormap.h"

#include <ros/ros.h>



int main(int argc, char* argv[])
{
	ros::init(argc, argv, "flow");
	ros::NodeHandle nh;

	Tracker::TrackingOptions track_options;

	Tracker t;
	t.stopCapture();

	t.startCapture();
	t.track();

	ros::spin();

	return 0;
}
//
// Created by arprice on 2/25/19.
//

#include "mps_simulation/GazeboMocap.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "gazebo_mocap");
	ros::NodeHandle pnh("~");

	bool publish_base = pnh.param("publish_mocap_base", true);

	std::random_device rd;
	size_t seed = rd();

	mps::GazeboMocap mocap(seed);

	std::vector<std::string> models = mocap.getModelNames();

	for (size_t i = 0; i < models.size(); ++i)
	{
		mocap.markerOffsets.insert({{models[i], std::string("link")}, tf::StampedTransform(tf::Transform::getIdentity(), ros::Time(0), mocap.mocap_frame_id, models[i])});
	}

	ros::Rate r(10.0);
	while(ros::ok())
	{
		r.sleep();
		ros::spinOnce();
		mocap.getTransforms();
		mocap.sendTransforms(publish_base);
	}

	return 0;
}

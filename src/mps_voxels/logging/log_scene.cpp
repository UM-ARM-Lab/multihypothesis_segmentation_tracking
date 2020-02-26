//
// Created by kunhuang on 2/24/20.
//

#include "mps_voxels/logging/log_scene.h"
#include "mps_voxels/logging/log_occupancy_data.h"
#include "mps_voxels/logging/log_segmentation_info.h"
#include "mps_voxels/logging/log_cv_mat.h"
#include "mps_voxels/logging/log_cv_roi.h"

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <tf_conversions/tf_eigen.h>


namespace mps
{

template <>
void DataLog::log<Scene>(const std::string& channel, const Scene& msg)
{
	std_msgs::Header header = msg.cameraModel.cameraInfo().header;

	activeChannels.insert(channel + "/cv_rgb_ptr");
	log(channel + "/cv_rgb_ptr", toMessage(msg.cv_rgb_ptr->image, header));

	activeChannels.insert(channel + "/cv_depth_ptr");
	log(channel + "/cv_depth_ptr", toMessage(msg.cv_depth_ptr->image, header));

	activeChannels.insert(channel + "/camera_model");
	log(channel + "/camera_model", msg.cameraModel);

	activeChannels.insert(channel + "/cameraFrame");
	std_msgs::String cf; cf.data = msg.cameraFrame;
	log(channel + "/cameraFrame", cf);

	activeChannels.insert(channel + "/roi");
	log(channel + "/roi", msg.roi);

	activeChannels.insert(channel + "/segInfo");
	log(channel + "/segInfo", *msg.segInfo);

	activeChannels.insert(channel + "/minExtent");
	geometry_msgs::Vector3 rMin;
	rMin.x = msg.minExtent.x();
	rMin.y = msg.minExtent.y();
	rMin.z = msg.minExtent.z();
	log(channel + "/minExtent", rMin);

	activeChannels.insert(channel + "/maxExtent");
	geometry_msgs::Vector3 rMax;
	rMax.x = msg.maxExtent.x();
	rMax.y = msg.maxExtent.y();
	rMax.z = msg.maxExtent.z();
	log(channel + "/maxExtent", rMax);

	activeChannels.insert(channel + "/pile_cloud");
	activeChannels.insert(channel + "/sceneOctree");
	activeChannels.insert(channel + "/occludedPts");

	activeChannels.insert(channel + "/scenario"); /// ?


}

template <>
bool DataLog::load<Scene>(const std::string& channel, Scene& msg)
{
	sensor_msgs::Image rgb;
	load(channel + "/cv_rgb_ptr", rgb);
	msg.cv_rgb_ptr = cv_bridge::toCvCopy(rgb);
	assert(msg.cv_rgb_ptr->image.type() == CV_8UC3);

	sensor_msgs::Image depth;
	load(channel + "/cv_depth_ptr", depth);
	msg.cv_depth_ptr = cv_bridge::toCvCopy(depth);
	assert(msg.cv_depth_ptr->image.type() == CV_16UC1);

	image_geometry::PinholeCameraModel cameraModel;
	load(channel + "/camera_model", cameraModel);
	msg.cameraModel = cameraModel;

	std_msgs::String cf;
	load(channel + "/cameraFrame", cf);
	msg.cameraFrame = cf.data;

	load(channel + "/roi", msg.roi);

	load(channel + "/segInfo", *msg.segInfo);

	geometry_msgs::Vector3 rMin;
	load(channel + "/minExtent", rMin);
	msg.minExtent.x() = rMin.x;
	msg.minExtent.y() = rMin.y;
	msg.minExtent.z() = rMin.z;
	msg.minExtent.w() = 1.0;

	geometry_msgs::Vector3 rMax;
	load(channel + "/maxExtent", rMax);
	msg.maxExtent.x() = rMax.x;
	msg.maxExtent.y() = rMax.y;
	msg.maxExtent.z() = rMax.z;
	msg.maxExtent.w() = 1.0;


	return true;
}

Scene computeSceneFromSensorHistorian(const SensorHistoryBuffer& buffer, const ros::Time& t, const std::string& worldFrame)
{
	Scene sout;
	if (buffer.rgb.find(t) == buffer.rgb.end())
	{
		ROS_ERROR_STREAM("Buffer does not contain target time step.");
		return sout;
	}
	sout.cv_rgb_ptr = buffer.rgb.at(t);
	sout.cv_depth_ptr = buffer.depth.at(t);
	sout.cameraModel = buffer.cameraModel;

	tf::StampedTransform worldTcameraTF;
	geometry_msgs::TransformStamped wTc = buffer.tfs->lookupTransform(worldFrame, buffer.cameraModel.tfFrame(), ros::Time(0));
	tf::transformStampedMsgToTF(wTc, worldTcameraTF);
	tf::transformTFToEigen(worldTcameraTF, sout.worldTcamera);

	SceneProcessor::loadAndFilterScene(sout);
	SceneProcessor::callSegmentation(sout);
	SceneProcessor::computeOcclusions(sout);

	return sout;
}

}

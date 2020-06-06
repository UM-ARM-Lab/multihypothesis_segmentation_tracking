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
Scene DataLog::load<Scene>(const std::string& channel)
{
	Scene msg;
	auto rgb = load<sensor_msgs::Image>(channel + "/cv_rgb_ptr");
	msg.cv_rgb_ptr = cv_bridge::toCvCopy(rgb);
	assert(msg.cv_rgb_ptr->image.type() == CV_8UC3);

	auto depth = load<sensor_msgs::Image>(channel + "/cv_depth_ptr");
	msg.cv_depth_ptr = cv_bridge::toCvCopy(depth);
	assert(msg.cv_depth_ptr->image.type() == CV_16UC1);

	auto cameraModel = load<image_geometry::PinholeCameraModel>(channel + "/camera_model");
	msg.cameraModel = cameraModel;

	auto cf = load<std_msgs::String>(channel + "/cameraFrame");
	msg.cameraFrame = cf.data;

	msg.roi = load<cv::Rect>(channel + "/roi");

	msg.segInfo = std::make_shared<SegmentationInfo>(load<SegmentationInfo>(channel + "/segInfo"));

	auto rMin = load<geometry_msgs::Vector3>(channel + "/minExtent");
	msg.minExtent.x() = rMin.x;
	msg.minExtent.y() = rMin.y;
	msg.minExtent.z() = rMin.z;
	msg.minExtent.w() = 1.0;

	auto rMax = load<geometry_msgs::Vector3>(channel + "/maxExtent");
	msg.maxExtent.x() = rMax.x;
	msg.maxExtent.y() = rMax.y;
	msg.maxExtent.z() = rMax.z;
	msg.maxExtent.w() = 1.0;

	return msg;
}

std::shared_ptr<Scene>
computeSceneFromSensorHistorian(const std::shared_ptr<Scenario>& scenario, const SensorHistoryBuffer& buffer,
                                      const ros::Time& t, const std::string& worldFrame)
{
	std::shared_ptr<Scene> sout = std::make_shared<Scene>();
	if (buffer.rgb.find(t) == buffer.rgb.end())
	{
		ROS_ERROR_STREAM("Buffer does not contain target time step.");
		return sout;
	}
	sout->cv_rgb_ptr = buffer.rgb.at(t);
	sout->cv_depth_ptr = buffer.depth.at(t);
	sout->cameraModel = buffer.cameraModel;
	sout->scenario = scenario;

	tf::StampedTransform worldTcameraTF;
	geometry_msgs::TransformStamped wTc = buffer.tfs->lookupTransform(worldFrame, buffer.cameraModel.tfFrame(), ros::Time(0));
	tf::transformStampedMsgToTF(wTc, worldTcameraTF);
	tf::transformTFToEigen(worldTcameraTF, sout->worldTcamera);

	SceneProcessor::loadAndFilterScene(*sout, *buffer.tfs);
	while (!SceneProcessor::callSegmentation(*sout))
	{
		std::cout << "Segmentation Failed!" << std::endl;
	}
	SceneProcessor::computeOcclusions(*sout);

	return sout;
}

}

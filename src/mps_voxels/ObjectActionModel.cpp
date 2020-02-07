//
// Created by kunhuang on 1/24/20.
//

#include "mps_voxels/ObjectActionModel.h"

namespace mps
{

objectActionModel::objectActionModel() : jlinkageActionClient("cluster_flow", true)
{

}

Eigen::Vector3d
objectActionModel::sampleActionFromMask(const cv::Mat& mask1, const cv::Mat& depth1,
                     const cv::Mat& mask2, const cv::Mat& depth2,
                     const image_geometry::PinholeCameraModel& cameraModel, const moveit::Pose& worldTcamera)
{
	assert(mask1.rows == mask2.rows);
	assert(mask1.cols == mask2.cols);

	assert(depth1.type() == CV_16UC1);
	assert(depth2.type() == CV_16UC1);

	float centerRow1 = 0;
	float centerCol1 = 0;
	float centerRow2 = 0;
	float centerCol2 = 0;
	float mask1size = 0;
	float mask2size = 0;
	for (int i = 0; i < mask1.rows; i++)
	{
		for (int j = 0; j < mask1.cols; j++)
		{
			if(mask1.at<bool>(i, j))
			{
				centerRow1 += (float)i;
				centerCol1 += (float)j;
				mask1size += 1.0;
			}
			if(mask2.at<bool>(i, j))
			{
				centerRow2 += (float)i;
				centerCol2 += (float)j;
				mask2size += 1.0;
			}
		}
	}
	centerRow1 /= mask1size;
	centerCol1 /= mask1size;
	centerRow2 /= mask2size;
	centerCol2 /= mask2size;

	auto dVal1 = depth1.at<uint16_t>((int)floor(centerRow1), (int)floor(centerCol1));
	auto dVal2 = depth2.at<uint16_t>((int)floor(centerRow2), (int)floor(centerCol2));

	float depthVal1 = mps::SensorHistorian::DepthTraits::toMeters(dVal1);
	float depthVal2 = mps::SensorHistorian::DepthTraits::toMeters(dVal2);

	auto pt1 = toPoint3D<Eigen::Vector3f>(centerCol1, centerRow1, depthVal1, cameraModel);
	auto pt2 = toPoint3D<Eigen::Vector3f>(centerCol2, centerRow2, depthVal2, cameraModel);

	Eigen::Vector3d pt1World = worldTcamera * pt1.cast<double>();
	Eigen::Vector3d pt2World = worldTcamera * pt2.cast<double>();

	Eigen::Vector3d objectAction(pt2World[0] - pt1World[0], pt2World[1] - pt1World[1], pt2World[2] - pt1World[2]);

	return objectAction;
}

Eigen::Vector3d
objectActionModel::sampleActionFromMask(const std::vector<std::vector<bool>>& mask1, const cv::Mat& depth1,
                     const std::vector<std::vector<bool>>& mask2, const cv::Mat& depth2,
                     const image_geometry::PinholeCameraModel& cameraModel, const moveit::Pose& worldTcamera)
{
	assert((int)mask1.size() == depth1.rows);
	assert((int)mask1[0].size() == depth1.cols);

	assert((int)mask2.size() == depth2.rows);
	assert((int)mask2[0].size() == depth2.cols);

	assert(depth1.type() == CV_16UC1);
	assert(depth2.type() == CV_16UC1);

	float centerRow1 = 0;
	float centerCol1 = 0;
	float centerRow2 = 0;
	float centerCol2 = 0;
	float mask1size = 0;
	float mask2size = 0;
	for (size_t i = 0; i < mask1.size(); i++)
	{
		for (size_t j = 0; j < mask1[0].size(); j++)
		{
			if(mask1[i][j])
			{
				centerRow1 += (float)i;
				centerCol1 += (float)j;
				mask1size += 1.0;
			}
			if(mask2[i][j])
			{
				centerRow2 += (float)i;
				centerCol2 += (float)j;
				mask2size += 1.0;
			}
		}
	}
	centerRow1 /= mask1size;
	centerCol1 /= mask1size;
	centerRow2 /= mask2size;
	centerCol2 /= mask2size;

	auto dVal1 = depth1.at<uint16_t>((int)floor(centerRow1), (int)floor(centerCol1));
	auto dVal2 = depth2.at<uint16_t>((int)floor(centerRow2), (int)floor(centerCol2));

	float depthVal1 = mps::SensorHistorian::DepthTraits::toMeters(dVal1);
	float depthVal2 = mps::SensorHistorian::DepthTraits::toMeters(dVal2);

	auto pt1 = toPoint3D<Eigen::Vector3f>(centerCol1, centerRow1, depthVal1, cameraModel);
	auto pt2 = toPoint3D<Eigen::Vector3f>(centerCol2, centerRow2, depthVal2, cameraModel);

	Eigen::Vector3d pt1World = worldTcamera * pt1.cast<double>();
	Eigen::Vector3d pt2World = worldTcamera * pt2.cast<double>();

	Eigen::Vector3d objectAction(pt2World[0] - pt1World[0], pt2World[1] - pt1World[1], pt2World[2] - pt1World[2]);

	return objectAction;
}

void objectActionModel::clusterRigidBodyTransformation(const std::map<std::pair<ros::Time, ros::Time>, Tracker::Flow3D>& flows3camera, const moveit::Pose& worldTcamera)
{
	for (auto& t2f : flows3camera) // go through all time steps
	{
		if (t2f.second.size() < 3) { ROS_ERROR_STREAM("Too few matches for jlinkage!"); continue; }

		mps_msgs::ClusterRigidMotionsGoal goal;
		mps_msgs::ClusterRigidMotionsResultConstPtr response;
		for (auto& f : t2f.second)
		{
			//// transform flows to worldframe
			Eigen::Vector3d pos1 = worldTcamera * f.first;
			Eigen::Vector3d pos2 = worldTcamera * (f.first + f.second);
			Eigen::Vector3d vel = pos2 - pos1;

			mps_msgs::FlowVector fv;
			fv.pos.x = pos1.x();
			fv.pos.y = pos1.y();
			fv.pos.z = pos1.z();
			fv.vel.x = vel.x();
			fv.vel.y = vel.y();
			fv.vel.z = vel.z();
			goal.flow_field.push_back(fv);
		}
		if (!jlinkageActionClient.isServerConnected())
		{
			std::cerr << "jlinkage server not connected" << std::endl;
		}

		auto success = jlinkageActionClient.sendGoalAndWait(goal);
		if (!success.isDone())
		{
			std::cerr << "jlinkage not done" << std::endl;
		}

		const auto& res = jlinkageActionClient.getResult();
		assert(res->motions.size() <= res->labels.size());
		for (size_t i = 0; i < res->motions.size(); ++i)
		{
			int count = std::count(res->labels.begin(), res->labels.end(), (int) i);
			if (count >= 3)
			{
				std::cerr << "Found valid rigid body transformation with " << count << " inliers.";
				std::cerr << "\t Linear: " << res->motions[i].linear.x << " " << res->motions[i].linear.y << " " << res->motions[i].linear.z;
				std::cerr << "\t Angular: " << res->motions[i].angular.x << " " << res->motions[i].angular.y << " " << res->motions[i].angular.z << std::endl;
			}
		}
	}

}

std::shared_ptr<octomap::OcTree>
moveOcTree(const octomap::OcTree* octree, const Eigen::Vector3d& action)
{
	std::shared_ptr<octomap::OcTree> resOcTree = std::make_shared<octomap::OcTree>(octree->getResolution());
	for (auto node = octree->begin_leafs(); node != octree->end_leafs(); node++){
		auto originalCoord = node.getCoordinate();
//		std::cerr << "Node center: " << originalCoord;
//		std::cerr << " value: " << node->getValue() << "\n";

		resOcTree->updateNode(originalCoord.x()+action.x(), originalCoord.y()+action.y(), originalCoord.z()+action.z(), true);
		resOcTree->setNodeValue(originalCoord.x()+action.x(), originalCoord.y()+action.y(), originalCoord.z()+action.z(), node->getValue());
	}
	resOcTree->setOccupancyThres(octree->getOccupancyThres());
	return resOcTree;
}

}
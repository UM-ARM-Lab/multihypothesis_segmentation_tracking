//
// Created by kunhuang on 1/24/20.
//

#include "mps_voxels/ObjectActionModel.h"

namespace mps
{

Eigen::Vector3d
sampleActionFromMask(const std::vector<std::vector<bool>>& mask1, const cv::Mat& depth1,
                     const std::vector<std::vector<bool>>& mask2, const cv::Mat& depth2,
                     const image_geometry::PinholeCameraModel& cameraModel, const moveit::Pose& worldTcamera)
{
	MPS_ASSERT((int)mask1.size() == depth1.rows);
	MPS_ASSERT((int)mask1[0].size() == depth1.cols);

	MPS_ASSERT((int)mask2.size() == depth2.rows);
	MPS_ASSERT((int)mask2[0].size() == depth2.cols);

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
//	std::cerr << "center of mask 1: " << (int)floor(centerRow1) << " " << (int)floor(centerCol1) << std::endl;
	centerRow2 /= mask2size;
	centerCol2 /= mask2size;
//	std::cerr << "center of mask 2: " << (int)floor(centerRow2) << " " << (int)floor(centerCol2) << std::endl;

	auto dVal1 = depth1.at<uint16_t>((int)floor(centerRow1), (int)floor(centerCol1));
	auto dVal2 = depth2.at<uint16_t>((int)floor(centerRow2), (int)floor(centerCol2));

	float depthVal1 = mps::SensorHistorian::DepthTraits::toMeters(dVal1);
//	std::cerr << "depth 1 = " << depthVal1 << std::endl;
	float depthVal2 = mps::SensorHistorian::DepthTraits::toMeters(dVal2);
//	std::cerr << "depth 2 = " << depthVal2 << std::endl;

	auto pt1 = toPoint3D<Eigen::Vector3f>(centerCol1, centerRow1, depthVal1, cameraModel);
//	std::cerr << "Before action in camera frame: " << pt1.x() << " " << pt1.y() << " " << pt1.z() << std::endl;
	auto pt2 = toPoint3D<Eigen::Vector3f>(centerCol2, centerRow2, depthVal2, cameraModel);
//	std::cerr << "After action in camera frame: " << pt2.x() << " " << pt2.y() << " " << pt2.z() << std::endl;

	Eigen::Vector3d pt1World = worldTcamera * pt1.cast<double>();
//	std::cerr << "Before action in world frame: " << pt1World.x() << " " << pt1World.y() << " " << pt1World.z() << std::endl;
//	pt1World = worldTcamera.inverse() * pt1.cast<double>();
//	std::cerr << "Before action in world frame (inv): " << pt1World.x() << " " << pt1World.y() << " " << pt1World.z() << std::endl;
	Eigen::Vector3d pt2World = worldTcamera * pt2.cast<double>();
//	std::cerr << "After action in world frame: " << pt2World.x() << " " << pt2World.y() << " " << pt2World.z() << std::endl;
//	pt2World = worldTcamera.inverse() * pt2.cast<double>();
//	std::cerr << "After action in world frame (inv): " << pt2World.x() << " " << pt2World.y() << " " << pt2World.z() << std::endl;

	Eigen::Vector3d objectAction(pt2World[0] - pt1World[0], pt2World[1] - pt1World[1], pt2World[2] - pt1World[2]);

	return objectAction;
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
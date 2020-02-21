//
// Created by kunhuang on 1/24/20.
//

#include "mps_voxels/ObjectActionModel.h"
#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/pointcloud_utils.h"
#include "mps_voxels/project_point.hpp"

#include <tf_conversions/tf_eigen.h>
#include <random>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

namespace mps
{

ObjectActionModel::ObjectActionModel(int n) : numSamples(n), jlinkageActionClient("cluster_flow", true)
{

}

Eigen::Vector3d
ObjectActionModel::sampleActionFromMask(const cv::Mat& mask1, const cv::Mat& depth1,
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

RigidTF ObjectActionModel::icpManifoldSampler(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, const std::map<ros::Time, cv::Mat>& masks, const moveit::Pose& worldTcamera)
{
	//// Construct PointCloud Segments
	pcl::PointCloud<PointT>::Ptr initCloudSegment = make_PC_segment(buffer.rgb.at(steps[0])->image, buffer.depth.at(steps[0])->image,
	                                                                buffer.cameraModel, masks.at(steps[0]));
	assert(!initCloudSegment->empty());
	pcl::PointCloud<PointT>::Ptr lastCloudSegment = make_PC_segment(buffer.rgb.at(steps[ steps.size()-1 ])->image,
	                                                                buffer.depth.at(steps[ steps.size()-1 ])->image,
	                                                                buffer.cameraModel,
	                                                                masks.at(steps[ steps.size()-1 ]));
	assert(!lastCloudSegment->empty());

	//// ICP
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputSource(initCloudSegment);
	icp.setInputTarget(lastCloudSegment);
	pcl::PointCloud<PointT> Final;
	icp.align(Final);
	std::cout << "is Converged: " << icp.hasConverged() << "; Score = " << icp.getFitnessScore() << std::endl;
	// TODO: use icp.getFitnessScore() to add random disturbance
	Eigen::Matrix<float, 4, 4> Mcamera = icp.getFinalTransformation();
	Eigen::Matrix<double, 4, 4> Mworld = worldTcamera.matrix() * Mcamera.cast<double>() * worldTcamera.inverse().matrix();
//	std::cout << Mworld << std::endl;

	RigidTF twist;
	twist.linear = {Mworld(0, 3), Mworld(1, 3), Mworld(2, 3)};
	double theta = acos((Mworld(0, 0) + Mworld(1, 1) + Mworld(2, 2) - 1) / 2.0);
	if (theta == 0) twist.angular = {0, 0, 0};
	else
	{
		Eigen::Vector3d temp = {Mworld(2, 1)-Mworld(1, 2), Mworld(0, 2)-Mworld(2, 0), Mworld(1, 0)-Mworld(0,1)};
		Eigen::Vector3d omega = 1/(2 * sin(theta)) * temp;
		twist.angular = theta * omega;
	}
	std::cerr << "ICP TF: ";
	std::cerr << "\t Linear: " << twist.linear.x() << " " << twist.linear.y() << " " << twist.linear.z();
	std::cerr << "\t Angular: " << twist.angular.x() << " " << twist.angular.y() << " " << twist.angular.z() << std::endl;
	return twist;
}

bool ObjectActionModel::clusterRigidBodyTransformation(const std::map<std::pair<ros::Time, ros::Time>, Tracker::Flow3D>& flows3camera, const moveit::Pose& worldTcamera)
{
	ROS_INFO("Waiting for Jlinkage server to start.");
	// wait for the action server to start
	jlinkageActionClient.waitForServer(); //will wait for infinite time

	ROS_INFO("Jlinkage server started, sending goal.");

	bool isClusterExist = false;
	siftRigidTFs.clear();
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

		//// Label starts with 1!!!
		for (size_t i = 0; i < res->motions.size(); ++i)
		{
			int count = std::count(res->labels.begin(), res->labels.end(), (int) i+1);
			if (count >= 3)
			{
				isClusterExist = true;
				std::cerr << "Found valid rigid body transformation with " << count << " inliers.";
				std::cerr << "\t Linear: " << res->motions[i].linear.x << " " << res->motions[i].linear.y << " " << res->motions[i].linear.z;
				std::cerr << "\t Angular: " << res->motions[i].angular.x << " " << res->motions[i].angular.y << " " << res->motions[i].angular.z << std::endl;
				Eigen::Vector3d linear(res->motions[i].linear.x, res->motions[i].linear.y, res->motions[i].linear.z);
				Eigen::Vector3d angular(res->motions[i].angular.x, res->motions[i].angular.y, res->motions[i].angular.z);
				RigidTF rbt;
				rbt.linear = linear;
				rbt.angular = angular;
				rbt.numInliers = count;
				siftRigidTFs.push_back(rbt);
			}
		}
	}
	return isClusterExist;
}

bool ObjectActionModel::sampleAction(SensorHistoryBuffer& buffer_out, SegmentationInfo& seg_out, std::unique_ptr<Tracker>& sparseTracker, std::unique_ptr<DenseTracker>& denseTracker, uint16_t label, mps_msgs::AABBox2d& bbox)
{
	cv::Mat startMask = cv::Mat::zeros(buffer_out.rgb.begin()->second->image.size(), CV_8UC1);
	cv::Mat subwindow(startMask, seg_out.roi);
	subwindow = label == seg_out.objectness_segmentation->image;
	return sampleAction(buffer_out, startMask, sparseTracker, denseTracker, label, bbox);
}

bool ObjectActionModel::sampleAction(SensorHistoryBuffer& buffer_out, cv::Mat& firstFrameSeg, std::unique_ptr<Tracker>& sparseTracker, std::unique_ptr<DenseTracker>& denseTracker, uint16_t label, mps_msgs::AABBox2d& bbox)
{
	actionSamples.clear();
	/////////////////////////////////////////////
	//// Construct tracking time steps
	/////////////////////////////////////////////
	std::vector<ros::Time> steps; // SiamMask tracks all these time steps except the first frame;
	for (auto iter = buffer_out.rgb.begin(); iter != buffer_out.rgb.end(); std::advance(iter, 5))
	{
		steps.push_back(iter->first);
	}
	std::vector<ros::Time> timeStartEnd;
	timeStartEnd.push_back(steps[0]);
	timeStartEnd.push_back(steps.back());

	/////////////////////////////////////////////
	//// Look up worldTcamera
	/////////////////////////////////////////////
	const std::string tableFrame = "table_surface";
	tf::StampedTransform worldTcameraTF;
	geometry_msgs::TransformStamped wTc = buffer_out.tfs->lookupTransform(tableFrame, buffer_out.cameraModel.tfFrame(), ros::Time(0));
	tf::transformStampedMsgToTF(wTc, worldTcameraTF);
	moveit::Pose worldTcamera;
	tf::transformTFToEigen(worldTcameraTF, worldTcamera);

	/////////////////////////////////////////////
	//// Tracking
	/////////////////////////////////////////////
	std::cerr << "-------------------------------------------------------------------------------------" << std::endl;
	//// SiamMask tracking: construct masks
	std::map<ros::Time, cv::Mat> masks;
	bool denseTrackSuccess = denseTracker->track(steps, buffer_out, bbox, masks);
	if (!denseTrackSuccess)
	{
		ROS_ERROR_STREAM("Dense Track Failed!!!");
		return false;
	}
	if (masks.find(steps.back()) == masks.end())
	{
		ROS_ERROR_STREAM("Failed to estimate motion because of insufficient masks! Return!");
		return false;
	}
	//// Fill in the first frame mask
	cv::Mat startMask = label == firstFrameSeg;
	masks.insert(masks.begin(), {steps.front(), startMask});

	/////////////////////////////////////////////
	//// Estimate motion using SiamMask
	/////////////////////////////////////////////
	Eigen::Vector3d roughMotion_w = sampleActionFromMask(masks[steps[0]], buffer_out.depth[steps[0]]->image,
	                                                     masks[steps.back()], buffer_out.depth[steps.back()]->image,
	                                                     buffer_out.cameraModel, worldTcamera);
	std::cerr << "Rough Motion from SiamMask: " << roughMotion_w.x() << " " << roughMotion_w.y() << " " << roughMotion_w.z() << std::endl;
	Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
	guess(0, 3) = roughMotion_w.x();
	guess(1, 3) = roughMotion_w.y();
	guess(2, 3) = roughMotion_w.z();
	Eigen::Matrix4f roughMotion_c = worldTcamera.inverse().matrix().cast<float>() * guess * worldTcamera.matrix().cast<float>();

	/////////////////////////////////////////////
	//// ICP check & store
	/////////////////////////////////////////////
	pcl::PointCloud<PointT>::Ptr initCloudSegment = make_PC_segment(buffer_out.rgb.at(steps[0])->image, buffer_out.depth.at(steps[0])->image,
	                                                                buffer_out.cameraModel, masks.at(steps[0]));
	assert(!initCloudSegment->empty());
	pcl::PointCloud<PointT>::Ptr lastCloudSegment = make_PC_segment(buffer_out.rgb.at(steps.back())->image,
	                                                                buffer_out.depth.at(steps.back())->image,
	                                                                buffer_out.cameraModel,
	                                                                masks.at(steps.back()));
	assert(!lastCloudSegment->empty());
	if (!isSiamMaskValidICPbased(initCloudSegment, lastCloudSegment, worldTcamera, 0.002, true, roughMotion_c)) // TODO: decide the value of threshold
	{
		//// Generate reasonable disturbance in ParticleFilter together with failed situations
		std::cerr << "Although ICP tells us SiamMask isn't reasonable, we still use it for now." << std::endl;
//		return false;
	}

	//// SIFT
	sparseTracker->track(timeStartEnd, buffer_out, masks, "/home/kunhuang/Videos/" + std::to_string((int)label) + "_");

	/////////////////////////////////////////////
	//// send request to jlinkage server & sample object motions
	/////////////////////////////////////////////
	if (clusterRigidBodyTransformation(sparseTracker->flows3, worldTcamera))
	{
		std::cerr << "USE sift" << std::endl;
		weightedSampleSIFT((int)round(numSamples * 0.4));
		for (int i = 0; i < numSamples - (int)round(numSamples * 0.4); ++i)
		{
			actionSamples.push_back(icpRigidTF);
		}
	}
	else
	{
		std::cerr << "USE SiamMask Manifold" << std::endl;
		for (int i = 0; i < numSamples; ++i)
		{
			actionSamples.push_back(icpRigidTF);
		}
	}
	for (auto& as : actionSamples)
	{
		std::cerr << "Action sample:";
		std::cerr << "\t Linear: " << as.linear.x() << " " << as.linear.y() << " " << as.linear.z();
		std::cerr << "\t Angular: " << as.angular.x() << " " << as.angular.y() << " " << as.angular.z() << std::endl;
	}
	return true;
}

void ObjectActionModel::weightedSampleSIFT(int n)
{
	std::default_random_engine generator;
	std::vector<int> weightBar;

	for (auto& rbt : siftRigidTFs)
	{
		weightBar.push_back(rbt.numInliers);
	}
	std::discrete_distribution<int> distribution(weightBar.begin(), weightBar.end());
	std::cout << "Probabilities: ";
	for (double x:distribution.probabilities()) std::cout << x << " ";
	std::cout << std::endl;

	for (int i=0; i<n; ++i) {
		int index = distribution(generator);
		actionSamples.push_back(siftRigidTFs[index]);
	}
}

bool ObjectActionModel::isSiamMaskValidICPbased(const pcl::PointCloud<PointT>::Ptr& initCloudSegment, const pcl::PointCloud<PointT>::Ptr& lastCloudSegment,
                             const moveit::Pose& worldTcamera, const double& scoreThreshold,
                             const bool& useGuess, const Eigen::Matrix4f& guessCamera)
{
	assert(!initCloudSegment->empty());
	assert(!lastCloudSegment->empty());

	bool isValid = true;
	//// ICP
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaximumIterations(50);
//	icp.setTransformationEpsilon(1e-9);
	icp.setEuclideanFitnessEpsilon(0.1);
	icp.setInputSource(initCloudSegment);
	icp.setInputTarget(lastCloudSegment);
	pcl::PointCloud<PointT> Final;
	if (useGuess) { icp.align(Final, guessCamera); }
	else { icp.align(Final); }
	double score = icp.getFitnessScore();
	std::cerr << "ICP is Converged: " << icp.hasConverged() << "; Score = " << score << std::endl;
	if (score > scoreThreshold)
	{
		ROS_ERROR_STREAM("SiamMask result doesn't make sense according to ICP.");
		isValid = false;
	}

	Eigen::Matrix<float, 4, 4> Mcamera = icp.getFinalTransformation();
	Eigen::Matrix<double, 4, 4> Mworld = worldTcamera.matrix() * Mcamera.cast<double>() * worldTcamera.inverse().matrix();

	RigidTF twist;
	twist.linear = {Mworld(0, 3), Mworld(1, 3), Mworld(2, 3)};
	double theta = acos((Mworld(0, 0) + Mworld(1, 1) + Mworld(2, 2) - 1) / 2.0);
	if (theta == 0) twist.angular = {0, 0, 0};
	else
	{
		Eigen::Vector3d temp = {Mworld(2, 1)-Mworld(1, 2), Mworld(0, 2)-Mworld(2, 0), Mworld(1, 0)-Mworld(0,1)};
		Eigen::Vector3d omega = 1/(2 * sin(theta)) * temp;
		twist.angular = theta * omega;
	}
	std::cerr << "ICP TF: ";
	std::cerr << "\t Linear: " << twist.linear.x() << " " << twist.linear.y() << " " << twist.linear.z();
	std::cerr << "\t Angular: " << twist.angular.x() << " " << twist.angular.y() << " " << twist.angular.z() << std::endl;

	icpRigidTF = twist;
	return isValid;
}

std::shared_ptr<octomap::OcTree>
moveOcTree(const octomap::OcTree* octree, const RigidTF& action)
{
	double theta = action.angular.norm();
//	std::cerr << "rotation theta = " << theta << std::endl;
	Eigen::Vector3d e;
	if (theta == 0) { e = {0, 0, 0}; }
	else { e = action.angular / theta; }

	std::shared_ptr<octomap::OcTree> resOcTree = std::make_shared<octomap::OcTree>(octree->getResolution());
	for (auto node = octree->begin_leafs(); node != octree->end_leafs(); node++)
	{
		auto originalCoord = node.getCoordinate();
		Eigen::Vector3d oCoord(originalCoord.x(), originalCoord.y(), originalCoord.z());
		Eigen::Vector3d newCoord;
		newCoord = cos(theta) *  oCoord + sin(theta) * e.cross(oCoord) + (1 - cos(theta)) * e.dot(oCoord) * e;
		newCoord += action.linear;
		resOcTree->updateNode(newCoord.x(), newCoord.y(), newCoord.z(), true);
		resOcTree->setNodeValue(newCoord.x(), newCoord.y(), newCoord.z(), node->getValue());
	}
	resOcTree->setOccupancyThres(octree->getOccupancyThres());
	return resOcTree;
}

Particle
moveParticle(const Particle& inputParticle, const std::map<int, RigidTF>& labelToMotionLookup)
{
	std::map<int, DecomposedRigidTF> labelToDecomposedMotionLookup;
	for (auto& pair : labelToMotionLookup)
	{
//		Eigen::AngleAxisd aa(7, Eigen::Vector3d::UnitZ());
//		aa.matrix();
//		Eigen::Matrix3d R(aa);
//		Eigen::Isometry3d T;
//		T.linear() = aa.matrix();
//		Eigen::Quaterniond q(aa);
		DecomposedRigidTF drtf;
		drtf.linear = pair.second.linear;
		drtf.theta = pair.second.angular.norm();
		if (drtf.theta == 0) { drtf.e = {0, 0, 0}; }
		else { drtf.e = pair.second.angular / drtf.theta; }
		labelToDecomposedMotionLookup.insert({pair.first, drtf});
	}

	Particle outputParticle;
	outputParticle.state = std::make_shared<OccupancyData>(inputParticle.state->voxelRegion);

#pragma omp parallel for
	for (int i = 0; i < (int)inputParticle.state->vertexState.size(); ++i)
	{
		if (inputParticle.state->vertexState[i] >= 0)
		{
			auto& tf = labelToDecomposedMotionLookup[inputParticle.state->vertexState[i]];
			VoxelRegion::vertex_descriptor vd = inputParticle.state->voxelRegion->vertex_at(i);
			Eigen::Vector3d originalCoord = vertexDescpToCoord(inputParticle.state->voxelRegion->resolution, inputParticle.state->voxelRegion->regionMin, vd);
			Eigen::Vector3d newCoord;
			newCoord = cos(tf.theta) * originalCoord + sin(tf.theta) * tf.e.cross(originalCoord) + (1 - cos(tf.theta)) * tf.e.dot(originalCoord) * tf.e;
			newCoord += tf.linear;
			if (!outputParticle.state->voxelRegion->isInRegion(newCoord)) continue;

			VoxelRegion::vertex_descriptor newVD = coordToVertexDesc(outputParticle.state->voxelRegion->resolution, outputParticle.state->voxelRegion->regionMin, newCoord);
			auto index = outputParticle.state->voxelRegion->index_of(newVD);
			outputParticle.state->vertexState[index] = inputParticle.state->vertexState[i];
		}
	}

	return outputParticle;
}

}
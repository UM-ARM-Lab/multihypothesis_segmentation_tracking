//
// Created by kunhuang on 1/24/20.
//

#include "mps_voxels/ObjectActionModel.h"
#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/pointcloud_utils.h"
#include "mps_voxels/project_point.hpp"
#include "mps_voxels/Scene.h"

#include <tf_conversions/tf_eigen.h>
#include <random>

#include "mps_voxels/PointT.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

// For visualization
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <mps_voxels/image_output.h>

namespace mps
{

std::shared_ptr<ObjectActionModel> estimateMotion(std::shared_ptr<const Scenario> scenario_, const SensorHistoryBuffer& buffer,
                                                  const cv::Mat& firstFrameSeg, uint16_t label, mps_msgs::AABBox2d& bbox,
                                                  std::unique_ptr<Tracker>& sparseTracker, std::unique_ptr<DenseTracker>& denseTracker, int n)
{
	try
	{
		return std::make_shared<ObjectActionModel>(scenario_, buffer, firstFrameSeg, label, bbox, sparseTracker, denseTracker, n);
	}
	catch (const std::runtime_error&)
	{
		return {};
	}
}


Eigen::Vector3d
ObjectActionModel::sampleActionFromMask(const cv::Mat& mask1, const cv::Mat& depth1,
                     const cv::Mat& mask2, const cv::Mat& depth2,
                     const image_geometry::PinholeCameraModel& cameraModel, const mps::Pose& worldTcamera)
{
	assert(mask1.rows == mask2.rows);
	assert(mask1.cols == mask2.cols);

	assert(depth1.type() == CV_16UC1);
	assert(depth2.type() == CV_16UC1);

	// TODO: Use moments command:
	cv::Moments m1 = cv::moments(mask1,true);
	cv::Point2f p1(m1.m10/m1.m00, m1.m01/m1.m00);
	cv::Moments m2 = cv::moments(mask2,true);
	cv::Point2f p2(m2.m10/m2.m00, m2.m01/m2.m00);
	float centerRow1 = p1.y;
	float centerCol1 = p1.x;
	float centerRow2 = p2.y;
	float centerCol2 = p2.x;

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

//RigidTF ObjectActionModel::icpManifoldSampler(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer,
//                                              const std::map<ros::Time, cv::Mat>& masks, const mps::Pose& worldTcamera)
//{
//	//// Construct PointCloud Segments
//	pcl::PointCloud<PointT>::Ptr initCloudSegment = make_PC_segment(buffer.rgb.at(steps[0])->image, buffer.depth.at(steps[0])->image,
//	                                                                buffer.cameraModel, masks.at(steps[0]));
//	assert(!initCloudSegment->empty());
//	pcl::PointCloud<PointT>::Ptr lastCloudSegment = make_PC_segment(buffer.rgb.at(steps[ steps.size()-1 ])->image,
//	                                                                buffer.depth.at(steps[ steps.size()-1 ])->image,
//	                                                                buffer.cameraModel,
//	                                                                masks.at(steps[ steps.size()-1 ]));
//	assert(!lastCloudSegment->empty());
//
//	//// ICP
//	pcl::IterativeClosestPoint<PointT, PointT> icp;
//	icp.setInputSource(initCloudSegment);
//	icp.setInputTarget(lastCloudSegment);
//	pcl::PointCloud<PointT> Final;
//	icp.align(Final);
//	std::cout << "is Converged: " << icp.hasConverged() << "; Score = " << icp.getFitnessScore() << std::endl;
//	// TODO: use icp.getFitnessScore() to add random disturbance
//	Eigen::Matrix<float, 4, 4> Mcamera = icp.getFinalTransformation();
//	Eigen::Matrix<double, 4, 4> Mworld = worldTcamera.matrix() * Mcamera.cast<double>() * worldTcamera.inverse().matrix();
////	std::cout << Mworld << std::endl;
//
//	RigidTF rtf;
//	rtf.tf = Mworld;
//	std::cerr << "ICP TF: \n";
//	std::cerr << rtf.tf.matrix() << std::endl;
//	return rtf;
//}

ObjectActionModel::TimePoseLookup
ObjectActionModel::icpManifoldSequentialSampler(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer,
                                                /*const std::map<ros::Time, cv::Mat>& masks,*/ const mps::Pose& worldTcamera)
{
	assert(steps.size() > 1);
	icpRigidTF.tf = mps::Pose::Identity();

	pcl::IterativeClosestPoint<PointT, PointT> icp;
	TimePoseLookup timeToMotionLookup;

	//// Construct PointCloud Segments
	pcl::PointCloud<PointT>::Ptr firstCloudSegment;
	pcl::PointCloud<PointT>::Ptr secondCloudSegment;

	Eigen::Isometry3d cameraTobject0 = Eigen::Isometry3d::Identity();

	for (int t = 0; t < (int)steps.size()-1; ++t)
	{
		if (t == 0)
		{
			firstCloudSegment = make_PC_segment(buffer.rgb.at(steps[t])->image, buffer.depth.at(steps[t])->image,
			                                    buffer.cameraModel, masks.at(steps[t]));
			MPS_ASSERT(!firstCloudSegment->empty());

			if (scenario->shouldVisualize("poseArray"))
			{
				Eigen::Vector4f centroid;
				pcl::compute3DCentroid(*firstCloudSegment, centroid);
				cameraTobject0.translation() = centroid.head<3>().cast<double>();
			}
		}
		else
		{
			firstCloudSegment = secondCloudSegment;
		}
		secondCloudSegment = make_PC_segment(buffer.rgb.at(steps[t + 1])->image,
		                                                                buffer.depth.at(steps[t + 1])->image,
		                                                                buffer.cameraModel,
		                                                                masks.at(steps[t + 1]));

		// NB: sometimes the second segment will be empty: fall through to score-based error handling
		double error = std::numeric_limits<double>::max();
		if (!secondCloudSegment->empty())
		{
//			icp.setMaximumIterations(50);
//			icp.setTransformationEpsilon(1e-9);
			icp.setUseReciprocalCorrespondences(true);
			icp.setEuclideanFitnessEpsilon(0.01);
			icp.setInputSource(firstCloudSegment);
			icp.setInputTarget(secondCloudSegment);
			pcl::PointCloud<PointT> Final;
			icp.align(Final);
			error = icp.getFitnessScore();
			std::cerr << "is Converged: " << icp.hasConverged() << "; Score = " << error << std::endl;

			//// Visualization of ICP
			if (scenario->shouldVisualize("icp"))
			{
				static ros::NodeHandle pnh("~");
				static ros::Publisher pcPub1 = pnh.advertise<pcl::PointCloud<PointT>>("icp_source", 1, true);
				static ros::Publisher pcPub2 = pnh.advertise<pcl::PointCloud<PointT>>("icp_target", 1, true);
				static ros::Publisher pcPub3 = pnh.advertise<pcl::PointCloud<PointT>>("icp_registered", 1, true);

				firstCloudSegment->header.frame_id = buffer.cameraModel.tfFrame();
				pcl_conversions::toPCL(ros::Time::now(), firstCloudSegment->header.stamp);
				pcPub1.publish(*firstCloudSegment);

				secondCloudSegment->header.frame_id = buffer.cameraModel.tfFrame();
				pcl_conversions::toPCL(ros::Time::now(), secondCloudSegment->header.stamp);
				pcPub2.publish(*secondCloudSegment);

				Final.header.frame_id = buffer.cameraModel.tfFrame();
				pcl_conversions::toPCL(ros::Time::now(), Final.header.stamp);
				pcPub3.publish(Final);
				sleep(0.8);
			}
			if (scenario->shouldVisualize("poseArray"))
			{
				tf::Transform temp;

				// TODO: Get some global properties like table_surface, etc.
				for (int i = 0; i < 3; ++i)
				{
					mps::Pose worldTobject0 = worldTcamera * cameraTobject0;
					tf::transformEigenToTF(worldTobject0, temp);
					scenario->broadcaster->sendTransform(
						tf::StampedTransform(temp, ros::Time::now(), "table_surface", "object0"));
//					sleep(1);

					mps::Pose T(icp.getFinalTransformation().cast<double>());
					T = T.inverse(Eigen::Isometry);
					tf::transformEigenToTF(T, temp);
					scenario->broadcaster->sendTransform(
						tf::StampedTransform(temp, ros::Time::now(),
						                     "object" + std::to_string(t), "object" + std::to_string(t + 1)));
//					sleep(1);

					T = mps::Pose(icpRigidTF.tf);
					tf::transformEigenToTF(T, temp);
					scenario->broadcaster->sendTransform(
						tf::StampedTransform(temp, ros::Time::now(), "table_surface" , "object_track"));
					usleep(100000);
				}
			}
		}

		if (error > 0.001) //// invalid
		{
			std::cerr << "This is an invalid ICP, use previous TF." << std::endl;
			if (t == 0)
			{
				timeToMotionLookup.emplace(std::make_pair(steps[t], steps[t + 1]), mps::Pose::Identity());
			}
			else
			{
				const auto& prevMworld = timeToMotionLookup.at(std::make_pair(steps[t-1], steps[t]));
				timeToMotionLookup.emplace(std::make_pair(steps[t], steps[t + 1]), prevMworld);
				icpRigidTF.tf = prevMworld.cast<double>() * icpRigidTF.tf;
			}
		}
		else
		{
			auto temp = icp.getFinalTransformation();
			mps::Pose Mcamera;
			Mcamera.linear() = temp.topLeftCorner<3, 3>().cast<double>();
			Mcamera.translation() = temp.topRightCorner<3, 1>().cast<double>();
			mps::Pose Mworld = worldTcamera * Mcamera * worldTcamera.inverse(Eigen::Isometry);
			timeToMotionLookup.emplace(std::make_pair(steps[t], steps[t + 1]), Mworld);
			icpRigidTF.tf = Mworld * icpRigidTF.tf;
		}
	}
	std::cerr << "icp total TF:\n";
	std::cerr << icpRigidTF.tf.matrix() << std::endl;

	return timeToMotionLookup;
}

bool ObjectActionModel::clusterRigidBodyTransformation(const std::map<std::pair<ros::Time, ros::Time>, Tracker::Flow3D>& flows3camera, const mps::Pose& worldTcamera)
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
				rbt.tf = convertTFformat(linear, angular);
				rbt.numInliers = count;
				siftRigidTFs.push_back(rbt);
			}
		}
	}
	return isClusterExist;
}

//bool ObjectActionModel::sampleAction(const SensorHistoryBuffer& buffer_out, SegmentationInfo& seg_out,
// std::unique_ptr<Tracker>& sparseTracker, std::unique_ptr<DenseTracker>& denseTracker, uint16_t label, mps_msgs::AABBox2d& bbox)
//{
//	cv::Mat startMask = cv::Mat::zeros(buffer_out.rgb.begin()->second->image.size(), CV_8UC1);
//	cv::Mat subwindow(startMask, seg_out.roi);
//	subwindow = label == seg_out.objectness_segmentation->image;
//	return sampleAction(buffer_out, startMask, seg_out.roi, sparseTracker, denseTracker, label, bbox);
//}

ObjectActionModel::ObjectActionModel(std::shared_ptr<const Scenario> scenario_, const SensorHistoryBuffer& buffer,
                                     const cv::Mat& firstFrameSeg, uint16_t label, mps_msgs::AABBox2d& bbox,
                                     std::unique_ptr<Tracker>& sparseTracker, std::unique_ptr<DenseTracker>& denseTracker, int n)
	: scenario(std::move(scenario_)),
	  numSamples(n),
	  jlinkageActionClient("cluster_flow", true)
{
	/////////////////////////////////////////////
	//// Construct tracking time steps
	/////////////////////////////////////////////
	std::vector<ros::Time> steps; // SiamMask tracks all these time steps except the first frame;
	for (auto iter = buffer.rgb.begin(); iter != buffer.rgb.end(); std::advance(iter, 3)) //// decide downsample rate
	{
		steps.push_back(iter->first);
	}
	std::vector<ros::Time> timeStartEnd;
	timeStartEnd.push_back(steps.front());
	timeStartEnd.push_back(steps.back());

	assert(firstFrameSeg.size() == buffer.rgb.at(steps.front())->image.size());
	assert(firstFrameSeg.size() == buffer.rgb.at(steps.back())->image.size());

	/////////////////////////////////////////////
	//// Look up worldTcamera
	/////////////////////////////////////////////
	const std::string tableFrame = "table_surface";
	tf::StampedTransform worldTcameraTF;
	geometry_msgs::TransformStamped wTc = buffer.tfs->lookupTransform(tableFrame, buffer.cameraModel.tfFrame(), ros::Time(0));
	tf::transformStampedMsgToTF(wTc, worldTcameraTF);
	mps::Pose worldTcamera;
	tf::transformTFToEigen(worldTcameraTF, worldTcamera);

	/////////////////////////////////////////////
	//// Tracking
	/////////////////////////////////////////////
//	std::cerr << "-------------------------------------------------------------------------------------" << std::endl;
	//// SiamMask tracking: construct masks
	masks.clear();
	bool denseTrackSuccess = denseTracker->track(steps, buffer, label, bbox, masks);
	if (!denseTrackSuccess)
	{
		ROS_ERROR_STREAM("Dense Track Failed!!!");
		throw std::runtime_error("Tracking call failed.");
	}
	if (masks.find(steps.back()) == masks.end())
	{
		ROS_ERROR_STREAM("Failed to estimate motion: SiamMask did not return masks for all frames! Return!");
		throw std::runtime_error("Tracking estimation failed.");
	}
	//// Fill in the first frame mask
	cv::Mat startMask = label == firstFrameSeg;

	masks.insert(masks.begin(), {steps.front(), startMask});

	/////////////////////////////////////////////
	//// Estimate motion using SiamMask
	/////////////////////////////////////////////
	Eigen::Vector3d roughMotion_w = sampleActionFromMask(masks.at(steps.front()), buffer.depth.at(steps.front())->image,
	                                                     masks.at(steps.back()), buffer.depth.at(steps.back())->image,
	                                                     buffer.cameraModel, worldTcamera);
	std::cerr << "Rough Motion from SiamMask: " << roughMotion_w.x() << " " << roughMotion_w.y() << " " << roughMotion_w.z() << std::endl;
	Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
	guess.topRightCorner<3, 1>() = roughMotion_w.cast<float>();
//	Eigen::Matrix4f roughMotion_c = worldTcamera.inverse().matrix().cast<float>() * guess * worldTcamera.matrix().cast<float>();

	/////////////////////////////////////////////
	//// ICP check & store
	/////////////////////////////////////////////
	ObjectActionModel::TimePoseLookup timeToMotionLookup = icpManifoldSequentialSampler(steps, buffer, worldTcamera);

	pcl::PointCloud<PointT>::Ptr initCloudSegment = make_PC_segment(buffer.rgb.at(steps[0])->image, buffer.depth.at(steps[0])->image,
	                                                                buffer.cameraModel, masks.at(steps[0]));
	assert(!initCloudSegment->empty());
	pcl::PointCloud<PointT>::Ptr lastCloudSegment = make_PC_segment(buffer.rgb.at(steps.back())->image,
	                                                                buffer.depth.at(steps.back())->image,
	                                                                buffer.cameraModel,
	                                                                masks.at(steps.back()));
	assert(!lastCloudSegment->empty());
//	if (!isSiamMaskValidICPbased(initCloudSegment, lastCloudSegment, worldTcamera, 0.002,
//	                             buffer.cameraModel.tfFrame(), true, icpRigidTF.tf.matrix().cast<float>())) // TODO: decide the value of threshold
	if (!isSiamMaskValidICPbased(initCloudSegment, lastCloudSegment, worldTcamera, 0.002,
	                             buffer.cameraModel.tfFrame()))
	{
		//// Generate reasonable disturbance in ParticleFilter together with failed situations
		std::cerr << "Although ICP tells us SiamMask isn't reasonable, we still use it for now." << std::endl;
	}

	//// SIFT
	sparseTracker->track(timeStartEnd, buffer, masks);

	/////////////////////////////////////////////
	//// send request to jlinkage server & sample object motions
	/////////////////////////////////////////////
	if (clusterRigidBodyTransformation(sparseTracker->flows3, worldTcamera))
	{
		std::cerr << "SIFT is usable" << std::endl;
		weightedSampleSIFT(1);
		double pSIFT = std::min(0.9, std::max(0.1, 0.1 * (double)actionSamples[0].numInliers - 0.25));
		double p = rand()/(double)RAND_MAX;
		if (p > pSIFT)
		{
			std::cerr << "But we use ICP" << std::endl;
			actionSamples[0] = icpRigidTF;
		}
	}
	else
	{
		std::cerr << "USE ICP" << std::endl;
		for (int i = 0; i < numSamples; ++i)
		{
			actionSamples.push_back(icpRigidTF);
		}
	}
	for (auto& as : actionSamples)
	{
		std::cerr << "Final action sample:\n";
		std::cerr << as.tf.matrix() << std::endl;
	}
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
                                                const mps::Pose& worldTcamera, const double& scoreThreshold, const std::string& frame_id,
                                                const bool& useGuess, const Eigen::Matrix4f& guessCamera)
{
	assert(!initCloudSegment->empty());
	assert(!lastCloudSegment->empty());

	bool isValid = true;
	//// ICP
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaximumIterations(20);
//	icp.setTransformationEpsilon(1e-9);
	icp.setEuclideanFitnessEpsilon(0.1);
	icp.setInputSource(initCloudSegment);
	icp.setInputTarget(lastCloudSegment);
	pcl::PointCloud<PointT> Final;
	if (useGuess) { icp.align(Final, guessCamera); }
	else { icp.align(Final); }
	double score = icp.getFitnessScore();
	std::cerr << "ICP is Converged: " << icp.hasConverged() << "; Score = " << score << std::endl;

	//// Visualization of ICP
	if (scenario->shouldVisualize("icp"))
	{
		static ros::NodeHandle pnh("~");
		static ros::Publisher pcPub1 = pnh.advertise<pcl::PointCloud<PointT>>("icp_source", 1, true);
		static ros::Publisher pcPub2 = pnh.advertise<pcl::PointCloud<PointT>>("icp_target", 1, true);
		static ros::Publisher pcPub3 = pnh.advertise<pcl::PointCloud<PointT>>("icp_registered", 1, true);

		initCloudSegment->header.frame_id = frame_id;
		pcl_conversions::toPCL(ros::Time::now(), initCloudSegment->header.stamp);
		pcPub1.publish(*initCloudSegment);

		lastCloudSegment->header.frame_id = frame_id;
		pcl_conversions::toPCL(ros::Time::now(), lastCloudSegment->header.stamp);
		pcPub2.publish(*lastCloudSegment);

		Final.header.frame_id = frame_id;
		pcl_conversions::toPCL(ros::Time::now(), Final.header.stamp);
		pcPub3.publish(Final);
		sleep(3);
	}

	if (score > scoreThreshold)
	{
		ROS_ERROR_STREAM("SiamMask result doesn't make sense according to ICP.");
		isValid = false;
	}

	Eigen::Matrix<float, 4, 4> Mcamera = icp.getFinalTransformation();
	Eigen::Matrix<double, 4, 4> Mworld = worldTcamera.matrix() * Mcamera.cast<double>() * worldTcamera.inverse().matrix();

	std::cerr << "ICP TF: \n";
	std::cerr << Mworld << std::endl;

	icpRigidTF.tf = Mworld;
	return isValid;
}

Eigen::Matrix4d convertTFformat(Eigen::Vector3d linear, Eigen::Vector3d angular)
{
	Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();

	double theta = angular.norm();
	Eigen::Vector3d e;
	if (theta == 0) { e = {0, 0, 0}; }
	else { e = angular / theta; }

	Eigen::AngleAxisd rot(theta, e);
	tf.topLeftCorner<3, 3>() = rot.matrix();
	tf.topRightCorner<3, 1>() = linear;

	return tf;
}

std::shared_ptr<octomap::OcTree>
moveOcTree(const octomap::OcTree* octree, const RigidTF& action)
{
	std::shared_ptr<octomap::OcTree> resOcTree = std::make_shared<octomap::OcTree>(octree->getResolution());
	for (auto node = octree->begin_leafs(); node != octree->end_leafs(); node++)
	{
		auto originalCoord = node.getCoordinate();
		Eigen::Vector4d oCoord(originalCoord.x(), originalCoord.y(), originalCoord.z(), 1);
		Eigen::Vector4d newCoord;
		newCoord = action.tf * oCoord;
		resOcTree->updateNode(newCoord.x(), newCoord.y(), newCoord.z(), true);
		resOcTree->setNodeValue(newCoord.x(), newCoord.y(), newCoord.z(), node->getValue());
	}
	resOcTree->setOccupancyThres(octree->getOccupancyThres());
	return resOcTree;
}

}

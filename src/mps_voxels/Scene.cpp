//
// Created by arprice on 12/11/18.
//

#include "mps_voxels/Scene.h"

#include "mps_voxels/VictorManipulator.h"
#include "mps_voxels/LocalOctreeServer.h"
#include "mps_voxels/octree_utils.h"
#include "mps_voxels/pointcloud_utils.h"
#include "mps_voxels/image_utils.h"
#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/shape_utils.h"
#include "mps_voxels/project_point.hpp"
#include "mps_voxels/util/assert.h"

#include <tf_conversions/tf_eigen.h>

// Point s.cloud utilities
#include <depth_image_proc/depth_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

namespace mps
{

bool Scenario::loadManipulators(robot_model::RobotModelPtr& pModel)
{
	ros::NodeHandle nh;

	if (robotModel && robotModel.get() != pModel.get())
	{
		ROS_WARN("Overwriting Robot Model.");
	}
	robotModel = pModel;

	for (robot_model::JointModelGroup* jmg : pModel->getJointModelGroups())
	{
		if (jmg->isChain() && jmg->getSolverInstance())
		{
			jmg->getSolverInstance()->setSearchDiscretization(0.1); // 0.1 = ~5 degrees
			const auto& ees = jmg->getAttachedEndEffectorNames();
			ROS_INFO_STREAM("Loaded jmg '" << jmg->getName() << "' " << jmg->getSolverInstance()->getBaseFrame());
			for (const std::string& eeName : ees)
			{
				robot_model::JointModelGroup* ee = pModel->getEndEffector(eeName);
				ee->getJointRoots();
				const robot_model::JointModel* rootJoint = ee->getCommonRoot();
				rootJoint->getNonFixedDescendantJointModels();

				ROS_INFO_STREAM("\t-" << eeName << "\t" << ee->getFixedJointModels().size());
				for (const std::string& eeSubName : ee->getAttachedEndEffectorNames())
				{
					ROS_INFO_STREAM("\t\t-" << eeSubName << "\t" << pModel->getEndEffector(eeSubName)->getFixedJointModels().size());

					auto manip = std::make_shared<VictorManipulator>(nh, pModel, jmg, ee, pModel->getEndEffector(eeSubName)->getLinkModelNames().front());
					this->manipulators.emplace_back(manip);
					if (!manip->configureHardware())
					{
						ROS_FATAL_STREAM("Failed to setup hardware '" << manip->arm->getName() << "'");
//						throw std::runtime_error("Hardware config failure.");
					}
					for (const std::string& jName : manip->arm->getJointModelNames())
					{
						this->jointToManipulator[jName] = manip;
					}
					for (const std::string& jName : manip->gripper->getJointModelNames())
					{
						this->jointToManipulator[jName] = manip;
					}
				}
			}
		}
//		else
//		{
//			ROS_INFO_STREAM("Did not load jmg '" << jmg->getName() << "'");
//			ROS_INFO_STREAM("\t is " << (jmg->isEndEffector()?"":"not ") << "end-effector.");
//		}
	}

	return true;
}

std::shared_ptr<Scenario> scenarioFactory(ros::NodeHandle& nh, ros::NodeHandle& pnh, robot_model::RobotModelPtr& robotModel)
{
	auto experiment = std::make_shared<Experiment>(nh, pnh);

	setIfMissing(pnh, "frame_id", "table_surface");
	setIfMissing(pnh, "resolution", 0.010);
	setIfMissing(pnh, "latch", false);
	setIfMissing(pnh, "filter_ground", false);
	setIfMissing(pnh, "filter_speckles", true);
	setIfMissing(pnh, "publish_free_space", false);

	setIfMissing(pnh, "roi/min/x", -0.4f);
	setIfMissing(pnh, "roi/min/y", -0.6f);
	setIfMissing(pnh, "roi/min/z", -0.020f);
	setIfMissing(pnh, "roi/max/x",  0.4f);
	setIfMissing(pnh, "roi/max/y",  0.6f);
	setIfMissing(pnh, "roi/max/z",  0.5f);

	setIfMissing(pnh, "sensor_model/max_range", 8.0);
	setIfMissing(pnh, "planning_samples", 25);
	setIfMissing(pnh, "planning_time", 60.0);
	setIfMissing(pnh, "track_color", "green");
	setIfMissing(pnh, "use_memory", true);
	setIfMissing(pnh, "use_completion", "optional");
	setIfMissing(pnh, "visualize", std::vector<std::string>{"particles", "icp"});

	bool gotParam;

	// Get shape completion requirements
	FEATURE_AVAILABILITY useShapeCompletion;
	std::string use_completion;
	gotParam = pnh.getParam("use_completion", use_completion); MPS_ASSERT(gotParam);
	std::transform(use_completion.begin(), use_completion.end(), use_completion.begin(), ::tolower);
	if (use_completion == "forbidden")
	{
		useShapeCompletion = FEATURE_AVAILABILITY::FORBIDDEN;
	}
	else if (use_completion == "optional")
	{
		useShapeCompletion = FEATURE_AVAILABILITY::OPTIONAL;
	}
	else if (use_completion == "required")
	{
		useShapeCompletion = FEATURE_AVAILABILITY::REQUIRED;
	}
	else
	{
		ROS_ERROR_STREAM("Color must be one of {forbidden, optional, required}.");
		throw std::runtime_error("Invalid completion option.");
	}

	bool use_memory;
	gotParam = pnh.getParam("use_memory", use_memory); MPS_ASSERT(gotParam);

	std::shared_ptr<Scenario> scenario = std::make_shared<Scenario>(use_memory, useShapeCompletion);
	scenario->loadManipulators(robotModel);

	scenario->experiment = experiment;

	scenario->listener = std::make_shared<tf2_ros::TransformListener>(scenario->transformBuffer);//listener.get();
	scenario->transformBuffer.setUsingDedicatedThread(true);

	scenario->minExtent = Eigen::Vector4f::Ones();
	scenario->maxExtent = Eigen::Vector4f::Ones();


	pnh.getParam("roi/min/x", scenario->minExtent.x());
	pnh.getParam("roi/min/y", scenario->minExtent.y());
	pnh.getParam("roi/min/z", scenario->minExtent.z());
	pnh.getParam("roi/max/x", scenario->maxExtent.x());
	pnh.getParam("roi/max/y", scenario->maxExtent.y());
	pnh.getParam("roi/max/z", scenario->maxExtent.z());
	pnh.getParam("frame_id", scenario->worldFrame);

	auto homeState = std::make_shared<robot_state::RobotState>(robotModel);
	homeState->setToDefaultValues();
	scenario->homeState = homeState;

	scenario->mapServer = std::make_shared<LocalOctreeServer>(pnh);


	// Sanity checks
	for (int i = 0; i < 3; ++i) { MPS_ASSERT(scenario->minExtent[i] < scenario->maxExtent[i]); }

	return scenario;
}

bool Scene::convertImages(const sensor_msgs::ImageConstPtr& rgb_msg,
                          const sensor_msgs::ImageConstPtr& depth_msg,
                          const sensor_msgs::CameraInfo& cam_msg)
{
	try
	{
		cv_rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return false;
	}

	// TODO: Right now everything assumes the depth image is in uint16_t, but it probably makes more sense to normalize
	//  to float, as that's what's needed in imagesToCloud() anyway.
	if ("16UC1" == depth_msg->encoding)
	{
		try
		{
			cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1); // MONO16?
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return false;
		}
	}
	else if ("32FC1" == depth_msg->encoding)
	{
		try
		{
			cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return false;
		}

		cv::Mat convertedDepthImg(cv_depth_ptr->image.size(), CV_16UC1);

		const int V = cv_depth_ptr->image.size().height;
		const int U = cv_depth_ptr->image.size().width;

		#pragma omp parallel for
		for (int v = 0; v < V; ++v)
		{
			for (int u = 0; u < U; ++u)
			{
				convertedDepthImg.at<uint16_t>(v, u)
					= depth_image_proc::DepthTraits<uint16_t>::fromMeters(cv_depth_ptr->image.at<float>(v, u));
			}
		}

		cv_depth_ptr->encoding = "16UC1";
		cv_depth_ptr->image = convertedDepthImg;
	}

	cameraModel.fromCameraInfo(cam_msg);

	return true;
}

bool SceneProcessor::loadAndFilterScene(Scene& s, const tf2_ros::Buffer& transformBuffer)
{
	if (!ros::ok()) { return false; }

	// NB: We do this every loop because we shrink the box during the crop/filter process
	s.minExtent = s.scenario->minExtent;//.head<3>().cast<double>();
	s.maxExtent = s.scenario->maxExtent;//.head<3>().cast<double>();

	s.cameraFrame = s.cameraModel.tfFrame();


	const ros::Time queryTime = transformBuffer.isUsingDedicatedThread() ? s.getTime() : ros::Time(0);
	const ros::Duration timeout = ros::Duration(5.0);
	std::string tfError;
	{
		bool canTransform = transformBuffer.isUsingDedicatedThread() ?
		                    transformBuffer.canTransform(s.scenario->worldFrame, s.cameraModel.tfFrame(), queryTime, timeout, &tfError) :
		                    transformBuffer.canTransform(s.scenario->worldFrame, s.cameraModel.tfFrame(), queryTime, &tfError);

		if (!canTransform) // ros::Duration(5.0)
		{
			ROS_WARN_STREAM("Failed to look up transform between '" << s.scenario->worldFrame << "' and '"
			                                                        << s.cameraModel.tfFrame() << "' with error '"
			                                                        << tfError << "'.");
			return false;
		}
	}

	// Get Octree
	octomap::OcTree* octree = s.scenario->mapServer->getOctree();
	MPS_ASSERT(octree);
	s.sceneOctree = octree;

	if (!s.scenario->useMemory)
	{
		octree->clear();
	}

	setBBox(s.minExtent, s.maxExtent, octree);

	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	{
		tf::StampedTransform cameraFrameInTableCoordinates;
		const auto temp = transformBuffer.lookupTransform(s.cameraFrame, s.scenario->worldFrame, queryTime);
		tf::transformStampedMsgToTF(temp, cameraFrameInTableCoordinates);
		tf::transformTFToEigen(cameraFrameInTableCoordinates.inverse(), s.worldTcamera);
	}

	octomap::point3d cameraOrigin((float)s.worldTcamera.translation().x(),
	                              (float)s.worldTcamera.translation().y(),
	                              (float)s.worldTcamera.translation().z());
	decayMemory(octree, cameraOrigin);
	const double missProb = 0.05;
	octree->setProbMiss(missProb);
	octree->setProbHit(1.0-missProb);

	if (cv::countNonZero(s.cv_depth_ptr->image) < 1)
	{
		ROS_ERROR("Depth image is all zeros!");
		return false;
	}

	s.cloud = imagesToCloud(s.cv_rgb_ptr->image, s.cv_depth_ptr->image, s.cameraModel);
	if (s.cloud->empty())
	{
		ROS_ERROR("Initial cloud contains no points.");
		return false;
	}

	/////////////////////////////////////////////////////////////////
	// Crop to bounding box
	/////////////////////////////////////////////////////////////////
	s.cropped_cloud = cropInCameraFrame(s.cloud, s.minExtent, s.maxExtent, s.worldTcamera);
	if (s.cropped_cloud->empty())
	{
		ROS_WARN("Filtered cloud contains no points.");
		return false;
	}
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	if (!ros::Time::isSimTime()) // TODO: Distinguish sim time from bag time
		s.cropped_cloud = filterInCameraFrame(s.cropped_cloud);
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	s.scenario->mapServer->insertCloud(s.cropped_cloud, s.worldTcamera);
//	s.scenario->mapServer->insertCloud(s.cloud, s.worldTcamera);
//	octree->prune();
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	pcl::PointCloud<PointT>::Ptr downsampled_cloud(new pcl::PointCloud<PointT>());
	pcl::VoxelGrid<PointT> voxelFilter;
	voxelFilter.setInputCloud(s.cropped_cloud);
	float resolution = (float)octree->getResolution()/4.0f;
	voxelFilter.setLeafSize(resolution, resolution, resolution);
	voxelFilter.filter(*downsampled_cloud);


	/////////////////////////////////////////////////////////////////
	// Apply prior motion models
	/////////////////////////////////////////////////////////////////

	// Update from robot state + TF
	for (const auto& model : s.selfModels)
	{
		bool canTransform = transformBuffer.isUsingDedicatedThread() ?
		                    transformBuffer.canTransform(model.first, s.scenario->worldFrame, queryTime, timeout, &tfError) :
		                    transformBuffer.canTransform(model.first, s.scenario->worldFrame, queryTime, &tfError);
		if (canTransform)
		{
			tf::StampedTransform stf;
			const auto temp = transformBuffer.lookupTransform(model.first, s.scenario->worldFrame, queryTime);
			tf::transformStampedMsgToTF(temp, stf);
			tf::transformTFToEigen(stf, model.second->localTglobal);
		}
		else if (std::find(s.scenario->robotModel->getLinkModelNames().begin(), s.scenario->robotModel->getLinkModelNames().end(), model.first) != s.scenario->robotModel->getLinkModelNames().end())
		{
			ROS_ERROR_STREAM("Unable to compute transform from '" << s.scenario->worldFrame << "' to '" << model.first << "'.");
			return false;
		}

		// Compute bounding spheres
		model.second->updateMembershipStructures();
	}


	/////////////////////////////////////////////////////////////////
	// Self-filter Point s.cloud
	/////////////////////////////////////////////////////////////////

	SensorModel sensor{s.worldTcamera.translation()};
	pcl::PointCloud<PointT>::Ptr non_self_cloud(new pcl::PointCloud<PointT>());
	{
		if (!ros::ok()) { return false; }
		std::vector<int> assignments(downsampled_cloud->size(), -1);
		const int nPts = static_cast<int>(downsampled_cloud->size());
		#pragma omp parallel for //schedule(dynamic)
		for (int i = 0; i < nPts; ++i)
		{
			Eigen::Vector3d pt = (s.worldTcamera.cast<float>()*downsampled_cloud->points[i].getVector3fMap()).cast<double>();
			int m = 0;
			double maxL = std::numeric_limits<double>::lowest();
			for (const auto& model : s.selfModels)
			{
				double L = model.second->membershipLikelihood(pt, sensor);
				if (L > 0.0 && L > maxL)
				{
					maxL = L;
					assignments[i] = m;
					break;
				}
				++m;
			}
		}

		pcl::PointIndices::Ptr self_assigned (new pcl::PointIndices);
		for (int i = 0; i < nPts; ++i)
		{
			if (assignments[i] >= 0)
			{
				self_assigned->indices.emplace_back(i);
			}
		}
		pcl::ExtractIndices<PointT> extractor;
		extractor.setInputCloud(downsampled_cloud);
		extractor.setIndices(self_assigned);
		extractor.setNegative(true);
		extractor.filter(*non_self_cloud);


//		downsampled_cloud->points.erase(std::remove_if(downsampled_cloud->points.begin(), downsampled_cloud->points.end(),
//		                                           [&](const PointT& p){ return assignments[&p - &*downsampled_cloud->points.begin()] >= 0;}),
//		                            downsampled_cloud->points.end());
	}
	if (downsampled_cloud->empty())
	{
		ROS_WARN_STREAM("All points were self-filtered!");
		return false;
	}


	/////////////////////////////////////////////////////////////////
	/// Self-filter Occupancy Map
	/////////////////////////////////////////////////////////////////

	{
		std::vector<octomap::OcTree::iterator> iters;
		for (octomap::OcTree::iterator it = octree->begin(octree->getTreeDepth()),
			     end = octree->end(); it != end; ++it)
		{
			if (octree->isNodeOccupied(*it))
			{
				iters.push_back(it);
			}
		}

		std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

		#pragma omp parallel for schedule(dynamic)
		for (size_t i = 0; i < iters.size(); ++i)
		{
			const auto& it = iters[i];
			Eigen::Vector3d pt(it.getX(), it.getY(), it.getZ());
//			double size = octree->getNodeSize(it.getDepth());
			int m = 0;
			for (const auto& model : s.selfModels)
			{
//				Eigen::Vector3d p = model.second->localTglobal.inverse() * model.second->boundingSphere.center;
//				pt -= ((p - pt).normalized() * size);
				double L = model.second->membershipLikelihood(pt, sensor);
				if (L > 0.0)
				{
					#pragma omp critical
					{
						octree->deleteNode(it.getKey());
					}
					break;
				}
				++m;
			}
		}
		octree->updateInnerOccupancy();

		std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

		iters.clear();
		for (octomap::OcTree::iterator it = octree->begin(octree->getTreeDepth()),
			     end = octree->end(); it != end; ++it)
		{
			if (octree->isNodeOccupied(*it))
			{
				iters.push_back(it);
			}
		}

		std::vector<octomap::OcTreeKey> toDelete;

		// Delete speckle nodes
		#pragma omp parallel for
		for (size_t i = 0; i < iters.size(); ++i)
		{
			const auto& it = iters[i];
			const octomap::OcTreeKey& key = it.getKey();
			if (isSpeckleNode(key, octree))
			{
				#pragma omp critical
				{
					toDelete.push_back(key);
				}
			}
		}

		for (const octomap::OcTreeKey& key : toDelete) { octree->deleteNode(key); }
		octree->updateInnerOccupancy();
	}
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	/////////////////////////////////////////////////////////////////
	/// Generate an ROI from the cropped, filtered s.cloud
	/////////////////////////////////////////////////////////////////
	s.pile_cloud = filterPlane(non_self_cloud, 0.02, s.worldTcamera.linear().col(2).cast<float>());
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
//	s.pile_cloud = filterSmallClusters(s.pile_cloud, 1000, 0.005); // sqrt(s.cloud->size())/50
//	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	s.pile_cloud = filterOutliers(s.pile_cloud, 100, 2.0);
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	if (s.pile_cloud->empty())
	{
		ROS_ERROR_STREAM("No points in pile s.cloud.");
		return false;
	}
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;


	{
		if (!ros::ok()) { return false; }
//		static pcl::visualization::CloudViewer viewer("PilePoints");
//		viewer.showCloud(s.pile_cloud);
//		while (!viewer.wasStopped ()){}

		std::vector<cv::Point> pile_points;
		pile_points.reserve(s.pile_cloud->size());

		for (const PointT& pt : *s.pile_cloud)
		{
			// NB: s.pile_cloud is in the camera frame
			Eigen::Vector3d p = pt.getVector3fMap().cast<double>();
			cv::Point3d worldPt_camera(p.x(), p.y(), p.z());
			pile_points.push_back(s.cameraModel.project3dToPixel(worldPt_camera));
		}

		s.roi = cv::boundingRect(pile_points);
		const int buffer = 50; //25
		if (buffer < s.roi.x) { s.roi.x -= buffer; s.roi.width += buffer; }
		if (s.roi.x+s.roi.width < static_cast<int>(s.cameraModel.cameraInfo().width)-buffer) { s.roi.width += buffer; }
		if (buffer < s.roi.y) { s.roi.y -= buffer; s.roi.height += buffer; }
		if (s.roi.y+s.roi.height < static_cast<int>(s.cameraModel.cameraInfo().height)-buffer) { s.roi.height += buffer; }

		cv::Mat rgb_cropped(s.cv_rgb_ptr->image, s.roi);
		cv::Mat depth_cropped(s.cv_depth_ptr->image, s.roi);
		s.cv_rgb_cropped = cv_bridge::CvImage(s.cv_rgb_ptr->header, s.cv_rgb_ptr->encoding, rgb_cropped);
		s.cv_depth_cropped = cv_bridge::CvImage(s.cv_depth_ptr->header, s.cv_depth_ptr->encoding, depth_cropped);
		s.cam_msg_cropped = s.cameraModel.cameraInfo();
		s.cam_msg_cropped.roi.width = s.roi.width;
		s.cam_msg_cropped.roi.height = s.roi.height;
		s.cam_msg_cropped.roi.x_offset = s.roi.x;
		s.cam_msg_cropped.roi.y_offset = s.roi.y;

////		s.cv_depth_cropped->image
//		cv::Mat cleaned_depth(s.cv_depth_cropped.image.size(), CV_16UC1);
//
//		cv::rgbd::DepthCleaner dc(CV_16U, 7);
//		dc(s.cv_depth_cropped.image, cleaned_depth);
//
////		std::cerr << "type: " << cleaned_depth.type() << std::endl;
////		cv::imshow("raw", s.cv_depth_cropped.image*10);
////		cv::imshow("cleaned", cleaned_depth*10);
////		cv::waitKey(0);
//		s.cv_depth_cropped.image = cleaned_depth;

		// Force out-of-bounds points to plane
		Pose cameraTworld = s.worldTcamera.inverse(Eigen::Isometry);

		#pragma omp parallel for
		for (int v = s.roi.y; v < s.roi.y+s.roi.height; ++v)
		{
			for (int u = s.roi.x; u < s.roi.x+s.roi.width; ++u)
			{
//				int idx = v * (int)s.cameraModel.cameraInfo().width + u;

				// Compute the ray-plane intersection
				Eigen::Vector3d p0 = cameraTworld.translation(); ///< A point in the table plane
				Eigen::Vector3d pn = cameraTworld.linear().col(2); ///< The table plane normal vector
				Eigen::Vector3d l0 = Eigen::Vector3d::Zero(); ///< The ray origin (= camera origin)
				Eigen::Vector3d ln = toPoint3D<Eigen::Vector3d>(u, v, 1.0,
				                                                s.cameraModel).normalized(); ///< The ray direction
				double t = (p0-l0).dot(pn)/(ln.dot(pn)); ///< Distance along ray of intersection
				Eigen::Vector3d ptIntersection = l0+(ln*t); ///< Point where ray intersects plane

				Eigen::Vector3d ptIntWorld = s.worldTcamera * ptIntersection;

//				auto color = s.cv_rgb_ptr->image.at<cv::Vec3b>(v, u);
				auto dVal = s.cv_depth_ptr->image.at<uint16_t>(v, u);
				if (0 == dVal
				    && s.minExtent.x() < ptIntWorld.x() && ptIntWorld.x() < s.maxExtent.x()
				    && s.minExtent.y() < ptIntWorld.y() && ptIntWorld.y() < s.maxExtent.y()
				    && s.minExtent.z() < ptIntWorld.z() && ptIntWorld.z() < s.maxExtent.z())
				{
					continue; // Invalid point, but inside
				}

				float depthVal = depth_image_proc::DepthTraits<uint16_t>::toMeters(dVal); // if (depth1 > maxZ || depth1 < minZ) { continue; }
				Eigen::Vector3f ptWorld = s.worldTcamera.cast<float>() * toPoint3D<Eigen::Vector3f>(u, v, depthVal, s.cameraModel);

				if (0 == dVal
				    || ptWorld.x() < s.minExtent.x() || s.maxExtent.x() < ptWorld.x()
				    || ptWorld.y() < s.minExtent.y() || s.maxExtent.y() < ptWorld.y()
				    || ptWorld.z() < s.minExtent.z() || s.maxExtent.z() < ptWorld.z())
				{
					if (t>0.0 && t<10.0)
					{
						s.cv_depth_cropped.image.at<uint16_t>(v-s.roi.y, u-s.roi.x)
						    = depth_image_proc::DepthTraits<uint16_t>::fromMeters(ptIntersection.z());
					}
					else
					{
						s.cv_depth_cropped.image.at<uint16_t>(v-s.roi.y, u-s.roi.x) = 0;
					}
					s.cv_rgb_cropped.image.at<cv::Vec3b>(v-s.roi.y, u-s.roi.x) = cv::Vec3b(255, 255, 255);
				}
			}
		}

		std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	}

	return true;
}

bool SceneProcessor::callSegmentation(Scene& s)
{
	s.segInfo = s.scenario->segmentationClient->segment(s.cv_rgb_cropped, s.cv_depth_cropped, s.cam_msg_cropped);

	if (!s.segInfo)
	{
		ROS_WARN_STREAM("Segmentation Failed!");
		return false;
	}

	return true;
}

bool SceneProcessor::computeOcclusions(Scene& s)
{
	/////////////////////////////////////////////////////////////////
	// Compute Occlusions
	/////////////////////////////////////////////////////////////////
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	{
		// Recompute bounding box
		Eigen::Vector3f min, max;
		pcl::PointCloud<PointT>::Ptr transformed_cloud (new pcl::PointCloud<PointT>());
		// You can either apply transform_1 or transform_2; they are the same
		pcl::transformPointCloud (*s.cropped_cloud, *transformed_cloud, Eigen::Affine3d(s.worldTcamera));
		getAABB(*transformed_cloud, min, max);
		s.minExtent.head<3>() = min; s.maxExtent.head<3>() = max;
	}

	std::tie(s.occludedPts, s.occlusionTree) = getOcclusionsInFOV(s.sceneOctree, s.cameraModel, s.worldTcamera.inverse(Eigen::Isometry), s.minExtent.head<3>(), s.maxExtent.head<3>());

	if (s.occludedPts.empty())
	{
		ROS_ERROR_STREAM("Occluded points returned empty.");
		return false;
	}

	/////////////////////////////////////////////////////////////////
	// Filter parts of models
	/////////////////////////////////////////////////////////////////
	{
		std::vector<int> assignments(s.occludedPts.size(), -1);
		const int nPts = static_cast<int>(s.occludedPts.size());
		#pragma omp parallel for schedule(dynamic)
		for (int i = 0; i < nPts; ++i)
		{
			Eigen::Vector3d pt(s.occludedPts[i].x(), s.occludedPts[i].y(), s.occludedPts[i].z());
			int m = 0;
			double maxL = std::numeric_limits<double>::lowest();
			for (const auto& model : s.selfModels)
			{
				double L = model.second->membershipLikelihood(pt);
				if (L > 0.0 && L > maxL)
				{
					maxL = L;
					assignments[i] = m;
				}
				++m;
			}
		}

		for (int i = 0; i < nPts; ++i)
		{
			if (assignments[i] >= 0)
			{
				s.occlusionTree->deleteNode(s.occludedPts[i]);
				s.sceneOctree->deleteNode(s.occludedPts[i]);
			}
		}
		s.occlusionTree->updateInnerOccupancy();
		// Lambda function: capture by reference, get contained value, predicate function
		s.occludedPts.erase(std::remove_if(s.occludedPts.begin(), s.occludedPts.end(),
		                                   [&](const octomath::Vector3& p){ return assignments[&p - &*s.occludedPts.begin()] >= 0;}),
		                    s.occludedPts.end());
		MPS_ASSERT(!s.occludedPts.empty());
	}

	return true;
}


bool SceneProcessor::performSegmentation(const Scene& s, const std::shared_ptr<SegmentationInfo>& segHypo, OccupancyData& occupancy)
{
	MPS_ASSERT(s.roi.width == segHypo->objectness_segmentation->image.cols);
	MPS_ASSERT(s.roi.height == segHypo->objectness_segmentation->image.rows);
	occupancy.segments = segmentCloudsFromImage(s.pile_cloud, segHypo->objectness_segmentation->image, s.cameraModel, s.roi, &occupancy.labelToIndexLookup);

	if (occupancy.segments.empty())
	{
		ROS_WARN_STREAM("No clusters were detected!");
		return false;
	}

	for (const auto label : unique(segHypo->objectness_segmentation->image))
	{
		if (occupancy.labelToIndexLookup.left.find(label) == occupancy.labelToIndexLookup.left.end())
		{
			segHypo->objectness_segmentation->image.setTo(0, label == segHypo->objectness_segmentation->image);
		}
	}

	return true;
}

bool SceneProcessor::buildObjects(const Scene& s, OccupancyData& occupancy)
{
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	occupancy.objects.clear();

	bool completionIsAvailable = FEATURE_AVAILABILITY::FORBIDDEN != s.scenario->useShapeCompletion &&
		s.scenario->completionClient->completionClient.exists();
	if (FEATURE_AVAILABILITY::REQUIRED == s.scenario->useShapeCompletion && !completionIsAvailable)
	{
		throw std::runtime_error("Shape completion is set to REQUIRED, but the server is unavailable.");
	}

	for (const auto& seg : occupancy.segments)
	{
		if (!ros::ok()) { return false; }

		const pcl::PointCloud<PointT>::Ptr& segment_cloud = seg.second;
		assert(!segment_cloud->empty());

		// Compute bounding box
		Eigen::Vector3f min, max;
		getBoundingCube(*segment_cloud, min, max);
		double edge_length = max.x()-min.x(); // all edge of cube are same size
		double inflation = edge_length/5.0;
		min -= inflation*Eigen::Vector3f::Ones();
		max += inflation*Eigen::Vector3f::Ones();

		std::shared_ptr<octomap::OcTree> subtree(s.sceneOctree->create());
		subtree->setProbMiss(0.05);
		subtree->setProbHit(0.95);
		setBBox(s.minExtent, s.maxExtent, subtree.get());
		insertCloudInOctree(segment_cloud, s.worldTcamera, subtree.get());
		MPS_ASSERT(subtree->size()>0);

		// Delete speckle nodes
		std::vector<octomap::OcTree::iterator> iters;
		std::vector<octomap::OcTreeKey> toDelete;
		for (octomap::OcTree::iterator it = subtree->begin(subtree->getTreeDepth()),
			     end = subtree->end(); it!=end; ++it)
		{
			if (subtree->isNodeOccupied(*it))
			{
				iters.push_back(it);
			}
		}

		#pragma omp parallel for
		for (size_t i = 0; i<iters.size(); ++i)
		{
			const auto& it = iters[i];
			const octomap::OcTreeKey& key = it.getKey();
			if (isSpeckleNode(key, subtree.get()))
			{
				#pragma omp critical
				{
					toDelete.push_back(key);
				}
			}
		}

		for (const octomap::OcTreeKey& key : toDelete) { subtree->deleteNode(key); }
		subtree->updateInnerOccupancy();

		if (completionIsAvailable)
		{
			s.scenario->completionClient->completeShape(min, max, s.worldTcamera.cast<float>(), s.sceneOctree, subtree.get(),
			                                          false);
		}
		setBBox(Eigen::Vector3f(-2, -2, -2), Eigen::Vector3f(2, 2, 2), subtree.get());


		// Compute approximate shape
		auto approx = approximateShape(subtree.get());
		if (approx)
		{
			auto res = occupancy.objects.emplace(seg.first, std::make_unique<Object>(seg.first, subtree));
			res.first->second->segment = seg.second;
			res.first->second->approximation = approx;
			res.first->second->points = getPoints(subtree.get());
			getAABB(*res.first->second->approximation, res.first->second->minExtent, res.first->second->maxExtent);
		}

	}


	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	/////////////////////////////////////////////////////////////////
	// Compute the Most Occluding Segment
	/////////////////////////////////////////////////////////////////
	auto octree = s.sceneOctree;
	for (const auto& seg : occupancy.segments)
	{
		// PC may be rejected by object fitting
		if (occupancy.objects.find(seg.first) == occupancy.objects.end()) { continue; }

		const pcl::PointCloud<PointT>::Ptr& pc = seg.second;
		for (const PointT& pt : *pc)
		{
			Eigen::Vector3f worldPt = s.worldTcamera.cast<float>()*pt.getVector3fMap();
			octomap::point3d coord = octree->keyToCoord(octree->coordToKey(octomap::point3d(worldPt.x(), worldPt.y(), worldPt.z())));
			occupancy.surfaceCoordToObject.insert({coord, seg.first});
			occupancy.coordToObject.insert({coord, seg.first});
		}
	}

	octomap::point3d cameraOrigin((float)s.worldTcamera.translation().x(),
	                              (float)s.worldTcamera.translation().y(),
	                              (float)s.worldTcamera.translation().z());
	#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(s.occludedPts.size()); ++i)
	{
		const auto& pt_world = s.occludedPts[i];
		octomap::point3d collision;
		bool hit = octree->castRay(cameraOrigin, pt_world-cameraOrigin, collision);
		MPS_ASSERT(hit);
		collision = octree->keyToCoord(octree->coordToKey(collision)); // regularize

		const auto& iter = occupancy.coordToObject.find(collision);
		if (iter != occupancy.coordToObject.end())
		{
			#pragma omp critical
			{
				occupancy.occludedBySegmentCount[iter->second]++;
				occupancy.coordToObject.insert({pt_world, iter->second});
				occupancy.objects.at(iter->second)->shadow.push_back(pt_world);
			}
		}
	}

	return true;
}

bool SceneProcessor::removeAccountedForOcclusion(
	const Scenario* scenario,
	octomap::point3d_collection& occludedPts,
	std::shared_ptr<octomap::OcTree>& occlusionTree,
	const OccupancyData& occupancy)
{
	/////////////////////////////////////////////////////////////////
	// Filter completion results
	/////////////////////////////////////////////////////////////////
	{
		const int nPts = static_cast<int>(occludedPts.size());
		std::set<octomap::point3d, vector_less_than<3, octomap::point3d>> rejects;
//		#pragma omp parallel for schedule(dynamic)
//		#pragma omp parallel
		{
//			#pragma omp single
			{
				for (const auto& obj : occupancy.objects)
				{
//					#pragma omp task
					{
						const std::shared_ptr<octomap::OcTree>& segment = obj.second->occupancy;
						unsigned d = segment->getTreeDepth();
						for (int i = 0; i<nPts; ++i)
						{
							octomap::OcTreeNode* node = segment->search(occludedPts[i], d);
							if (node && node->getOccupancy()>0.5)
							{
//								#pragma omp critical
								{
									rejects.insert(occludedPts[i]);
									occlusionTree->setNodeValue(occludedPts[i], -std::numeric_limits<float>::infinity(),
									                            true);
								}
							}
						}
					}
				}
			}
		}
		std::cerr << "Rejected " << rejects.size() << " hidden voxels due to shape completion." << std::endl;
		bool completionIsAvailable = FEATURE_AVAILABILITY::FORBIDDEN != scenario->useShapeCompletion &&
		                             scenario->completionClient->completionClient.exists();
		if (completionIsAvailable && rejects.empty())
		{
			throw std::runtime_error("Shape completion did not contribute any points.");
		}
		occlusionTree->updateInnerOccupancy();
		occludedPts.erase(std::remove_if(occludedPts.begin(), occludedPts.end(),
		                                 [&](const octomath::Vector3& p){ return rejects.find(p) != rejects.end();}),
		                  occludedPts.end());
		MPS_ASSERT(!occludedPts.empty());

		return true;
	}

}

}
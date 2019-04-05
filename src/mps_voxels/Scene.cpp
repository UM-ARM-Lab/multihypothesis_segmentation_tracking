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
#include "mps_voxels/assert.h"

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

Object::Object(const ObjectIndex i, const std::shared_ptr<octomap::OcTree>& tree)
 : index(i), occupancy(tree)
{
}

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
			std::cerr << "Loaded jmg '" << jmg->getName() << "' " << jmg->getSolverInstance()->getBaseFrame() << std::endl;
			for (const std::string& eeName : ees)
			{
				robot_model::JointModelGroup* ee = pModel->getEndEffector(eeName);
				ee->getJointRoots();
				const robot_model::JointModel* rootJoint = ee->getCommonRoot();
				rootJoint->getNonFixedDescendantJointModels();

				std::cerr << "\t-" << eeName << "\t" << ee->getFixedJointModels().size() << std::endl;
				for (const std::string& eeSubName : ee->getAttachedEndEffectorNames())
				{
					std::cerr << "\t\t-" << eeSubName << "\t" << pModel->getEndEffector(eeSubName)->getFixedJointModels().size() << std::endl;

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
		else
		{
			std::cerr << "Did not load jmg '" << jmg->getName() << "'" << std::endl;
			std::cerr << "\t is " << (jmg->isEndEffector()?"":"not ") << "end-effector." << std::endl;
		}
	}

	return true;
}


collision_detection::WorldPtr
Scene::computeCollisionWorld()
{
	auto world = std::make_shared<collision_detection::World>();

	Pose robotTworld = worldTrobot.inverse(Eigen::Isometry);

	for (const auto& obstacle : scenario->staticObstacles)
	{
		world->addToObject(CLUTTER_NAME, obstacle.first, robotTworld * obstacle.second);
	}

	// Use aliasing shared_ptr constructor
//	world->addToObject(CLUTTER_NAME,
//	                   std::make_shared<shapes::OcTree>(std::shared_ptr<octomap::OcTree>(std::shared_ptr<octomap::OcTree>{}, sceneOctree)),
//	                   robotTworld);

	for (const auto& obj : objects)
	{
		const std::shared_ptr<octomap::OcTree>& segment = obj.second->occupancy;
		world->addToObject(std::to_string(obj.first.id), std::make_shared<shapes::OcTree>(segment), robotTworld);
	}

//	for (auto& approxSegment : approximateSegments)
//	{
//		world->addToObject(CLUTTER_NAME, approxSegment, robotTworld);
//	}

	collisionWorld = world;
	return world;
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

bool SceneProcessor::loadAndFilterScene(Scene& s)
{
	if (!ros::ok()) { return false; }

	s.worldFrame = scenario->mapServer->getWorldFrame();
	s.cameraFrame = s.cameraModel.tfFrame();

	if (!scenario->listener->waitForTransform(s.worldFrame, s.cameraModel.tfFrame(), s.getTime(), ros::Duration(5.0)))
	{
		ROS_WARN_STREAM("Failed to look up transform between '" << s.worldFrame << "' and '" << s.cameraModel.tfFrame() << "'.");
		return false;
	}

	// Get Octree
	octomap::OcTree* octree = scenario->mapServer->getOctree();
	MPS_ASSERT(octree);
	s.sceneOctree = octree;

	if (!useMemory)
	{
		octree->clear();
	}

	setBBox(s.minExtent, s.maxExtent, octree);

	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	tf::StampedTransform cameraFrameInTableCoordinates;
	scenario->listener->lookupTransform(s.cameraFrame, s.worldFrame, s.getTime(), cameraFrameInTableCoordinates);
	tf::transformTFToEigen(cameraFrameInTableCoordinates.inverse(), s.worldTcamera);

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

	s.cropped_cloud = filterInCameraFrame(s.cropped_cloud);
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	scenario->mapServer->insertCloud(s.cropped_cloud, s.worldTcamera);
//	scenario->mapServer->insertCloud(s.cloud, s.worldTcamera);
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
		if (scenario->listener->waitForTransform(model.first, s.worldFrame, s.getTime(), ros::Duration(5.0)))
		{
			tf::StampedTransform stf;
			scenario->listener->lookupTransform(model.first, s.worldFrame, s.getTime(), stf);
			tf::transformTFToEigen(stf, model.second->localTglobal);
		}
		else if (std::find(scenario->robotModel->getLinkModelNames().begin(), scenario->robotModel->getLinkModelNames().end(), model.first) != scenario->robotModel->getLinkModelNames().end())
		{
			ROS_ERROR_STREAM("Unable to compute transform from '" << s.worldFrame << "' to '" << model.first << "'.");
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


bool SceneProcessor::performSegmentation(Scene& s)
{
	s.segInfo = scenario->segmentationClient->segment(s.cv_rgb_cropped, s.cv_depth_cropped, s.cam_msg_cropped);

	if (!s.segInfo)
	{
		ROS_WARN_STREAM("Segmentation Failed!");
		return false;
	}

	s.segments = segmentCloudsFromImage(s.pile_cloud, s.segInfo->objectness_segmentation->image, s.cameraModel, s.roi, &s.labelToIndexLookup);

	if (s.segments.empty())
	{
		ROS_WARN_STREAM("No clusters were detected!");
		return false;
	}

	for (const auto label : unique(s.segInfo->objectness_segmentation->image))
	{
		if (s.labelToIndexLookup.find(label) == s.labelToIndexLookup.end())
		{
			s.segInfo->objectness_segmentation->image.setTo(0, label == s.segInfo->objectness_segmentation->image);
		}
	}

	return true;
}

bool SceneProcessor::buildObjects(Scene& s)
{
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	s.objects.clear();

	bool completionIsAvailable = scenario->completionClient->completionClient.exists()
		&& FEATURE_AVAILABILITY::FORBIDDEN != useShapeCompletion;
	if (FEATURE_AVAILABILITY::REQUIRED == useShapeCompletion && !completionIsAvailable)
	{
		throw std::runtime_error("Shape completion is set to REQUIRED, but the server is unavailable.");
	}

	for (const auto& seg : s.segments)
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
			scenario->completionClient->completeShape(min, max, s.worldTcamera.cast<float>(), s.sceneOctree, subtree.get(),
			                                          false);
		}
		setBBox(Eigen::Vector3f(-2, -2, -2), Eigen::Vector3f(2, 2, 2), subtree.get());


		// Compute approximate shape
		auto approx = approximateShape(subtree.get());
		if (approx)
		{
			auto res = s.objects.insert(std::make_pair(seg.first, std::make_unique<Object>(seg.first, subtree)));
			res.first->second->segment = seg.second;
			res.first->second->approximation = approx;
			res.first->second->points = getPoints(subtree.get());
		}

	}

	/////////////////////////////////////////////////////////////////
	// Compute Occlusions
	/////////////////////////////////////////////////////////////////
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	{
		// Recompute bounding box
		Eigen::Vector3f min, max;
		pcl::PointCloud<PointT>::Ptr transformed_cloud (new pcl::PointCloud<PointT>());
		// You can either apply transform_1 or transform_2; they are the same
		pcl::transformPointCloud (*s.cropped_cloud, *transformed_cloud, s.worldTcamera);
		getAABB(*transformed_cloud, min, max);
		s.minExtent.head<3>() = min; s.maxExtent.head<3>() = max;
	}

	std::tie(s.occludedPts, s.occlusionTree) = getOcclusionsInFOV(s.sceneOctree, s.cameraModel, s.worldTcamera.inverse(Eigen::Isometry), s.minExtent.head<3>(), s.maxExtent.head<3>());
//	scene->occluded_pts = occluded_pts;

	if (s.occludedPts.empty())
	{
		ROS_ERROR_STREAM("Occluded points returned empty.");
		return false;
	}

	/////////////////////////////////////////////////////////////////
	// Filter completion results
	/////////////////////////////////////////////////////////////////
	{
		const int nPts = static_cast<int>(s.occludedPts.size());
		std::set<octomap::point3d, vector_less_than<3, octomap::point3d>> rejects;
//		#pragma omp parallel for schedule(dynamic)
//		#pragma omp parallel
		{
//			#pragma omp single
			{
				for (const auto& obj : s.objects)
				{
//					#pragma omp task
					{
						const std::shared_ptr<octomap::OcTree>& segment = obj.second->occupancy;
						unsigned d = segment->getTreeDepth();
						for (int i = 0; i<nPts; ++i)
						{
							octomap::OcTreeNode* node = segment->search(s.occludedPts[i], d);
							if (node && node->getOccupancy()>0.5)
							{
//								#pragma omp critical
								{
									rejects.insert(s.occludedPts[i]);
									s.occlusionTree->setNodeValue(s.occludedPts[i], -std::numeric_limits<float>::infinity(),
									                            true);
								}
							}
						}
					}
				}
			}
		}
		std::cerr << "Rejected " << rejects.size() << " hidden voxels due to shape completion." << std::endl;
		if (completionIsAvailable && rejects.empty())
		{
			throw std::runtime_error("Shape completion did not contribute any points.");
		}
		s.occlusionTree->updateInnerOccupancy();
		s.occludedPts.erase(std::remove_if(s.occludedPts.begin(), s.occludedPts.end(),
		                                 [&](const octomath::Vector3& p){ return rejects.find(p) != rejects.end();}),
		                  s.occludedPts.end());
		MPS_ASSERT(!s.occludedPts.empty());
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
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	/////////////////////////////////////////////////////////////////
	// Compute the Most Occluding Segment
	/////////////////////////////////////////////////////////////////
	auto octree = s.sceneOctree;
	for (const auto& seg : s.segments)
	{
		// PC may be rejected by object fitting
		if (s.objects.find(seg.first) == s.objects.end()) { continue; }

		const pcl::PointCloud<PointT>::Ptr& pc = seg.second;
		for (const PointT& pt : *pc)
		{
			Eigen::Vector3f worldPt = s.worldTcamera.cast<float>()*pt.getVector3fMap();
			octomap::point3d coord = octree->keyToCoord(octree->coordToKey(octomap::point3d(worldPt.x(), worldPt.y(), worldPt.z())));
			s.surfaceCoordToObject.insert({coord, seg.first});
			s.coordToObject.insert({coord, seg.first});
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

		const auto& iter = s.coordToObject.find(collision);
		if (iter != s.coordToObject.end())
		{
			#pragma omp critical
			{
				s.occludedBySegmentCount[iter->second]++;
				s.coordToObject.insert({pt_world, iter->second});
				s.objects.at(iter->second)->shadow.push_back(pt_world);
			}
		}
	}

	return true;
}
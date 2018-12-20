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

#include <tf_conversions/tf_eigen.h>

// Point cloud utilities
#include <depth_image_proc/depth_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

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
						throw std::runtime_error("Hardware config failure.");
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

	try
	{
		cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1); // MONO16?
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return false;
	}

	cameraModel.fromCameraInfo(cam_msg);

	return true;
}

bool Scene::loadAndFilterScene()
{
	if (!ros::ok()) { return false; }
	if (!scenario->listener->waitForTransform(scenario->mapServer->getWorldFrame(), cameraModel.tfFrame(), ros::Time(0), ros::Duration(5.0)))
	{
		ROS_WARN_STREAM("Failed to look up transform between '" << scenario->mapServer->getWorldFrame() << "' and '" << cameraModel.tfFrame() << "'.");
		return false;
	}

	setBBox(minExtent, maxExtent, scenario->mapServer->getOctree());

	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	scenario->mapServer->getOctree()->clear();

	// Get Octree
	octomap::OcTree* octree = scenario->mapServer->getOctree();
	worldFrame = scenario->mapServer->getWorldFrame();
	cameraFrame = cameraModel.tfFrame();
	sceneOctree = octree;

	if (!octree)
	{
		ROS_ERROR("Octree generation failed.");
		return false;
	}

	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	tf::StampedTransform cameraFrameInTableCoordinates;
	scenario->listener->lookupTransform(cameraFrame, worldFrame, ros::Time(0), cameraFrameInTableCoordinates);
	tf::transformTFToEigen(cameraFrameInTableCoordinates.inverse(), worldTcamera);

	cloud = imagesToCloud(cv_rgb_ptr->image, cv_depth_ptr->image, cameraModel);

	/////////////////////////////////////////////////////////////////
	// Crop to bounding box
	/////////////////////////////////////////////////////////////////
	cropped_cloud = cropInCameraFrame(cloud, minExtent, maxExtent, worldTcamera);
	if (cropped_cloud->empty())
	{
		ROS_WARN("Filtered cloud contains no points.");
		return false;
	}
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	cropped_cloud = filterInCameraFrame(cropped_cloud);
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	scenario->mapServer->insertCloud(cropped_cloud, worldTcamera);
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	pcl::VoxelGrid<PointT> voxelFilter;
	voxelFilter.setInputCloud(cropped_cloud);
	double resolution = octree->getResolution();
	voxelFilter.setLeafSize(resolution/2.0, resolution/2.0, resolution/2.0);
	voxelFilter.filter(*cropped_cloud);


	/////////////////////////////////////////////////////////////////
	// Apply prior motion models
	/////////////////////////////////////////////////////////////////

	// Update from robot state + TF
	for (const auto& model : selfModels)
	{
		if (scenario->listener->waitForTransform(model.first, scenario->mapServer->getWorldFrame(), ros::Time(0), ros::Duration(5.0)))
		{
			tf::StampedTransform stf;
			scenario->listener->lookupTransform(model.first, scenario->mapServer->getWorldFrame(), ros::Time(0), stf);
			tf::transformTFToEigen(stf, model.second->localTglobal);
		}
		else if (std::find(scenario->robotModel->getLinkModelNames().begin(), scenario->robotModel->getLinkModelNames().end(), model.first) != scenario->robotModel->getLinkModelNames().end())
		{
			ROS_ERROR_STREAM("Unable to compute transform from '" << scenario->mapServer->getWorldFrame() << "' to '" << model.first << "'.");
			return false;
		}

		// Compute bounding spheres
		model.second->updateMembershipStructures();
	}


	/////////////////////////////////////////////////////////////////
	// Self-filter Point Cloud
	/////////////////////////////////////////////////////////////////

	SensorModel sensor{worldTcamera.translation()};
	{
		if (!ros::ok()) { return false; }
		std::vector<int> assignments(cropped_cloud->size(), -1);
		const int nPts = static_cast<int>(cropped_cloud->size());
		#pragma omp parallel for schedule(dynamic)
		for (int i = 0; i < nPts; ++i)
		{
			Eigen::Vector3d pt = (worldTcamera.cast<float>()*cropped_cloud->points[i].getVector3fMap()).cast<double>();
			int m = 0;
			double maxL = std::numeric_limits<double>::lowest();
			for (const auto& model : selfModels)
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

		cropped_cloud->points.erase(std::remove_if(cropped_cloud->points.begin(), cropped_cloud->points.end(),
		                                           [&](const PointT& p){ return assignments[&p - &*cropped_cloud->points.begin()] >= 0;}),
		                            cropped_cloud->points.end());
	}
	if (cropped_cloud->empty())
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
			for (const auto& model : selfModels)
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
	/// Generate an ROI from the cropped, filtered cloud
	/////////////////////////////////////////////////////////////////
	pile_cloud = filterPlane(cropped_cloud, 0.02);
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
//	pile_cloud = filterSmallClusters(pile_cloud, 1000, 0.005); // sqrt(cloud->size())/50
//	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	pile_cloud = filterOutliers(pile_cloud, 100);
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	if (pile_cloud->empty())
	{
		ROS_ERROR_STREAM("No points in pile cloud.");
		return false;
	}
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;


	{
		if (!ros::ok()) { return false; }
//		static pcl::visualization::CloudViewer viewer("PilePoints");
//		viewer.showCloud(pile_cloud);
//		while (!viewer.wasStopped ()){}

		std::vector<cv::Point> pile_points;
		pile_points.reserve(pile_cloud->size());

		for (const PointT& pt : *pile_cloud)
		{
			// NB: pile_cloud is in the camera frame
			Eigen::Vector3d p = pt.getVector3fMap().cast<double>();
			cv::Point3d worldPt_camera(p.x(), p.y(), p.z());
			pile_points.push_back(cameraModel.project3dToPixel(worldPt_camera));
		}

		roi = cv::boundingRect(pile_points);
		const int buffer = 25;
		if (buffer < roi.x) { roi.x -= buffer; roi.width += buffer; }
		if (roi.x+roi.width < static_cast<int>(cameraModel.cameraInfo().width)-buffer) { roi.width += buffer; }
		if (buffer < roi.y) { roi.y -= buffer; roi.height += buffer; }
		if (roi.y+roi.height < static_cast<int>(cameraModel.cameraInfo().height)-buffer) { roi.height += buffer; }

		cv::Mat rgb_cropped(cv_rgb_ptr->image, roi);
		cv::Mat depth_cropped(cv_depth_ptr->image, roi);
		cv_rgb_cropped = cv_bridge::CvImage(cv_rgb_ptr->header, cv_rgb_ptr->encoding, rgb_cropped);
		cv_depth_cropped = cv_bridge::CvImage(cv_depth_ptr->header, cv_depth_ptr->encoding, depth_cropped);
		cam_msg_cropped = cameraModel.cameraInfo();
		cam_msg_cropped.roi.width = roi.width;
		cam_msg_cropped.roi.height = roi.height;
		cam_msg_cropped.roi.x_offset = roi.x;
		cam_msg_cropped.roi.y_offset = roi.y;
		std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	}

	return true;
}


bool Scene::performSegmentation()
{
	segInfo = scenario->segmentationClient->segment(cv_rgb_cropped, cv_depth_cropped, cam_msg_cropped);

	if (!segInfo)
	{
		ROS_WARN_STREAM("Segmentation Failed!");
		return false;
	}

	segments = segmentCloudsFromImage(pile_cloud, segInfo->objectness_segmentation->image, cameraModel, roi, &labelToIndexLookup);

	if (segments.empty())
	{
		ROS_WARN_STREAM("No clusters were detected!");
		return false;
	}

	return true;
}

bool Scene::completeShapes()
{
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	approximateSegments.clear();
	completedSegments.clear();
	for (const auto& seg : segments)
	{
		if (!ros::ok()) { return false; }
		assert(!segment_cloud->empty());

		const pcl::PointCloud<PointT>::Ptr& segment_cloud = seg.second;

		// Compute bounding box
		Eigen::Vector3f min, max;
		getBoundingCube(*segment_cloud, min, max);
		double edge_length = max.x()-min.x(); // all edge of cube are same size
		double inflation = edge_length/5.0;
		min -= inflation * Eigen::Vector3f::Ones();
		max += inflation * Eigen::Vector3f::Ones();

		std::shared_ptr<octomap::OcTree> subtree(sceneOctree->create());
		subtree->setProbMiss(0.05);
		subtree->setProbHit(0.95);
		setBBox(minExtent, maxExtent, subtree.get());
		insertCloudInOctree(segment_cloud, worldTcamera, subtree.get());
		std::cerr << subtree->size() << std::endl;
//		for (const PointT& pt : *segment_cloud)
//		{
//			Eigen::Vector3f worldPt = worldTcamera.cast<float>()*pt.getVector3fMap();
//			assert(worldPt.cwiseMax(maxExtent.head<3>())==maxExtent.head<3>());
//			assert(worldPt.cwiseMin(minExtent.head<3>())==minExtent.head<3>());
//
//			octomap::OcTreeNode* node = subtree->setNodeValue(worldPt.x(), worldPt.y(), worldPt.z(),
//			                                                  std::numeric_limits<float>::infinity(), false);
//			subtree->insertPointCloud()
//			assert(node);
//		}

		if (scenario->completionClient->completionClient.exists())
		{
			scenario->completionClient->completeShape(min, max, worldTcamera.cast<float>(), sceneOctree, subtree.get(), false);
		}
		setBBox(Eigen::Vector3f(-2,-2,-2), Eigen::Vector3f(2,2,2), subtree.get());
		completedSegments.insert({seg.first, subtree});


		// Visualize approximate shape
		visualization_msgs::Marker m;
		auto approx = approximateShape(subtree.get());
		approximateSegments.insert({seg.first, approx});

		// Search for this segment in past models
//		MotionModel* model = matchModel(subtree, selfModels);
//		if (!model)
//		{
//			std::string modelID = "completed_"+std::to_string(selfModels.size());
//			auto newModel = std::make_unique<RigidMotionModel>();
//			newModel->membershipShapes
//			selfModels[modelID] = std::move(newModel);
//			model = selfModels[modelID].get();
//		}

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
		pcl::transformPointCloud (*cropped_cloud, *transformed_cloud, worldTcamera);
		getAABB(*transformed_cloud, min, max);
		minExtent.head<3>() = min; maxExtent.head<3>() = max;
	}

	std::tie(occludedPts, occlusionTree) = getOcclusionsInFOV(sceneOctree, cameraModel, worldTcamera.inverse(Eigen::Isometry), minExtent.head<3>(), maxExtent.head<3>());
//	planningEnvironment->occluded_pts = occluded_pts;

	if (occludedPts.empty())
	{
		ROS_ERROR_STREAM("Occluded points returned empty.");
		return false;
	}

	/////////////////////////////////////////////////////////////////
	// Filter completion results
	/////////////////////////////////////////////////////////////////
	{
		const int nPts = static_cast<int>(occludedPts.size());
		std::set<octomap::point3d, vector_less_than<3, octomap::point3d>> rejects;
//		#pragma omp parallel for schedule(dynamic)
		#pragma omp parallel
		{
			#pragma omp single
			{
				for (const auto seg : completedSegments)
				{
					#pragma omp task
					{
						const std::shared_ptr<octomap::OcTree>& segment = seg.second;
						unsigned d = segment->getTreeDepth();
						for (int i = 0; i<nPts; ++i)
						{
							octomap::OcTreeNode* node = segment->search(occludedPts[i], d);
							if (node && node->getOccupancy()>0.5)
							{
								#pragma omp critical
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
		occlusionTree->updateInnerOccupancy();
		occludedPts.erase(std::remove_if(occludedPts.begin(), occludedPts.end(),
		                                 [&](const octomath::Vector3& p){ return rejects.find(p) != rejects.end();}),
		                  occludedPts.end());
	}

	/////////////////////////////////////////////////////////////////
	// Filter parts of models
	/////////////////////////////////////////////////////////////////
	{
		std::vector<int> assignments(occludedPts.size(), -1);
		const int nPts = static_cast<int>(occludedPts.size());
		#pragma omp parallel for schedule(dynamic)
		for (int i = 0; i < nPts; ++i)
		{
			Eigen::Vector3d pt(occludedPts[i].x(), occludedPts[i].y(), occludedPts[i].z());
			int m = 0;
			double maxL = std::numeric_limits<double>::lowest();
			for (const auto& model : selfModels)
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
				occlusionTree->deleteNode(occludedPts[i]);
				sceneOctree->deleteNode(occludedPts[i]);
			}
		}
		occlusionTree->updateInnerOccupancy();
		// Lambda function: capture by reference, get contained value, predicate function
		occludedPts.erase(std::remove_if(occludedPts.begin(), occludedPts.end(),
		                                 [&](const octomath::Vector3& p){ return assignments[&p - &*occludedPts.begin()] >= 0;}),
		                  occludedPts.end());
	}
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	return true;
}
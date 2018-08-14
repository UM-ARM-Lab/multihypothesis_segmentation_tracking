//
// Created by arprice on 7/24/18.
//

// Octree utilities
#include "mps_voxels/MotionModel.h"
#include "mps_voxels/octree_utils.h"
#include "mps_voxels/pointcloud_utils.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <Eigen/Geometry>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>

// Point cloud utilities
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/ros.h>

#include <algorithm>
#include <memory>
#include <random>
#include <queue>


//#include <octomap_server/OctomapServer.h>
#include <octomap/Pointcloud.h>

template <typename Point>
void setBBox(const Point& min, const Point& max, octomap::OcTree* ot)
{
	octomap::point3d om_min{(float)min.x(), (float)min.y(), (float)min.z()};
	octomap::point3d om_max{(float)max.x(), (float)max.y(), (float)max.z()};
	ot->setBBXMin(om_min);
	ot->setBBXMax(om_max);
	ot->useBBXLimit(true);
}

class LocalOctreeServer// : public octomap_server::OctomapServer
{
public:
	using OcTreeT = octomap::OcTree;

	double m_res;
	std::string m_worldFrameId; // the map frame
	std::unique_ptr<OcTreeT> m_octree;

	LocalOctreeServer(ros::NodeHandle private_nh_ = ros::NodeHandle("~"))
		: m_res(0.05),
		  m_worldFrameId("/map"),
		  m_octree(nullptr)
//		: octomap_server::OctomapServer(private_nh_)
	{
//		this->m_pointCloudSub->unsubscribe();

		private_nh_.param("resolution", m_res, m_res);
		private_nh_.param("frame_id", m_worldFrameId, m_worldFrameId);

		m_octree = std::make_unique<OcTreeT>(m_res);


		Eigen::Vector3d min, max;
		m_octree->getMetricMin(min.x(), min.y(), min.z());
		m_octree->getMetricMax(max.x(), max.y(), max.z());
		setBBox(min, max, m_octree.get());
	}

	void insertCloud(const pcl::PointCloud<PointT>::Ptr& cloud, const Eigen::Affine3d& worldTcamera)
	{
		pcl::PointCloud<PointT>::Ptr transformed_cloud (new pcl::PointCloud<PointT>());
		pcl::transformPointCloud (*cloud, *transformed_cloud, worldTcamera);

		octomap::point3d min = m_octree->getBBXMin(), max = m_octree->getBBXMax();
		for (int i = 0; i < 3; ++i)
		{
			min(i) = std::min(min(i), (float)worldTcamera.translation()[i]);
			max(i) = std::max(max(i), (float)worldTcamera.translation()[i]);
		}
		setBBox(min, max, m_octree.get());

		octomap::Pointcloud pc;
		pc.reserve(transformed_cloud->size());
		for (const PointT& pt : *transformed_cloud)
		{
			pc.push_back(pt.x, pt.y, pt.z);
		}
		octomap::point3d origin((float)worldTcamera.translation().x(),
		                        (float)worldTcamera.translation().y(),
		                        (float)worldTcamera.translation().z());
//		m_octree->insertPointCloud(pc, origin, -1, true, false);
//		m_octree->insertPointCloudRays(pc, origin, -1, true);
		m_octree->insertPointCloud(pc, origin, -1, false, true);
		m_octree->updateInnerOccupancy();
	}

	const std::string& getWorldFrame() const { return this->m_worldFrameId; }
	const OcTreeT* getOctree() const { return this->m_octree.get(); }
	OcTreeT* getOctree() { return this->m_octree.get(); }
};

template<int DIM,
	typename PointT=Eigen::Matrix<double, DIM, 1>>
struct vector_less_than
{
//	bool operator()(const PointT& a,
//	                const PointT& b) const
//	{
//		for(int i=0; i<DIM; ++i)
//		{
//			if(a[i]<b[i]) return true;
//			if(a[i]>b[i]) return false;
//		}
//		return false;
//	}
	bool operator()(const PointT& a,
	                const PointT& b) const
	{
		for(int i=0; i<DIM; ++i)
		{
			if(a(i)<b(i)) return true;
			if(a(i)>b(i)) return false;
		}
		return false;
	}
};


struct SeedDistanceFunctor
{
	using Solution = std::vector<double>;
	const Solution seed;
	SeedDistanceFunctor(Solution _seed) : seed(std::move(_seed)) {}
	static double distance(const Solution& a, const Solution& b)
	{
		assert(a.size() == b.size());
		double d = 0.0;
		for (size_t i = 0; i < a.size(); ++i)
		{
			d += fabs(a[i]-b[i]);
		}
		return d;
	}

	// NB: priority_queue is a max-heap structure, so less() should actually return >
	// "highest priority"
	bool operator()(const Solution& a, const Solution& b) const
	{
		return distance(seed, a) > distance(seed, b);
	}
};

MotionModel* matchModel(std::shared_ptr<octomap::OcTree> subtree, std::map<std::string, std::unique_ptr<MotionModel>> motionModels)
{

}


std::shared_ptr<LocalOctreeServer> mapServer;
//std::shared_ptr<OctreeRetriever> mapClient;
std::shared_ptr<VoxelCompleter> completionClient;
ros::Publisher octreePub;
ros::Publisher targetPub;
ros::Publisher commandPub;
ros::Publisher displayPub;
std::shared_ptr<tf::TransformListener> listener;
std::shared_ptr<tf::TransformBroadcaster> broadcaster;
robot_model::RobotModelPtr pModel;
std::vector<robot_model::JointModelGroup*> jmgs;
sensor_msgs::JointState::ConstPtr latestJoints;
std::map<std::string, std::unique_ptr<MotionModel>> motionModels;

int octomapBurnInCounter = 0;
const int OCTOMAP_BURN_IN = 2; /// Number of point clouds to process before using octree
const double maxStepSize = std::numeric_limits<double>::infinity();

void publishOctree(octomap::OcTree* tree, const std::string& globalFrame, const std::string& ns = "")
{
	bool publishMarkerArray = (octreePub.getNumSubscribers()>0);

	if (publishMarkerArray)
	{
		visualization_msgs::MarkerArray occupiedNodesVis = visualizeOctree(tree, globalFrame);
		if (!ns.empty())
		{
			for (visualization_msgs::Marker& m : occupiedNodesVis.markers)
			{
				m.ns = ns;
			}
		}
		octreePub.publish(occupiedNodesVis);
	}
}

void handleJointState(const sensor_msgs::JointState::ConstPtr& js)
{
	latestJoints = js;
	ROS_DEBUG_ONCE("Joint joints!");
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	std::cerr << "Got message." << std::endl;
	if (!listener->waitForTransform(mapServer->getWorldFrame(), cloud_msg->header.frame_id, ros::Time(0), ros::Duration(5.0)))
	{
		ROS_WARN_STREAM("Failed to look up transform between '" << mapServer->getWorldFrame() << "' and '" << cloud_msg->header.frame_id << "'.");
		return;
	}

	Eigen::Vector4f maxExtent(0.4f, 0.6f, 0.5f, 1);
	Eigen::Vector4f minExtent(-0.4f, -0.6f, -0.05f, 1);

	setBBox(minExtent, maxExtent, mapServer->getOctree());

	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	if (octomapBurnInCounter++ < OCTOMAP_BURN_IN) { return; }
	if (octomapBurnInCounter > 8*OCTOMAP_BURN_IN) { mapServer->getOctree()->clear(); octomapBurnInCounter = 0; return; }

	image_geometry::PinholeCameraModel cameraModel;
	cameraModel.fromCameraInfo(info_msg);

	// Get Octree
	octomap::OcTree* octree = mapServer->getOctree();
	std::string globalFrame = mapServer->getWorldFrame();
//	std::tie(octree, globalFrame) = mapClient->getOctree();
	if (!octree)
	{
		ROS_ERROR("Octree generation failed.");
		return;
	}

	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	tf::StampedTransform cameraFrameInTableCoordinates;
	listener->lookupTransform(cameraModel.tfFrame(), globalFrame, ros::Time(0), cameraFrameInTableCoordinates);
	Eigen::Affine3d cameraTtable, worldTcamera;
	tf::transformTFToEigen(cameraFrameInTableCoordinates, cameraTtable);
	tf::transformTFToEigen(cameraFrameInTableCoordinates.inverse(), worldTcamera);

	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
	pcl::fromROSMsg(*cloud_msg, *cloud);

	/////////////////////////////////////////////////////////////////
	// Crop to bounding box
	/////////////////////////////////////////////////////////////////
	pcl::PointCloud<PointT>::Ptr cropped_cloud = cropInCameraFrame(cloud, minExtent, maxExtent, worldTcamera);
	if (cropped_cloud->empty())
	{
		ROS_WARN("Filtered cloud contains no points.");
		return;
	}
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	cropped_cloud = filterInCameraFrame(cropped_cloud);
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	mapServer->insertCloud(cropped_cloud, worldTcamera);
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;



	/////////////////////////////////////////////////////////////////
	// Apply prior motion models
	/////////////////////////////////////////////////////////////////
	robot_state::RobotState currentState(pModel);
	robot_state::RobotState toState(pModel);
	currentState.setToDefaultValues();
	toState.setToDefaultValues();
	if (latestJoints)
	{
		moveit::core::jointStateToRobotState(*latestJoints, currentState);
		moveit::core::jointStateToRobotState(*latestJoints, toState);
		currentState.updateLinkTransforms();
		toState.updateLinkTransforms();
	}

	// Update from robot state + TF
	for (const auto& model : motionModels)
	{
		if (listener->waitForTransform(model.first, mapServer->getWorldFrame(), ros::Time(0), ros::Duration(5.0)))
		{
			tf::StampedTransform stf;
			listener->lookupTransform(model.first, mapServer->getWorldFrame(), ros::Time(0), stf);
			tf::transformTFToEigen(stf, model.second->localTglobal);
		}
		else if (std::find(pModel->getLinkModelNames().begin(), pModel->getLinkModelNames().end(), model.first) != pModel->getLinkModelNames().end())
		{

			ROS_ERROR_STREAM("Unable to compute transform from '" << mapServer->getWorldFrame() << "' to '" << model.first << "'.");
			return;
		}

		// Compute bounding spheres
		model.second->updateMembershipStructures();
	}
	{
		visualization_msgs::MarkerArray markers;
		int id = 0;
		for (const auto& model : motionModels)
		{
			visualization_msgs::Marker m;
			m.ns = "collision";
			m.id = id++;
			m.type = visualization_msgs::Marker::SPHERE;
			m.action = visualization_msgs::Marker::ADD;
			m.scale.x = m.scale.y = m.scale.z = model.second->boundingSphere.radius*2.0;
			m.color.a = 0.5f;
			Eigen::Vector3d p = model.second->localTglobal.inverse() * model.second->boundingSphere.center;
			m.pose.position.x = p.x();
			m.pose.position.y = p.y();
			m.pose.position.z = p.z();
			m.pose.orientation.w = 1.0;
			m.frame_locked = true;
			m.header.stamp = cloud_msg->header.stamp;
			m.header.frame_id = globalFrame;
			markers.markers.push_back(m);
		}
		octreePub.publish(markers);
	}

	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	{
		std::vector<int> assignments(cropped_cloud->size(), -1);
		const int nPts = static_cast<int>(cropped_cloud->size());
		#pragma omp parallel for schedule(dynamic)
		for (int i = 0; i < nPts; ++i)
		{
			Eigen::Vector3d pt = (worldTcamera.cast<float>()*cropped_cloud->points[i].getVector3fMap()).cast<double>();
			int m = 0;
			double maxL = std::numeric_limits<double>::lowest();
			for (const auto& model : motionModels)
			{
				double L = model.second->membershipLikelihood(pt);
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
		return;
	}

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

		SensorModel sensor{worldTcamera.translation()};

		#pragma omp parallel for schedule(dynamic)
		for (size_t i = 0; i < iters.size(); ++i)
		{
			const auto& it = iters[i];
			Eigen::Vector3d pt(it.getX(), it.getY(), it.getZ());
			double size = octree->getNodeSize(it.getDepth());
			int m = 0;
			double maxL = std::numeric_limits<double>::lowest();
			for (const auto& model : motionModels)
			{
//				Eigen::Vector3d p = model.second->localTglobal.inverse() * model.second->boundingSphere.center;
//				pt -= ((p - pt).normalized() * size);
				double L = model.second->membershipLikelihood(pt, sensor);
				if (L > 0.0 && L > maxL)
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

	{
		visualization_msgs::MarkerArray occupiedNodesVis = visualizeOctree(octree, globalFrame);
		for (visualization_msgs::Marker& m : occupiedNodesVis.markers)
		{
			m.ns = "map";
		}
		octreePub.publish(occupiedNodesVis);
	}


	// Perform segmentation
	std::vector<pcl::PointCloud<PointT>::Ptr> segments = segment(cropped_cloud);
	if (segments.empty())
	{
		ROS_WARN_STREAM("No clusters were detected!");
		return;
	}

	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	std::vector<std::shared_ptr<octomap::OcTree>> completedSegments;
	for (const pcl::PointCloud<PointT>::Ptr& segment_cloud : segments)
	{
		// Compute bounding box
		Eigen::Vector3f min, max;
		getBoundingCube(*segment_cloud, min, max);

		std::shared_ptr<octomap::OcTree> subtree(octree->create());
		for (const PointT& pt : *segment_cloud)
		{
			Eigen::Vector3f worldPt = worldTcamera.cast<float>()*pt.getVector3fMap();
			assert(worldPt.cwiseMax(maxExtent.head<3>())==maxExtent.head<3>());
			assert(worldPt.cwiseMin(minExtent.head<3>())==minExtent.head<3>());

			octomap::OcTreeNode* node = subtree->setNodeValue(worldPt.x(), worldPt.y(), worldPt.z(),
			                                                  std::numeric_limits<float>::infinity(), false);
			assert(node);
		}

		if (completionClient->completionClient.exists())
		{
			completionClient->completeShape(min, max, worldTcamera.cast<float>(), octree, subtree.get());
			publishOctree(subtree.get(), globalFrame, "completed");
		}

		completedSegments.push_back(subtree);

		// Search for this segment in past models
//		MotionModel* model = matchModel(subtree, motionModels);
//		if (!model)
//		{
//			std::string modelID = "completed_"+std::to_string(motionModels.size());
//			auto newModel = std::make_unique<RigidMotionModel>();
//			newModel->membershipShapes
//			motionModels[modelID] = std::move(newModel);
//			model = motionModels[modelID].get();
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
	octomap::point3d_collection occluded_pts;
	std::shared_ptr<octomap::OcTree> occlusionTree;
	std::tie(occluded_pts, occlusionTree) = getOcclusionsInFOV(octree, cameraModel, cameraTtable, minExtent.head<3>(), maxExtent.head<3>());
//	octomap::point3d cameraOrigin(worldTcamera.translation().x(), worldTcamera.translation().y(), worldTcamera.translation().z());
//	octomap::point3d collision;
//	bool collided = octree->castRay(cameraOrigin, -cameraOrigin, collision, false);
//	assert(collided);
//	collided = octree->castRay(cameraOrigin, octomap::point3d(0, 0, 0.25)-cameraOrigin, collision, false);

	if (occluded_pts.empty())
	{
		ROS_ERROR_STREAM("Occluded points returned empty.");
		return;
	}

	/////////////////////////////////////////////////////////////////
	// Filter parts of models
	/////////////////////////////////////////////////////////////////
	{
		std::vector<int> assignments(occluded_pts.size(), -1);
		const int nPts = static_cast<int>(occluded_pts.size());
		#pragma omp parallel for schedule(dynamic)
		for (int i = 0; i < nPts; ++i)
		{
			Eigen::Vector3d pt(occluded_pts[i].x(), occluded_pts[i].y(), occluded_pts[i].z());
			int m = 0;
			double maxL = std::numeric_limits<double>::lowest();
			for (const auto& model : motionModels)
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
				occlusionTree->deleteNode(occluded_pts[i]);
				octree->deleteNode(occluded_pts[i]);
			}
		}
		occlusionTree->updateInnerOccupancy();
		// Lambda function: capture by reference, get contained value, predicate function
		occluded_pts.erase(std::remove_if(occluded_pts.begin(), occluded_pts.end(),
		                                  [&](const octomath::Vector3& p){ return assignments[&p - &*occluded_pts.begin()] >= 0;}),
		                   occluded_pts.end());
	}
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	{
		visualization_msgs::MarkerArray occupiedNodesVis = visualizeOctree(octree, globalFrame);
		for (visualization_msgs::Marker& m : occupiedNodesVis.markers)
		{
			m.ns = "map";
		}
		octreePub.publish(occupiedNodesVis);
	}

	/////////////////////////////////////////////////////////////////
	// Compute the Most Occluding Segment
	/////////////////////////////////////////////////////////////////
	std::map<octomap::point3d, int, vector_less_than<3, octomap::point3d>> coordToSegment;
	std::vector<int> occludedBySegmentCount(segments.size(), 0);
	for (int i = 0; i < static_cast<int>(segments.size()); ++i)
	{
		const pcl::PointCloud<PointT>::Ptr& pc = segments[i];
		for (const PointT& pt : *pc)
		{
			octomap::point3d coord = octree->keyToCoord(octree->coordToKey(octomap::point3d(pt.x, pt.y, pt.z)));
			coordToSegment.insert({coord, i});
		}
	}
	#pragma omp parallel for
	for (int i = 0; i < occluded_pts.size(); ++i)
	{
		const auto& pt_world = occluded_pts[i];
		octomap::point3d cameraOrigin((float)cameraTtable.translation().x(), (float)cameraTtable.translation().y(), (float)cameraTtable.translation().z());
		octomap::point3d collision;
		octree->castRay(cameraOrigin, pt_world-cameraOrigin, collision);
		collision = octree->keyToCoord(octree->coordToKey(collision)); // regularize

		const auto& iter = coordToSegment.find(collision);
		if (iter != coordToSegment.end())
		{
			#pragma omp critical
			occludedBySegmentCount[iter->second]++;
		}
	}
	int mostOccludingSegmentIdx = std::distance(occludedBySegmentCount.begin(),
		std::max_element(occludedBySegmentCount.begin(), occludedBySegmentCount.end()));
	visualization_msgs::MarkerArray objToMoveVis = visualizeOctree(completedSegments[mostOccludingSegmentIdx].get(), globalFrame);
	for (visualization_msgs::Marker& m : objToMoveVis.markers)
	{
		m.colors.clear();
		m.color.r = 1.0;
		m.color.a = 1.0;
		m.scale.x *= 1.2; m.scale.y *= 1.2; m.scale.z *= 1.2;
		m.ns = "worst" + std::to_string(m.id);
	}
	octreePub.publish(objToMoveVis);

	std::random_device rd;
	std::mt19937 g(rd());

	std::shuffle(occluded_pts.begin(), occluded_pts.end(), g);
	geometry_msgs::Point pt;
	pt.x = occluded_pts.front().x();
	pt.y = occluded_pts.front().y();
	pt.z = occluded_pts.front().z();
//	targetPub.publish(pt);

//	publishOctree(occlusionTree.get(), globalFrame);
	{
		visualization_msgs::MarkerArray occupiedNodesVis = visualizeOctree(occlusionTree.get(), globalFrame);
		for (visualization_msgs::Marker& m : occupiedNodesVis.markers)
		{
			m.ns = "hidden";
			m.colors.clear();
			m.color.a = 1.0f;
			m.color.r = 0.5f;
			m.color.g = 0.5f;
			m.color.b = 0.5f;
		}
		octreePub.publish(occupiedNodesVis);
	}
	std::cerr << "Published " << occluded_pts.size() << " points." << std::endl;

	// Reach out and touch target

	assert(!jmgs.empty());

	for (const robot_model::JointModelGroup* ee : pModel->getEndEffectors())
	{
		assert(ee->isEndEffector());
		ee->getEndEffectorParentGroup();
		ee->getEndEffectorName();
	}

	const std::string& robotFrame = pModel->getRootLinkName();
	tf::StampedTransform robotFrameInGlobalCoordinates;
	if (!listener->waitForTransform(robotFrame, globalFrame, ros::Time(0), ros::Duration(1.0)))
	{
		ROS_ERROR_STREAM("Unable to compute transform from '" << globalFrame << "' to '" << robotFrame << "'.");
		return;
	}
	listener->lookupTransform(robotFrame, globalFrame, ros::Time(0), robotFrameInGlobalCoordinates);
	Eigen::Isometry3d robotTworld;
	tf::transformTFToEigen(robotFrameInGlobalCoordinates, robotTworld);

	robot_model::JointModelGroup* active_jmg = nullptr;

//	auto& push_points_world = occluded_pts;
	octomap::point3d_collection push_points_world;

//	for (const std::shared_ptr<octomap::OcTree>& ot : completedSegments)
	auto& ot = completedSegments[mostOccludingSegmentIdx];
	{
		for (octomap::OcTree::iterator it = ot->begin(ot->getTreeDepth()),
			     end = ot->end(); it != end; ++it)
		{
			if (ot->isNodeOccupied(*it))
			{
				push_points_world.emplace_back(octomath::Vector3{(float)it.getX(), (float)it.getY(), (float)it.getZ()});
			}
		}
	}
	std::shuffle(push_points_world.begin(), push_points_world.end(), g);

	trajectory_msgs::JointTrajectory cmd;
	cmd.header.frame_id = pModel->getRootLinkName();
	std::vector<std::vector<double>> solutions;
	for (const auto& pt_world : push_points_world)
	{
		if (pt_world.z() < 0.05) { continue; }
		Eigen::Vector3d pt(pt_world.x(), pt_world.y(), pt_world.z());

		std::sort(jmgs.begin(), jmgs.end(), [&](const robot_model::JointModelGroup* a, const robot_model::JointModelGroup* b)->bool
		{
			return (pt-currentState.getFrameTransform(a->getOnlyOneEndEffectorTip()->getName()).translation()).squaredNorm()
			       < (pt-currentState.getFrameTransform(b->getOnlyOneEndEffectorTip()->getName()).translation()).squaredNorm();
		});

		for (robot_model::JointModelGroup* jmg : jmgs)
		{
			const kinematics::KinematicsBaseConstPtr& solver = jmg->getSolverInstance();
			assert(solver.get());

			Eigen::Affine3d solverTrobot = Eigen::Affine3d::Identity();
			currentState.setToIKSolverFrame(solverTrobot, solver);

			// Convert to solver frame
			Eigen::Isometry3d goalPose = Eigen::Isometry3d::Identity();
			goalPose.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
			goalPose.translation() = pt;
			goalPose.translation() += 0.25 * Eigen::Vector3d::UnitZ();
			Eigen::Affine3d pt_solver = solverTrobot * robotTworld * goalPose;

			std::vector<geometry_msgs::Pose> targetPoses;
			Eigen::Quaterniond q(pt_solver.linear());
			geometry_msgs::Pose pose;
			pose.position.x = pt_solver.translation().x();
			pose.position.y = pt_solver.translation().y();
			pose.position.z = pt_solver.translation().z();
			pose.orientation.x = q.x();
			pose.orientation.y = q.y();
			pose.orientation.z = q.z();
			pose.orientation.w = q.w();
			targetPoses.push_back(pose);

			for (size_t i = 0; i < targetPoses.size(); ++i)
			{
				const geometry_msgs::Pose& p = targetPoses[i];
				if (!latestJoints)
				{
					tf::Transform t2;
					tf::transformEigenToTF(solverTrobot, t2);
					broadcaster->sendTransform(
						tf::StampedTransform(t2.inverse(), ros::Time::now(), pModel->getRootLinkName(), solver->getBaseFrame()));
				}

				tf::Transform t;
				tf::poseMsgToTF(p, t);
				broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), solver->getBaseFrame(),
				                                                solver->getBaseFrame() + "_goal_pose_" + std::to_string(i)));
				ros::Duration(0.2).sleep();
			}


//			solver->getTipFrame();

			std::vector<double> seed(jmg->getVariableCount(), 0.0);
//			currentState.copyJointGroupPositions(jmg, seed);
			kinematics::KinematicsResult result;
			kinematics::KinematicsQueryOptions options;
			options.discretization_method = kinematics::DiscretizationMethod::ALL_DISCRETIZED;
			solver->getPositionIK(targetPoses, seed, solutions, result, options);

			// Toss out invalid options
			if (maxStepSize < std::numeric_limits<double>::infinity())
			{
				solutions.erase(std::remove_if(solutions.begin(), solutions.end(),
				                               [&](const std::vector<double>& sln){ return SeedDistanceFunctor::distance(seed, sln) > maxStepSize;}),
				                solutions.end());
			}

			SeedDistanceFunctor functor(seed);
			std::priority_queue<std::vector<double>, std::vector<std::vector<double>>, SeedDistanceFunctor>
				slnQueue(solutions.begin(), solutions.end(), functor);

			while (!slnQueue.empty())
			{
				toState.setJointGroupPositions(jmg, slnQueue.top());
				planning_scene::PlanningScene ps(pModel);

				collision_detection::CollisionRequest collision_request;
				collision_detection::CollisionResult collision_result;

				ps.checkSelfCollision(collision_request, collision_result, toState);

				if (!collision_result.collision)
				{
					active_jmg = jmg;

					const int INTERPOLATE_STEPS = 15;
					cmd.joint_names = active_jmg->getActiveJointModelNames();
					cmd.points.resize(INTERPOLATE_STEPS);

					// Verify the "plan" is collision-free

					for (int i = 0; i < INTERPOLATE_STEPS; ++i)
					{
						double t = i/static_cast<double>(INTERPOLATE_STEPS-1);
						robot_state::RobotState interpState(currentState);
						currentState.interpolate(toState, t, interpState, jmg);

						ps.checkSelfCollision(collision_request, collision_result, interpState);
						if (collision_result.collision)
						{
							active_jmg = nullptr;
							break;
						}

						interpState.copyJointGroupPositions(active_jmg, cmd.points[i].positions);
						cmd.points[i].time_from_start = ros::Duration(t*3.0);
					}

					if (active_jmg)
					{
						break;
					}
				}
				slnQueue.pop();
			}

			if (active_jmg)
			{
				break;
			}
		}

		if (active_jmg)
		{
			break;
		}
	}

	if (!active_jmg)
	{
		ROS_WARN("Unable to find a solution to any occluded point from any arm.");
		return;
	}

	cmd.header.stamp = ros::Time::now();
	moveit_msgs::DisplayTrajectory dt;
//	moveit_msgs::RobotState drs;
	moveit_msgs::RobotTrajectory drt;

	moveit::core::robotStateToRobotStateMsg(currentState, dt.trajectory_start);
	dt.model_id = pModel->getName();
	drt.joint_trajectory = cmd;
	dt.trajectory.push_back(drt);
	displayPub.publish(dt);

	commandPub.publish(cmd);
}

template <typename T>
void setIfMissing(ros::NodeHandle& nh, const std::string& param_name, const T& param_val)
{
	if (!nh.hasParam(param_name))
	{
		nh.setParam(param_name, param_val);
	}
}


using SyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo>;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "scene_explorer");
	ros::NodeHandle nh, pnh("~");

	listener = std::make_shared<tf::TransformListener>();
	broadcaster = std::make_shared<tf::TransformBroadcaster>();

	ros::Subscriber joint_sub = nh.subscribe("joint_states", 2, handleJointState);

	setIfMissing(pnh, "frame_id", "table_surface");
	setIfMissing(pnh, "resolution", 0.01);
	setIfMissing(pnh, "latch", false);
	setIfMissing(pnh, "filter_ground", false);
	setIfMissing(pnh, "filter_speckles", true);
	setIfMissing(pnh, "publish_free_space", false);
	setIfMissing(pnh, "sensor_model/max_range", 8.0);

	octreePub = nh.advertise<visualization_msgs::MarkerArray>("occluded_points", 1, true);
	targetPub = nh.advertise<geometry_msgs::Point>("target_point", 1, false);
	commandPub = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_trajectory", 1, false);
	displayPub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, false);
	mapServer = std::make_shared<LocalOctreeServer>(pnh);
	ros::Duration(1.0).sleep();
//	mapClient = std::make_shared<OctreeRetriever>(nh);
	completionClient = std::make_shared<VoxelCompleter>(nh);

	std::shared_ptr<robot_model_loader::RobotModelLoader> mpLoader
		= std::make_shared<robot_model_loader::RobotModelLoader>();
	pModel = mpLoader->getModel();

	if (!loadLinkMotionModels(pModel.get(), motionModels))
	{
		ROS_ERROR("Model loading failed.");
	}
	motionModels.erase("victor_base_plate"); // HACK: camera always collides
	motionModels.erase("victor_pedestal");
	motionModels.erase("victor_left_arm_mount");
	motionModels.erase("victor_right_arm_mount");

	assert(!pModel->getJointModelGroupNames().empty());
	for (robot_model::JointModelGroup* jmg : pModel->getJointModelGroups())
	{
		if (jmg->isChain() && jmg->getSolverInstance())
		{
			jmgs.push_back(jmg);
			jmg->getSolverInstance()->setSearchDiscretization(0.1); // ~5 degrees
			const auto& ees = jmg->getAttachedEndEffectorNames();
			std::cerr << "Loaded jmg '" << jmg->getName() << "' " << jmg->getSolverInstance()->getBaseFrame() << std::endl;
			for (const std::string& eeName : ees)
			{
				const robot_model::JointModelGroup* ee = pModel->getEndEffector(eeName);
				ee->getJointRoots();
				const robot_model::JointModel* rootJoint = ee->getCommonRoot();
				rootJoint->getNonFixedDescendantJointModels();

				std::cerr << "\t-" << eeName << ee->getFixedJointModels().size() << std::endl;
			}
		}
		else
		{
			std::cerr << "Did not load jmg '" << jmg->getName() << "'" << std::endl;
		}
	}

	std::string topic_prefix = "/kinect2_victor_head/qhd";
//	ros::Subscriber camInfoSub = nh.subscribe("kinect2_victor_head/qhd/camera_info", 1, camera_cb);

	message_filters::Subscriber<sensor_msgs::PointCloud2> point_sub(nh, topic_prefix+"/points", 10);
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, topic_prefix+"/camera_info", 10);

	message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), point_sub, info_sub);
	sync.registerCallback(cloud_cb);

//	ros::Subscriber sub = nh.subscribe ("kinect2_roof/qhd/points", 1, cloud_cb);


	//(new octomap::OcTree(d));


	ros::spin();

	return 0;
}
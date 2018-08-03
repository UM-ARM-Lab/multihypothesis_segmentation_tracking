//
// Created by arprice on 7/24/18.
//

// Octree utilities
#include "mps_voxels/octree_utils.h"
#include "mps_voxels/pointcloud_utils.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
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


#include <octomap_server/OctomapServer.h>

class LocalOctreeServer : public octomap_server::OctomapServer
{
public:
	LocalOctreeServer(ros::NodeHandle private_nh_ = ros::NodeHandle("~"))
		: octomap_server::OctomapServer(private_nh_)
	{
		this->m_pointCloudSub->unsubscribe();
	}

	const std::string& getWorldFrame() const { return this->m_worldFrameId; }
	const OcTreeT* getOctree() const { return this->m_octree; }
	OcTreeT* getOctree() { return this->m_octree; }
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

std::shared_ptr<LocalOctreeServer> mapServer;
std::shared_ptr<OctreeRetriever> mapClient;
std::shared_ptr<VoxelCompleter> completionClient;
ros::Publisher octreePub;
ros::Publisher targetPub;
ros::Publisher commandPub;
std::shared_ptr<tf::TransformListener> listener;
std::shared_ptr<tf::TransformBroadcaster> broadcaster;
robot_model::RobotModelPtr pModel;
sensor_msgs::JointState::ConstPtr latestJoints;
int octomapBurnInCounter = 0;
const int OCTOMAP_BURN_IN = 5; /// Number of point clouds to process before using octree

void publishOctree(octomap::OcTree* tree, const std::string& globalFrame)
{
	bool publishMarkerArray = (octreePub.getNumSubscribers()>0);

	if (publishMarkerArray)
	{
		visualization_msgs::MarkerArray occupiedNodesVis = visualizeOctree(tree, globalFrame);

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

	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
	mapServer->insertCloudCallback(cloud_msg);
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	if (octomapBurnInCounter++ < OCTOMAP_BURN_IN) { return; }
	if (octomapBurnInCounter > 8*OCTOMAP_BURN_IN) { mapServer->getOctree()->clear(); octomapBurnInCounter = 0; return; }

	image_geometry::PinholeCameraModel cameraModel;
	cameraModel.fromCameraInfo(info_msg);

	Eigen::Vector4f maxExtent(0.4f, 0.6f, 0.5f, 1);
	Eigen::Vector4f minExtent(-0.4f, -0.6f, -0.05f, 1);

	// Get Octree
	octomap::OcTree* octree = mapServer->getOctree();
	std::string globalFrame = mapServer->getWorldFrame();
//	std::tie(octree, globalFrame) = mapClient->getOctree();
	if (!octree)
	{
		ROS_ERROR("Octree lookup failed.");
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

	// Crop to bounding box
	pcl::PointCloud<PointT>::Ptr cropped_cloud = crop(cloud, minExtent, maxExtent, worldTcamera);
	if (cropped_cloud->empty())
	{
		ROS_WARN("Filtered cloud contains no points.");
		return;
	}
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	// Perform segmentation
	std::vector<pcl::PointCloud<PointT>::Ptr> segments = segment(cropped_cloud);

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
		}

		completedSegments.push_back(subtree);

		publishOctree(subtree.get(), globalFrame);
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
	for (const auto& pt_world : occluded_pts)
	{
		octomap::point3d cameraOrigin(cameraTtable.translation().x(), cameraTtable.translation().y(), cameraTtable.translation().z());
		octomap::point3d collision;
		octree->castRay(cameraOrigin, pt_world-cameraOrigin, collision);
		collision = octree->keyToCoord(octree->coordToKey(collision)); // regularize

		const auto& iter = coordToSegment.find(collision);
		if (iter != coordToSegment.end())
		{
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
	targetPub.publish(pt);

//	occlusionTree->
	publishOctree(occlusionTree.get(), globalFrame);
	std::cerr << "Published " << occluded_pts.size() << " points." << std::endl;

	// Reach out and touch target
	robot_state::RobotState rs(pModel);
	rs.setToDefaultValues();
	if (latestJoints)
	{
		moveit::core::jointStateToRobotState(*latestJoints, rs);
	}

	assert(!pModel->getJointModelGroupNames().empty());

	std::vector<robot_model::JointModelGroup*> jmgs;
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
	assert(!jmgs.empty());

	for (const robot_model::JointModelGroup* ee : pModel->getEndEffectors())
	{
		assert(ee->isEndEffector());
		ee->getEndEffectorParentGroup();
		ee->getEndEffectorName();
	}

	const std::string& robotFrame = pModel->getRootLinkName();
	tf::StampedTransform robotFrameInGlobalCoordinates;
	listener->lookupTransform(robotFrame, globalFrame, ros::Time(0), robotFrameInGlobalCoordinates);
	Eigen::Isometry3d robotTworld;
	tf::transformTFToEigen(robotFrameInGlobalCoordinates, robotTworld);

	robot_model::JointModelGroup* active_jmg;

	std::vector<std::vector<double>> solutions;
	for (const auto& pt_world : occluded_pts)
	{

		for (robot_model::JointModelGroup* jmg : jmgs)
		{
			const kinematics::KinematicsBaseConstPtr& solver = jmg->getSolverInstance();
			assert(solver.get());

			Eigen::Affine3d solverTrobot = Eigen::Affine3d::Identity();
			rs.setToIKSolverFrame(solverTrobot, solver);

			// Convert to solver frame
			Eigen::Isometry3d goalPose = Eigen::Isometry3d::Identity();
			goalPose.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
			goalPose.translation() = Eigen::Vector3d(pt_world.x(), pt_world.y(), pt_world.z());
			goalPose.translation() += 0.3 * Eigen::Vector3d::UnitZ();
			Eigen::Affine3d pt_solver = solverTrobot * robotTworld * goalPose;

			// TODO: Loop around z-axis
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


			solver->getTipFrame();

			std::vector<double> seed;
			rs.copyJointGroupPositions(jmg, seed);
			kinematics::KinematicsResult result;
			kinematics::KinematicsQueryOptions options;
			options.discretization_method = kinematics::DiscretizationMethod::ALL_DISCRETIZED;
			solver->getPositionIK(targetPoses, seed, solutions, result, options);


			if (!solutions.empty())
			{
				rs.setJointGroupPositions(jmg, solutions.front());
				active_jmg = jmg;
				std::cerr << "Got " << solutions.size() << " solutions." << std::endl;
				break;
			}
		}

		if (!solutions.empty())
		{
			break;
		}
	}

	if (solutions.empty())
	{
		ROS_WARN("Unable to find a solution to any occluded point from any arm.");
		return;
	}

	moveit_msgs::DisplayTrajectory dt;
//	moveit_msgs::RobotState drs;
	moveit_msgs::RobotTrajectory drt;

	sensor_msgs::JointState js;
	moveit::core::robotStateToJointStateMsg(rs, js);
	moveit::core::robotStateToRobotStateMsg(rs, dt.trajectory_start);
	dt.model_id = pModel->getName();
//	dt.trajectory_start = drs;
	drt.joint_trajectory.joint_names = js.name;
	drt.joint_trajectory.points.resize(1);
	drt.joint_trajectory.points[0].positions = js.position;
	drt.joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
	dt.trajectory.push_back(drt);
	commandPub.publish(dt);
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
	commandPub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, false);
	mapServer = std::make_shared<LocalOctreeServer>(pnh);
	mapClient = std::make_shared<OctreeRetriever>(nh);
	completionClient = std::make_shared<VoxelCompleter>(nh);

	std::shared_ptr<robot_model_loader::RobotModelLoader> mpLoader
		= std::make_shared<robot_model_loader::RobotModelLoader>();
	pModel = mpLoader->getModel();

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
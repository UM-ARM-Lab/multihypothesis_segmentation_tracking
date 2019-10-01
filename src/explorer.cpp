//
// Created by arprice on 7/24/18.
//

#include "mps_voxels/Manipulator.h"
#include "mps_voxels/MotionModel.h"
#include "mps_voxels/Tracker.h"
#include "mps_voxels/CudaTracker.h"
#include "mps_voxels/SiamTracker.h"
#include "mps_voxels/TargetDetector.h"
#include "mps_voxels/LocalOctreeServer.h"
#include "mps_voxels/octree_utils.h"
#include "mps_voxels/pointcloud_utils.h"
#include "mps_voxels/image_utils.h"
#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/shape_utils.h"
#include "mps_voxels/map_nearest.hpp"
#include "mps_voxels/planning/MotionPlanner.h"
#include "mps_voxels/ObjectLogger.h"
#include "mps_voxels/assert.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <pcl_ros/point_cloud.h> // Needed to publish a point cloud

#include <Eigen/Geometry>

#include <realtime_tools/realtime_publisher.h>
#include <mutex>

// Point cloud utilities
#include <depth_image_proc/depth_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <boost/core/null_deleter.hpp>

//#include <pcl/visualization/cloud_viewer.h>

#include <geometric_shapes/shape_operations.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Int64.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <mps_msgs/TrackBBoxAction.h>
#include <mps_msgs/AABBox2d.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/ros.h>

#include <algorithm>
#include <memory>
#include <random>
#include <queue>


#define ENABLE_PROFILING
#include <arc_utilities/timing.hpp>

#include <arm_video_recorder/TriggerVideoRecording.h>

#define _unused(x) ((void)(x))

#define HEADLESS true

//#if !HEADLESS
#include <opencv2/highgui.hpp>
//#endif

#if HEADLESS
#define waitKey(x) _unused((x))
#else
#define waitKey(x) cv::waitKey((x))
#endif

#if HEADLESS
#define imshow(n, t) _unused((n)); _unused((t))
#else
#define imshow(n, t) cv::imshow((n), (t))
#endif

#if HEADLESS
#define namedWindow(n, t) _unused((n)); _unused((t))
#else
#define namedWindow(n, t) cv::namedWindow((n), (t))
#endif


sensor_msgs::JointState::ConstPtr latestJoints;
std::mutex joint_mtx;

int mostAmbNode;
void manCallback(const std_msgs::Int64::ConstPtr& msg)
{
    mostAmbNode = msg->data;
    ROS_DEBUG_ONCE("Most Amb Node callback!");
}

void handleJointState(const sensor_msgs::JointState::ConstPtr& js)
{
	std::lock_guard<std::mutex> lk(joint_mtx);
	latestJoints = js;
	ROS_DEBUG_ONCE("Joint joints!");
}

template <typename T>
void operator+=(std::vector<T> &v1, const std::vector<T> &v2)
{
	v1.insert(v1.end(), v2.begin(), v2.end());
}

void insertDirect(const octomap::point3d_collection& points, const moveit::Pose& T, octomap::OcTree* tree)
{
	for (const auto& p : points)
	{
		const Eigen::Vector3d pt = T * Eigen::Vector3d(p.x(), p.y(), p.z());
		tree->setNodeValue(pt.x(), pt.y(), pt.z(), tree->getProbHitLog(), true);
	}
	tree->updateInnerOccupancy();
	tree->prune();
}

struct MarkerSet
{
	std::map<std::string, visualization_msgs::MarkerArray> arrays;
	visualization_msgs::MarkerArray& operator[](const std::string& s) { return arrays[s]; }
	visualization_msgs::MarkerArray flatten() const
	{
		visualization_msgs::MarkerArray ma;
		for (const auto& a : arrays)
		{
			ma.markers += a.second.markers;
		}
		return ma;
	}
};

void setAlpha(visualization_msgs::MarkerArray& ma, const float alpha)
{
	for (auto& ms : ma.markers)
	{
		ms.color.a = alpha;
		for (auto& c : ms.colors)
		{
			c.a = ms.color.a;
		}
	}
}

const std::string DEFAULT_TRAJECTORY_SERVER = "follow_joint_trajectory";

class SceneExplorer
{
public:
	//using SyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo>;
	using SyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>;
	using TrajectoryClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

	std::shared_ptr<LocalOctreeServer> mapServer;
	std::shared_ptr<VoxelCompleter> completionClient;
	std::shared_ptr<RGBDSegmenter> segmentationClient;
	std::unique_ptr<Tracker> tracker;
	std::unique_ptr<TargetDetector> targetDetector;
	std::unique_ptr<TrajectoryClient> trajectoryClient;
	std::unique_ptr<realtime_tools::RealtimePublisher<victor_hardware_interface::Robotiq3FingerCommand>> gripperLPub;
	std::unique_ptr<realtime_tools::RealtimePublisher<victor_hardware_interface::Robotiq3FingerCommand>> gripperRPub;
	ros::Publisher octreePub;
	ros::Publisher displayPub;
	ros::Publisher pcPub;
	ros::Subscriber indexofinterestSub;
	std::unique_ptr<image_transport::Publisher> segmentationPub;
	std::unique_ptr<image_transport::Publisher> targetPub;

	std::unique_ptr<image_transport::SubscriberFilter> rgb_sub;
	std::unique_ptr<image_transport::SubscriberFilter> depth_sub;
	std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> cam_sub;
	std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

    std::unique_ptr<ros::CallbackQueue> sensor_queue;
    std::unique_ptr<ros::AsyncSpinner> sensor_spinner;
	std::unique_ptr<tf::TransformListener> listener;
	std::unique_ptr<tf::TransformBroadcaster> broadcaster;

	std::unique_ptr<robot_model_loader::RobotModelLoader> mpLoader;
	robot_model::RobotModelPtr pModel;
	std::shared_ptr<Scenario> scenario;
	std::unique_ptr<Scene> scene;
	std::unique_ptr<SceneProcessor> processor;
	std::unique_ptr<MotionPlanner> planner;
	std::map<std::string, std::shared_ptr<MotionModel>> selfModels;

	double planning_time = 60.0;
	int planning_samples = 25;
//	int mostAmbNode = -1;
	std::string experiment_id;
	std::string experiment_dir;
	ros::ServiceClient externalVideoClient;
	MarkerSet allMarkers;

	std_msgs::ColorRGBA mapColor;

	SceneExplorer(ros::NodeHandle& nh, ros::NodeHandle& pnh);

	robot_state::RobotState getCurrentRobotState();
	trajectory_msgs::JointTrajectory fillMissingJointTrajectories(const trajectory_msgs::JointTrajectory& trajIn);
	std::map<ObjectIndex, moveit::Pose> followObjects(const std::shared_ptr<Motion>& motion, const Tracker* track);
	bool executeMotion(const std::shared_ptr<Motion>& motion, const robot_state::RobotState& recoveryState);

	void cloud_cb (const sensor_msgs::ImageConstPtr& rgb_msg,
	               const sensor_msgs::ImageConstPtr& depth_msg,
	               const sensor_msgs::CameraInfoConstPtr& cam_msg);
//    void manCallback(const std_msgs::Int64& msg);
	moveit_msgs::DisplayTrajectory visualize(const std::shared_ptr<Motion>& motion, const bool primaryOnly = false);
};

robot_state::RobotState SceneExplorer::getCurrentRobotState()
{
	robot_state::RobotState currentState(pModel);
	currentState.setToDefaultValues();
	{
		std::lock_guard<std::mutex> lk(joint_mtx);
		if (latestJoints)
		{
			moveit::core::jointStateToRobotState(*latestJoints, currentState);
		}
	}
	currentState.update();

	return currentState;
}

trajectory_msgs::JointTrajectory SceneExplorer::fillMissingJointTrajectories(const trajectory_msgs::JointTrajectory& trajIn)
{
	trajectory_msgs::JointTrajectory trajOut = trajIn;
	std::string executor_name = ros::names::resolve(DEFAULT_TRAJECTORY_SERVER, true);
	const std::size_t found = executor_name.find_last_of('/');
	if (found == std::string::npos)
	{
		// Unable to lookup list of joints
		return trajOut;
	}

	std::string executor_ns = executor_name.substr(0, found);
	std::vector<std::string> controller_joint_names;
	ros::NodeHandle nh;
	nh.param(executor_ns + "/" + "joints", controller_joint_names, controller_joint_names);
	if (controller_joint_names.empty())
	{
		return trajOut;
	}

	for (const auto& jName : trajOut.joint_names)
	{
		if (std::find(controller_joint_names.begin(), controller_joint_names.end(), jName) == controller_joint_names.end())
		{
			ROS_ERROR_STREAM("Joint '" << jName << "' is not listed as controlled by '" << executor_ns << "'.");
			return trajOut;
		}
	}

	robot_state::RobotState currentState = getCurrentRobotState();

	for (const auto& jName : controller_joint_names)
	{
		if (std::find(trajOut.joint_names.begin(), trajOut.joint_names.end(), jName) == trajOut.joint_names.end())
		{
			double pos = currentState.getJointPositions(jName)[0];
			trajOut.joint_names.push_back(jName);

			for (auto& pt : trajOut.points)
			{
				if (!pt.positions.empty()) { pt.positions.push_back(pos); }
				if (!pt.velocities.empty()) { pt.velocities.push_back(0); }
				if (!pt.accelerations.empty()) { pt.accelerations.push_back(0); }
//				if (!pt.effort.empty()) { pt.effort.push_back(0); }
			}
		}
	}

	return trajOut;
}

std::map<ObjectIndex, moveit::Pose> SceneExplorer::followObjects(const std::shared_ptr<Motion>& motion, const Tracker* track)
{
	moveit::Pose worldTstart = moveit::Pose::Identity();
	moveit::Pose worldTend = moveit::Pose::Identity();
	moveit::Pose worldTobject_init = moveit::Pose::Identity(); // Object pose at the initial time

#ifdef USE_PLAN_POSES
	// Compute the motion transform from the initial hand pose to the final
	const auto& composite = std::dynamic_pointer_cast<CompositeAction>(motion->action);
	const auto& action = std::dynamic_pointer_cast<JointTrajectoryAction>(composite->actions[composite->primaryAction]);

	worldTstart = action->palm_trajectory.front();
	worldTend = action->palm_trajectory.back();

#else
	auto msgStart = find_nearest(track->joint_buffer, track->rgb_buffer.begin()->first)->second;
	auto msgEnd = find_nearest(track->joint_buffer, track->rgb_buffer.rbegin()->first)->second;

	// Assume that state at start of track is start of manipulation
	robot_state::RobotState qStart(pModel);
	qStart.setToDefaultValues();
	moveit::core::jointStateToRobotState(*msgStart, qStart);
	qStart.update();

	robot_state::RobotState qEnd(pModel);
	qEnd.setToDefaultValues();
	moveit::core::jointStateToRobotState(*msgEnd, qEnd);
	qEnd.update();

	double maxDist = 0;
	bool isGrasping = false;
	for (const auto& manip : scenario->manipulators)
	{
		if (manip->isGrasping(qStart) && manip->isGrasping(qEnd))
		{
			isGrasping = true;
			std::cerr << "Is grasping!" << std::endl;
			const moveit::Pose Tstart = qStart.getFrameTransform(manip->palmName);
			const moveit::Pose Tend = qEnd.getFrameTransform(manip->palmName);

			const moveit::Pose Tdist = Tstart.inverse(Eigen::Isometry)*Tend;
			double dist = Tdist.translation().norm()+2.0*Eigen::Quaterniond(Tdist.rotation()).vec().norm();

			if (dist>maxDist)
			{
				worldTstart = scene->worldTrobot*Tstart;
				worldTend = scene->worldTrobot*Tend;
			}
		}
	}

#endif

	auto worldTobject_final = worldTend * worldTstart.inverse(Eigen::Isometry) * worldTobject_init;

	std::map<ObjectIndex, moveit::Pose> trajs;
	if (isGrasping)
	{
		trajs.insert({motion->targets.front(), worldTobject_final});
	}
	else
	{
		std::cerr << "Did not grasp!" << std::endl;
	}

	return trajs;
}

bool SceneExplorer::executeMotion(const std::shared_ptr<Motion>& motion, const robot_state::RobotState& recoveryState)
{
	if (!ros::ok()) { return false; }
	ros::Duration totalTime(0.0);
	if (motion && motion->action && std::dynamic_pointer_cast<CompositeAction>(motion->action))
	{
		auto actions = std::dynamic_pointer_cast<CompositeAction>(motion->action);
		for (size_t idx = 0; idx<actions->actions.size(); ++idx)
		{
			auto subTraj = std::dynamic_pointer_cast<JointTrajectoryAction>(actions->actions[idx]);
			if (subTraj)
			{
				totalTime += subTraj->cmd.points.back().time_from_start-subTraj->cmd.points.front().time_from_start;
			}
		}
	}

	auto compositeAction = std::dynamic_pointer_cast<CompositeAction>(motion->action);
	for (size_t a = 0; a < compositeAction->actions.size(); ++a)
	{
		const auto& action = compositeAction->actions[a];
		bool isPrimaryAction = ((compositeAction->primaryAction >= 0) && (compositeAction->primaryAction == static_cast<int>(a)));

		std::unique_ptr<CaptureGuard> captureGuard; ///< RAII-style stopCapture() for various exit paths

		if (isPrimaryAction)
		{
			captureGuard = std::make_unique<CaptureGuard>(tracker.get());
			auto cache = std::dynamic_pointer_cast<CachingRGBDSegmenter>(scenario->segmentationClient);
			if (cache) { cache->cache.clear(); }
			tracker->startCapture();
		}
//        if (a == 1)
//        {
//            captureGuard = std::make_unique<CaptureGuard>(tracker.get());
//            auto cache = std::dynamic_pointer_cast<CachingRGBDSegmenter>(scenario->segmentationClient);
//            if (cache) { cache->cache.clear(); }
//            tracker->startCapture();
//        }

		auto armAction = std::dynamic_pointer_cast<JointTrajectoryAction>(action);
		if (armAction)
		{
			control_msgs::FollowJointTrajectoryGoal goal;
			goal.trajectory = fillMissingJointTrajectories(armAction->cmd);
			for (const auto& jName : goal.trajectory.joint_names)
			{
				control_msgs::JointTolerance tol;
				tol.name = jName;
				tol.position = 0.1;
				tol.velocity = 0.5;
				tol.acceleration = 1.0;
				goal.goal_tolerance.push_back(tol);
			}
			goal.goal_time_tolerance = ros::Duration(30.0);

			std::cerr << "Sending joint trajectory." << std::endl;

			auto res = trajectoryClient->sendGoalAndWait(goal, ros::Duration(30.0) + totalTime, ros::Duration(1.0));
			if (!res.isDone() || (res.state_!=actionlib::SimpleClientGoalState::SUCCEEDED))
			{
				if (!res.isDone())
				{
					ROS_ERROR_STREAM("Trajectory client timed out.");
				}
				else if (res.state_!=actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					ROS_ERROR_STREAM("Trajectory following failed: " << res.toString() << " '" << res.text_ << "'; " << trajectoryClient->getResult()->error_code << trajectoryClient->getResult()->error_string);
				}

				planner->computePlanningScene(false);
				std::shared_ptr<Motion> recovery = planner->recoverCrash(getCurrentRobotState(), recoveryState);
				if (recovery)
				{
					executeMotion(recovery, recoveryState);
				}
				return false;
			}
			std::cerr << "Finished joint trajectory." << std::endl;
			ros::Duration(2.0).sleep(); // Trajectory seems to terminate slightly early
		}

		auto gripAction = std::dynamic_pointer_cast<GripperCommandAction>(action);
		if (gripAction)
		{
			gripAction->grasp.header.frame_id = pModel->getRootLinkName();
			gripAction->grasp.header.stamp = ros::Time::now();
			realtime_tools::RealtimePublisher<victor_hardware_interface::Robotiq3FingerCommand>* gripperPub = nullptr;
			if (gripAction->jointGroupName.find("left") != std::string::npos)
			{
				std::cerr << "Sending left gripper command." << std::endl;
				gripperPub = gripperLPub.get();
			}
			else if (gripAction->jointGroupName.find("right") != std::string::npos)
			{
				std::cerr << "Sending right gripper command." << std::endl;
				gripperPub = gripperRPub.get();
			}
			else
			{
				throw std::logic_error("Unknown gripper.");
			}


			if (gripperPub->trylock())
			{
				gripperPub->msg_ = gripAction->grasp;
				gripperPub->unlockAndPublish();
			}
			ros::Duration(2.0).sleep();
		}

		if (isPrimaryAction)
		{
			tracker->stopCapture();

			std::vector<ros::Time> steps;
			for (auto iter = tracker->rgb_buffer.begin(); iter != tracker->rgb_buffer.end(); std::advance(iter, 5))
			{
				steps.push_back(iter->first);
			}

//			tracker->track(steps);

//TODO: go through all objectindexes, compute their bboxes
//      useful functions:
//			scene->targetObjectID = std::make_shared<ObjectIndex>(scene->labelToIndexLookup.at((unsigned)matchID));


			cv::Mat temp_seg = scene->segInfo->objectness_segmentation->image;
			scene->labelToBBoxLookup = getBBox(temp_seg, scene->roi);

			for(auto pair:scene->labelToBBoxLookup){
				tracker->siamtrack(steps, pair.second);
			}

		}
//        if (a == compositeAction->actions.size())
//        {
//            tracker->stopCapture();
//
//            std::vector<ros::Time> steps;
//            for (auto iter = tracker->rgb_buffer.begin(); iter != tracker->rgb_buffer.end(); std::advance(iter, 5))
//            {
//                steps.push_back(iter->first);
//            }
//
//            tracker->track(steps);
//        }
	}
	return true;
}

void SceneExplorer::cloud_cb(const sensor_msgs::ImageConstPtr& rgb_msg,
                             const sensor_msgs::ImageConstPtr& depth_msg,
                             const sensor_msgs::CameraInfoConstPtr& cam_msg)
{
	std::cerr << "Got message." << std::endl;
	if (!ros::ok()) { return; }
	if (!listener->waitForTransform(mapServer->getWorldFrame(), cam_msg->header.frame_id, ros::Time(0), ros::Duration(5.0)))
	{
		ROS_WARN_STREAM("Failed to look up transform between '" << mapServer->getWorldFrame() << "' and '" << cam_msg->header.frame_id << "'.");
		return;
	}

	allMarkers.arrays.clear();

	scene = std::make_unique<Scene>();
	scene->scenario = scenario;
	scene->selfModels = selfModels;

	planner->env = scene.get();
	scene->worldFrame = mapServer->getWorldFrame();
	scene->visualize = true;

	// NB: We do this every loop because we shrink the box during the crop/filter process
	Eigen::Vector4f maxExtent(0.4f, 0.6f, 0.5f, 1);
	Eigen::Vector4f minExtent(-0.4f, -0.6f, -0.020f, 1);
	scene->minExtent = minExtent;//.head<3>().cast<double>();
	scene->maxExtent = maxExtent;//.head<3>().cast<double>();

	mostAmbNode = -1;

	PROFILE_START("Preprocess Scene");

	bool convertImages = scene->convertImages(rgb_msg, depth_msg, *cam_msg);
	if (!convertImages)
	{
		return;
	}

	bool getScene = processor->loadAndFilterScene(*scene);
	if (!getScene)
	{
		return;
	}

	PROFILE_RECORD("Preprocess Scene");

	// Get Octree
	octomap::OcTree* octree = mapServer->getOctree();
	const std::string globalFrame = mapServer->getWorldFrame();
	const std::string cameraFrame = cam_msg->header.frame_id;


	// Show robot collision
	{
		visualization_msgs::MarkerArray markers;
		int id = 0;
		for (const auto& model : scene->selfModels)
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
			m.header.stamp = ros::Time::now();//cam_msg->header.stamp;
			m.header.frame_id = globalFrame;
			markers.markers.push_back(m);
		}
		allMarkers["collision"] = markers;
		octreePub.publish(allMarkers.flatten());
	}

	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	PROFILE_START("Segment Scene");

	bool getSegmentation = processor->performSegmentation(*scene);
	if (!getSegmentation)
	{
		return;
	}

//	const long invalidGoalID = rand();
//	ObjectIndex goalSegmentID{invalidGoalID};
	{
		if (!ros::ok()) { return; }

		/////////////////////////////////////////////////////////////////
		// Search for target object
		/////////////////////////////////////////////////////////////////
		cv::Mat targetMask = targetDetector->getMask(scene->cv_rgb_cropped.image);
		imshow("Target Mask", targetMask);
		waitKey(10);

		imshow("rgb", scene->cv_rgb_cropped.image);
		waitKey(10);
		std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

		if (!scene->segInfo)
		{
			ROS_ERROR_STREAM("Segmentation failed.");
			return;
		}
		double alpha = 0.75;
		cv::Mat labelColorsMap = colorByLabel(scene->segInfo->objectness_segmentation->image);
		labelColorsMap.setTo(0, 0 == scene->segInfo->objectness_segmentation->image);
		labelColorsMap = alpha*labelColorsMap + (1.0-alpha)*scene->cv_rgb_cropped.image;
		imshow("segmentation", labelColorsMap);
		waitKey(10);

		if (segmentationPub->getNumSubscribers() > 0)
		{
			segmentationPub->publish(cv_bridge::CvImage(cam_msg->header, "bgr8", labelColorsMap).toImageMsg());
		}


//		int matchID = -1;
//		matchID = targetDetector->matchGoalSegment(targetMask, scene->segInfo->objectness_segmentation->image);
//		if (matchID >= 0)
//		{
//			scene->targetObjectID = std::make_shared<ObjectIndex>(scene->labelToIndexLookup.at((unsigned)matchID));
//			std::cerr << "**************************" << std::endl;
//			std::cerr << "Found target: " << matchID << " -> " << scene->targetObjectID->id << std::endl;
//			std::cerr << "**************************" << std::endl;
//		}
//        if (mostAmbNode >= 0)
//        {
//            ObjectIndex manidx;
//            manidx.id=(unsigned)mostAmbNode;
//            scene->targetObjectID = std::make_shared<ObjectIndex>(manidx);
//        }
        if (mostAmbNode >= 0)
        {
            auto res = scene->labelToIndexLookup.find((unsigned)mostAmbNode);
            if (res == scene->labelToIndexLookup.end())
            {
                ROS_ERROR_STREAM("Most ambiguous node '" << mostAmbNode << "' is not in the label to object map.");
            }
            else
            {
                //scene->targetObjectID = std::make_shared<ObjectIndex>(res->second);
//                std::cerr<<"Target is set to '"<< (scene->targetObjectID)->id << "' successfully. (label: "<<mostAmbNode <<")"<<std::endl;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Most ambiguous node '" << mostAmbNode << "' not received.");
        }

		if (targetPub->getNumSubscribers() > 0)
		{
			targetPub->publish(cv_bridge::CvImage(cam_msg->header, "mono8", targetMask).toImageMsg());
		}
	}

	PROFILE_RECORD("Segment Scene");

	// Clear visualization
	{
		visualization_msgs::MarkerArray ma;
		ma.markers.resize(1);
		ma.markers.front().action = visualization_msgs::Marker::DELETEALL;
		octreePub.publish(ma);
	}

	{
		visualization_msgs::MarkerArray ma = visualizeOctree(octree, globalFrame, &mapColor);
		for (visualization_msgs::Marker& m : ma.markers)
		{
			m.ns = "map";
		}
		allMarkers["map"] = ma;
		octreePub.publish(allMarkers.flatten());
	}
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	PROFILE_START("Complete Scene");

	bool getCompletion = processor->buildObjects(*scene);
	if (!getCompletion)
	{
		return;
	}

	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	std::cerr << "Completed segments: " << scene->objects.size() << __FILE__ << ": " << __LINE__ << std::endl;

//	{
//
//		scene->cloud->header.frame_id = scene->cameraFrame;
//		pcl_conversions::toPCL(ros::Time::now(), scene->cloud->header.stamp);
//		pcPub.publish(*scene->cloud);
//		std::cerr << "cloud." << std::endl;
//		sleep(3);
//
//		scene->cropped_cloud->header.frame_id = scene->cameraFrame;
//		pcl_conversions::toPCL(ros::Time::now(), scene->cropped_cloud->header.stamp);
//		pcPub.publish(*scene->cropped_cloud);
//		std::cerr << "cropped_cloud." << std::endl;
//		sleep(3);
//
//		scene->pile_cloud->header.frame_id = scene->cameraFrame;
//		pcl_conversions::toPCL(ros::Time::now(), scene->pile_cloud->header.stamp);
//		pcPub.publish(*scene->pile_cloud);
//		std::cerr << "pile_cloud." << std::endl;
//		sleep(3);
//
//		for (auto& seg : scene->segments)
//		{
//
//			seg.second->header.frame_id = scene->cameraFrame;
//			pcl_conversions::toPCL(ros::Time::now(), seg.second->header.stamp);
//			pcPub.publish(*seg.second);
//			std::cerr << seg.first.id << std::endl;
//			sleep(1);
//		}
//	}

	// Remove objects from "map" tree visualization
	for (const auto& obj : scene->objects)
	{
		auto& mapPts = allMarkers["map"];
		for (auto& m : mapPts.markers)
		{
			m.color = mapColor;
			m.colors.clear();
			m.points.erase(std::remove_if(m.points.begin(), m.points.end(),
			                              [&](const geometry_msgs::Point& p)
			                              {
				                              auto* v = obj.second->occupancy->search(p.x, p.y, p.z);
				                              return (v && v->getOccupancy() >= obj.second->occupancy->getOccupancyThres());
			                              }), m.points.end());
		}
	}

	for (const auto& obj : scene->objects)
	{
		MPS_ASSERT(scene->segments.find(obj.first) != scene->segments.end());

		ObjectLogger::logObject(obj.second.get(), experiment_dir, std::to_string(std::abs(obj.first.id)));

		if (!ros::ok()) { return; }
		const auto& subtree = obj.second->occupancy;

		std_msgs::ColorRGBA colorRGBA;
		colorRGBA.a = 1.0f;
		colorRGBA.r = rand()/(float)RAND_MAX;
		colorRGBA.g = rand()/(float)RAND_MAX;
		colorRGBA.b = rand()/(float)RAND_MAX;
		visualization_msgs::MarkerArray ma = visualizeOctree(subtree.get(), globalFrame, &colorRGBA);
		const std::string name = "completed_"+std::to_string(std::abs(obj.first.id));
		for (auto& m : ma.markers)
		{
//			m.colors.clear();
//			m.color = colorRGBA;
			m.ns = name;
			m.header.stamp = cam_msg->header.stamp;
		}
		allMarkers["completed_"+std::to_string(std::abs(obj.first.id))].markers += ma.markers;

		// Visualize approximate shape
		visualization_msgs::Marker m;
		auto approx = obj.second->approximation;
		shapes::constructMarkerFromShape(approx.get(), m, true);
		m.id = std::abs(obj.first.id);
		m.ns = "bounds";
		m.header.frame_id = globalFrame;
		m.header.stamp = cam_msg->header.stamp;
		m.pose.orientation.w = 1;
		m.color = colorRGBA; m.color.a = 0.6;
		m.frame_locked = true;
		visualization_msgs::MarkerArray ms;
		ms.markers.push_back(m);

		allMarkers["bounds"] = ms;
		octreePub.publish(allMarkers.flatten());
		waitKey(200);

	}

	PROFILE_RECORD("Complete Scene");

	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

//	int mostOccludingSegmentIdx = std::advance(occludedBySegmentCount.begin(), (int)std::distance(occludedBySegmentCount.begin(),
//		std::max_element(occludedBySegmentCount.begin(), occludedBySegmentCount.end()))).first;
//	if (goalSegmentID.id != -1) { mostOccludingSegmentIdx = goalSegmentID; }
//	visualization_msgs::MarkerArray objToMoveVis = visualizeOctree(scene->completedSegments[mostOccludingSegmentIdx].get(), globalFrame);
//	for (visualization_msgs::Marker& m : objToMoveVis.markers)
//	{
//		m.colors.clear();
//		m.color.r = 1.0;
//		m.color.a = 1.0;
//		m.scale.x *= 1.2; m.scale.y *= 1.2; m.scale.z *= 1.2;
//		m.ns = "worst";// + std::to_string(m.id);
//	}
//	octreePub.publish(objToMoveVis);
//	waitKey(10);

	std::random_device rd;
	std::mt19937 g(rd());

	{//visualize the shadow
		std_msgs::ColorRGBA shadowColor;
		shadowColor.a = 1.0f;
		shadowColor.r = 0.5f;
		shadowColor.g = 0.5f;
		shadowColor.b = 0.5f;
		visualization_msgs::MarkerArray ma = visualizeOctree(scene->occlusionTree.get(), globalFrame, &shadowColor);
		for (visualization_msgs::Marker& m : ma.markers)
		{
			m.ns = "hidden";
//			m.colors.clear();
//			m.color.a=1.0f;
//			m.color.r
        }
		allMarkers["hidden"] = ma;
		octreePub.publish(allMarkers.flatten());
	}
    std::cerr << "Published " << scene->occludedPts.size() << " points." << std::endl;

    {
        //visualize shadows with different colors
//        scene->occlusionTree->expand();
//        const auto shadow_pts = getPoints(scene->occlusionTree.get());
        pcl::PointCloud<PointT>::Ptr all_shadow_points (new pcl::PointCloud<PointT>);
        for (const auto & pt_octo : scene->occludedPts){
            PointT pt;
            pt.x=pt_octo.x();
            pt.y=pt_octo.y();
            pt.z=pt_octo.z();
            all_shadow_points->points.push_back(pt);
        }

        std::vector<pcl::PointCloud<PointT>::Ptr> segments;

        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud(all_shadow_points);

        std::vector<pcl::PointIndices> clusters;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (0.03); // 3cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (all_shadow_points);
        ec.extract (clusters);

        for (const pcl::PointIndices& cluster : clusters)
        {
            pcl::ExtractIndices<PointT> cluster_extractor;
            cluster_extractor.setInputCloud(all_shadow_points);
            cluster_extractor.setIndices(pcl::PointIndices::ConstPtr(&cluster, boost::null_deleter()));
            cluster_extractor.setNegative(false);

            pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>());
            cluster_extractor.filter(*cluster_cloud);

            segments.push_back(cluster_cloud);
        }
        int count=0;
        visualization_msgs::MarkerArray ma;
        for (const auto & segment : segments){
            visualization_msgs::Marker shadow_mk;
            for(const auto & pt : *segment){
                geometry_msgs::Point geo_pt;
                geo_pt.x=pt.x;
                geo_pt.y=pt.y;
                geo_pt.z=pt.z;
                shadow_mk.points.emplace_back(geo_pt);
            }
            double size = octree->getResolution();

            shadow_mk.header.frame_id=globalFrame;
            shadow_mk.header.stamp=ros::Time::now();
            shadow_mk.ns="shadow" ;
            shadow_mk.id=count;
            shadow_mk.type=visualization_msgs::Marker::CUBE_LIST;
            shadow_mk.scale.x=size;
            shadow_mk.scale.y=size;
            shadow_mk.scale.z=size;
            shadow_mk.color.a=1;
            shadow_mk.color.r=rand()/(float)RAND_MAX;
            shadow_mk.color.g=rand()/(float)RAND_MAX;
            shadow_mk.color.b=rand()/(float)RAND_MAX;
            count++;
            ma.markers.push_back(shadow_mk);
        }
        octreePub.publish(ma);
    }


	// Reach out and touch target

	MPS_ASSERT(!scenario->manipulators.empty());

	const std::string& robotFrame = pModel->getRootLinkName();
	tf::StampedTransform robotFrameInGlobalCoordinates;
	if (!listener->waitForTransform(robotFrame, globalFrame, ros::Time::now(), ros::Duration(1.0)))
	{
		ROS_ERROR_STREAM("Unable to compute transform from '" << globalFrame << "' to '" << robotFrame << "'.");
		return;
	}
	listener->lookupTransform(robotFrame, globalFrame, ros::Time(0), robotFrameInGlobalCoordinates);
	Eigen::Isometry3d robotTworld;
	tf::transformTFToEigen(robotFrameInGlobalCoordinates, robotTworld);
	scene->worldTrobot = robotTworld.inverse(Eigen::Isometry);


//	trajectory_msgs::JointTrajectory cmd;
//	cmd.header.frame_id = pModel->getRootLinkName();

	if (!ros::ok()) { return; }

	PROFILE_START("Planning");

	using RankedMotion = std::pair<double, std::pair<std::shared_ptr<Motion>, MotionPlanner::Introspection> >;
	auto comp = [](const RankedMotion& a, const RankedMotion& b ) { return a.first < b.first; };
	std::priority_queue<RankedMotion, std::vector<RankedMotion>, decltype(comp)> motionQueue(comp);

	scene->visualize = false;
	scene->obstructions.clear();
	scene->computeCollisionWorld();
	planner->computePlanningScene();
	auto rs = getCurrentRobotState();

	std::shared_ptr<Motion> motion;
	MotionPlanner::Introspection pushInfo;
	if (scene->targetObjectID)
	{
		ROS_WARN_STREAM("Got target Object ID.");
//		motion = planner->pick(rs, *scene->targetObjectID, scene->obstructions);
		motion = planner->samplePush(rs, &pushInfo);
		if (motion)
		{
			ROS_WARN_STREAM("Pushing motion successfully computed.");
		}
		if (!motion)
		{
			ROS_WARN_STREAM("Saw target object, but failed to compute grasp plan.");
			std::cerr << "Saw target object, but failed to compute grasp plan." << " (" << scene->obstructions.size() << " obstructions)" << std::endl;
		}
//		MPS_ASSERT(scene->obstructions.find(*scene->targetObjectID) == scene->obstructions.end());
	}

	if (scene->targetObjectID && !motion && processor->useShapeCompletion != FEATURE_AVAILABILITY::FORBIDDEN)
	{
		// We've seen the target, but can't pick it up
		bool hasVisualObstructions = planner->addVisualObstructions(*scene->targetObjectID, scene->obstructions);
		if (hasVisualObstructions)
		{
			ROS_INFO_STREAM("Added visual occlusion(s)");
		}
	}

	ros::Time planningDeadline = ros::Time::now() + ros::Duration(planning_time);

	if (!motion)
	{
//		#pragma omp parallel for private(SceneExplorer::scene) num_threads(omp_get_max_threads()/4)
		for (int i = 0; i<planning_samples; ++i)
		{
			if (ros::Time::now() > planningDeadline)
			{
				ROS_WARN_STREAM("Planning timed out. (" << planning_time << "s).");
				continue;
			}

			MotionPlanner::Introspection slideInfo;
			std::shared_ptr<Motion> motionSlide = planner->sampleSlide(rs, &slideInfo);
			if (motionSlide)
			{
				double reward = planner->reward(rs, motionSlide.get());
				#pragma omp critical
				{
					motionQueue.push({reward, {motionSlide, slideInfo}});
				}
			}

			if (ros::Time::now() > planningDeadline)
			{
				ROS_WARN_STREAM("Planning timed out. (" << planning_time << "s).");
				continue;
			}

//			MotionPlanner::Introspection pushInfo;
			std::shared_ptr<Motion> motionPush = planner->samplePush(rs, &pushInfo);
			if (motionPush)
			{
//				double reward = planner->reward(rs, motionPush.get());
				double reward = 10000000;
				#pragma omp critical
				{
					motionQueue.push({reward, {motionPush, pushInfo}});
				}
			}
		}

		if (motionQueue.empty())
		{
			ROS_WARN_STREAM("Unable to sample any valid actions for scene.");
			return;
		}
		else
		{
			std::cerr << "Found " << motionQueue.size() << " viable solutions." << std::endl;
		}
		motion = motionQueue.top().second.first;
	}

	PROFILE_RECORD("Planning");

//	for (int i = 0; i < std::min(6, (int)motionQueue.size()/3); ++i)
//	{
//		const auto res = motionQueue.top().second;
//		const ObjectSampler sample = motionQueue.top().second.second.objectSampleInfo;
//		motionQueue.pop(); motionQueue.pop(); motionQueue.pop();
//
//		visualization_msgs::MarkerArray ma;
//		visualization_msgs::Marker m;
//		m.header.frame_id = globalFrame;
//		m.header.stamp = ros::Time::now();
//		m.action = visualization_msgs::Marker::ADD;
//		m.frame_locked = true;
//		m.id = 1;
//		m.pose.orientation.w = 1.0;
//
//		// Display sample point
//		m.type = visualization_msgs::Marker::CUBE;
//		m.ns = "sample_point";
//		m.scale.x = m.scale.y = m.scale.z = octree->getResolution();
//		m.pose.position.x = sample.samplePoint.x();
//		m.pose.position.y = sample.samplePoint.y();
//		m.pose.position.z = sample.samplePoint.z();
//		m.color.r = 1.0;
//		m.color.g = 0.0;
//		m.color.b = 1.0;
//		m.color.a = 1.0;
//
//
//		ma.markers.push_back(m);
//		for (auto& a : allMarkers.arrays)
//		{
//			float alpha = 0.05;
//			if ("map" == a.first) { alpha = 0.2; }
//			if (std::string::npos != a.first.find("completed")) { alpha = 0.2; }
//			setAlpha(a.second, alpha);
//		}
//		allMarkers[m.ns] = ma;
//		octreePub.publish(allMarkers.flatten());
//		sleep(1);
//
//		// Display sample ray
//		m.type = visualization_msgs::Marker::ARROW;
//		m.ns = "sample_ray";
//		m.points.resize(2);
//		m.points[1] = m.pose.position; // Copy target location to head
//		m.points[0].x = sample.cameraOrigin.x();
//		m.points[0].y = sample.cameraOrigin.y();
//		m.points[0].z = sample.cameraOrigin.z();
//		m.pose.position.x =  m.pose.position.y = m.pose.position.z = 0.0f;
//		m.scale.x /= 2.0;
//		m.scale.y *= 2.0;
//		m.scale.z = 0.1;
//		ma.markers.front() = m;
//
//		allMarkers[m.ns] = ma;
//		octreePub.publish(allMarkers.flatten());
//		sleep(1);
//
//		// Display sample object
//		setAlpha(allMarkers["completed_"+std::to_string(std::abs(sample.id.id))], 1.0);
//		octreePub.publish(allMarkers.flatten());
//		sleep(1);
////		ma.markers.push_back(m);
//
//
//		for (auto& a : allMarkers.arrays)
//		{
//			setAlpha(a.second, 1.0);
//		}
//
////		139  69  19
//
//		// Display move(s?)
////		displayPub.publish(visualize(res.first, true));
////		sleep(3);
//	}

	waitKey(10);

	PROFILE_START("Execution");

	auto compositeAction = std::dynamic_pointer_cast<CompositeAction>(motion->action);
	// Allow some visualization time
//	ros::Duration(3.0+compositeAction->actions.size()).sleep();
	if (!ros::ok()) { return; }
	if (trajectoryClient->isServerConnected())
	{
		// For safety sake
		for (auto& manip : scenario->manipulators)
		{
			bool configured = manip->configureHardware();
			ros::Duration(0.5).sleep();
			if (!configured)
			{
				ROS_FATAL("Failed to configure hardware. Potential safety violation.");
				ros::shutdown();
				return;
			}
		}
		if (!ros::ok()) { return; }

		bool success = executeMotion(motion, *scenario->homeState);
		if (success)
		{
			static int actionCount = 0;
			std::cerr << "Actions: " << ++actionCount << std::endl;

			const auto& grasp = std::dynamic_pointer_cast<GripperCommandAction>(compositeAction->actions[compositeAction->primaryAction]);
			bool isGraspingSomething = false;
			for (const auto& manip : scenario->manipulators)
			{
				if (manip->isGrasping(getCurrentRobotState()))
				{
					isGraspingSomething = true;
				}
			}
			std::cerr << "Is currently grasping: " << isGraspingSomething << std::endl;

			if (grasp/* && isGraspingSomething*/) // Not Joint Action
			{
				PROFILE_RECORD("Execution");
				PROFILE_START("Actions Required");
				PROFILE_RECORD_DOUBLE("Actions Required", actionCount);
				PROFILE_WRITE_SUMMARY_FOR_ALL(experiment_dir + "/profile.txt");
				PROFILE_WRITE_ALL(experiment_dir + "/profile.txt");

				if (externalVideoClient.waitForExistence(ros::Duration(3)))
				{
					arm_video_recorder::TriggerVideoRecordingRequest req;
					req.filename = experiment_dir + "/external.mp4";
					req.timeout_in_sec = 3600;
					req.record = false;
					arm_video_recorder::TriggerVideoRecordingResponse resp;
					externalVideoClient.call(req, resp);
				}

//				sync.reset();
//				planner.reset();
//				processor.reset();
//				listener.reset();
//				broadcaster.reset();

				ros::shutdown();
				return;
			}

			const auto& action = std::dynamic_pointer_cast<JointTrajectoryAction>(compositeAction->actions[compositeAction->primaryAction]);
			std::map<ObjectIndex, moveit::Pose> objTrajs = followObjects(motion, tracker.get());
			for (int iter = 0; iter < 1; ++iter)
			{
				tf::Transform temp;
				for (const auto& p : objTrajs)
				{
					const std::string frameID = "next_"+std::to_string(std::abs(p.first.id));
					tf::poseEigenToTF(p.second, temp);
					broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), globalFrame, frameID));

					if (!action) { continue; }

					tf::poseEigenToTF(action->palm_trajectory.front(), temp);
					broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), globalFrame, frameID+"front"));

					tf::poseEigenToTF(action->palm_trajectory.back(), temp);
					broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), globalFrame, frameID+"back"));

					insertDirect(scene->objects.at(p.first)->points, p.second, scene->sceneOctree);

					std_msgs::ColorRGBA positiveMemoryColor;
					positiveMemoryColor.a = 1.0;
					positiveMemoryColor.r = 20/255.0f;
					positiveMemoryColor.g = 90/255.0f;
					positiveMemoryColor.b = 200/255.0f;
					visualization_msgs::MarkerArray ma = visualizeOctree(scene->objects.at(p.first)->occupancy.get(), frameID, &positiveMemoryColor);
					for (visualization_msgs::Marker& m : ma.markers)
					{
						m.ns = frameID;
					}
					allMarkers[frameID] = ma;
					octreePub.publish(allMarkers.flatten());
					ros::Duration(0.5).sleep();
				}
			}
		}
	}
	else
	{
		ros::Duration(5).sleep();
		ROS_WARN_ONCE("Trajectory action server is not connected. No trajectory will be executed.");
	}

	PROFILE_RECORD("Execution");

//	// Check if we used to see the target, but don't anymore
//	if (goalSegmentID >= 0)
//	{
//		cv::Mat targetMask = targetDetector->getMask(targetMask, seg->image);
//		if (matchID >= 0)
//		{
//
//		}
//
//	}

	waitKey(10);

}


//void SceneExplorer::manCallback(const std_msgs::Int64& msg)
//{
//    this->mostAmbNode = msg.data;
//}

moveit_msgs::DisplayTrajectory SceneExplorer::visualize(const std::shared_ptr<Motion>& motion, const bool primaryOnly)
{
	moveit_msgs::DisplayTrajectory dt;
	moveit::core::robotStateToRobotStateMsg(getCurrentRobotState(), dt.trajectory_start);
	dt.model_id = pModel->getName();
	ros::Duration totalTime(0.0);



	if (motion && motion->action && std::dynamic_pointer_cast<CompositeAction>(motion->action))
	{
		auto actions = std::dynamic_pointer_cast<CompositeAction>(motion->action);
		for (size_t idx = 0; idx < actions->actions.size(); ++idx)
		{
			if (primaryOnly && static_cast<int>(idx) != actions->primaryAction) {continue;}

			auto subTraj = std::dynamic_pointer_cast<JointTrajectoryAction>(actions->actions[idx]);
			if (subTraj)// && actions->primaryAction == static_cast<int>(idx))
			{
				moveit_msgs::RobotTrajectory drt;
				drt.joint_trajectory.joint_names = subTraj->cmd.joint_names;
				drt.joint_trajectory.points.insert(drt.joint_trajectory.points.end(), subTraj->cmd.points.begin(), subTraj->cmd.points.end());
				dt.trajectory.push_back(drt);
				totalTime += subTraj->cmd.points.back().time_from_start-subTraj->cmd.points.front().time_from_start;
			}
		}

		if (actions->primaryAction >= 0)
		{
			auto jointTraj = std::dynamic_pointer_cast<JointTrajectoryAction>(actions->actions[actions->primaryAction]);
			if (jointTraj)
			{
				int i = 0;
				for (const auto& interpPose : jointTraj->palm_trajectory)
				{
					tf::Transform temp; tf::poseEigenToTF(interpPose, temp);
					broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), scene->worldFrame, "slide_"+std::to_string(i++)));
				}
			}
		}
	}
	else
	{
		ROS_WARN("Unable to find a solution to any occluded point from any arm.");
		return dt;
	}

	return dt;
}



SceneExplorer::SceneExplorer(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    sensor_queue = std::make_unique<ros::CallbackQueue>();
	sensor_spinner = std::make_unique<ros::AsyncSpinner>(1, sensor_queue.get());
	listener = std::make_unique<tf::TransformListener>(ros::Duration(60.0));
	broadcaster = std::make_unique<tf::TransformBroadcaster>();

	auto joint_sub_options = ros::SubscribeOptions::create<sensor_msgs::JointState>("joint_states", 2, handleJointState, ros::VoidPtr(), sensor_queue.get());
	ros::Subscriber joint_sub = nh.subscribe(joint_sub_options);

	setIfMissing(pnh, "frame_id", "table_surface");
	setIfMissing(pnh, "resolution", 0.010);
	setIfMissing(pnh, "latch", false);
	setIfMissing(pnh, "filter_ground", false);
	setIfMissing(pnh, "filter_speckles", true);
	setIfMissing(pnh, "publish_free_space", false);
	setIfMissing(pnh, "sensor_model/max_range", 8.0);
	setIfMissing(pnh, "planning_samples", 25);
	setIfMissing(pnh, "planning_time", 60.0);
	setIfMissing(pnh, "track_color", "green");
	setIfMissing(pnh, "use_memory", true);
	setIfMissing(pnh, "use_completion", "optional");

	tracker = std::make_unique<SiamTracker>(listener.get());
//	tracker = std::make_unique<CudaTracker>(listener.get());
//	tracker = std::make_unique<Tracker>(listener.get());
	tracker->stopCapture();

	bool gotParam = false;

	// Get target color
	std::string track_color;
	gotParam = pnh.getParam("track_color", track_color); MPS_ASSERT(gotParam);
	std::transform(track_color.begin(), track_color.end(), track_color.begin(), ::tolower);
	if (track_color == "green")
	{
		targetDetector = std::make_unique<TargetDetector>(TargetDetector::TRACK_COLOR::GREEN);
	}
	else if (track_color == "purple")
	{
		targetDetector = std::make_unique<TargetDetector>(TargetDetector::TRACK_COLOR::PURPLE);
	}
	else
	{
		ROS_ERROR_STREAM("Color must be one of {green, purple}.");
		throw std::runtime_error("Invalid tracking color.");
	}

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
	gotParam = pnh.getParam("planning_samples", planning_samples); MPS_ASSERT(gotParam);
	gotParam = pnh.getParam("planning_time", planning_time); MPS_ASSERT(gotParam);

	nh.getParam("/experiment/id", experiment_id);
	nh.getParam("/experiment/directory", experiment_dir);

	octreePub = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1, true);
	gripperLPub = std::make_unique<realtime_tools::RealtimePublisher<victor_hardware_interface::Robotiq3FingerCommand>>(nh, "/left_arm/gripper_command", 1, false);
	gripperRPub = std::make_unique<realtime_tools::RealtimePublisher<victor_hardware_interface::Robotiq3FingerCommand>>(nh, "/right_arm/gripper_command", 1, false);
	displayPub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, false);
	auto vizPub = nh.advertise<visualization_msgs::MarkerArray>("roi", 10, true);
	pcPub = nh.advertise<pcl::PointCloud<PointT>>("segment_clouds", 1, true);
	mapServer = std::make_shared<LocalOctreeServer>(pnh);
	ros::Duration(1.0).sleep();
	completionClient = std::make_shared<VoxelCompleter>(nh);
	segmentationClient = std::make_shared<RGBDSegmenter>(nh);
	trajectoryClient = std::make_unique<SceneExplorer::TrajectoryClient>(DEFAULT_TRAJECTORY_SERVER, true);
	if (!trajectoryClient->waitForServer(ros::Duration(3.0)))
	{
		ROS_WARN("Trajectory server not connected.");
	}

	mpLoader = std::make_unique<robot_model_loader::RobotModelLoader>();
	pModel = mpLoader->getModel();

	MPS_ASSERT(!pModel->getJointModelGroupNames().empty());

	scenario = std::make_shared<Scenario>();
	scenario->loadManipulators(pModel);

	std::cerr << scenario->manipulators.front()->isGrasping(getCurrentRobotState()) << std::endl;
	std::cerr << scenario->manipulators.back()->isGrasping(getCurrentRobotState()) << std::endl;

	planner = std::make_unique<MotionPlanner>();

	if (!loadLinkMotionModels(pModel.get(), selfModels))
	{
		ROS_ERROR("Model loading failed.");
		throw std::runtime_error("Model loading failed.");
	}
	selfModels.erase("victor_base_plate"); // HACK: camera always collides
	selfModels.erase("victor_pedestal");
	selfModels.erase("victor_left_arm_mount");
	selfModels.erase("victor_right_arm_mount");
	selfModels.erase("victor_left_arm_link_0");
	selfModels.erase("victor_right_arm_link_0");

	scenario->listener = listener.get();
	scenario->broadcaster = broadcaster.get();
	scenario->mapServer = mapServer;
	scenario->completionClient = completionClient;
	scenario->segmentationClient = segmentationClient;

	processor = std::make_unique<SceneProcessor>(scenario, use_memory, useShapeCompletion);

	// Wait for joints, then set the current state as the return state
	sensor_spinner->start();
	while(ros::ok())
	{
		usleep(100000);
		if (latestJoints) { break; }
	}
	scenario->homeState = std::make_shared<robot_state::RobotState>(getCurrentRobotState());

	const std::string mocapFrame = "world_origin";
	if (listener->waitForTransform(mapServer->getWorldFrame(), mocapFrame, ros::Time(0), ros::Duration(5.0)))
	{
		tf::StampedTransform tableFrameInMocapCoordinates;
		listener->lookupTransform(mapServer->getWorldFrame(), mocapFrame, ros::Time(0), tableFrameInMocapCoordinates);
		moveit::Pose tableTmocap;
		tf::transformTFToEigen(tableFrameInMocapCoordinates, tableTmocap);

		for (int i = 0; i < 2; ++i)
		{
			auto wall = std::make_shared<shapes::Box>();
			wall->size[0] = 5;
			wall->size[1] = 0.1;
			wall->size[2] = 3;
			moveit::Pose pose = moveit::Pose::Identity();
			pose.translation() = Eigen::Vector3d(2.0, 1.0, ((0==i)?1.0:-1.0));
//			scenario->staticObstacles.push_back({wall, tableTmocap*pose});  // In Gazebo, we don't need to consider the monitors.
		}
		auto table = std::make_shared<shapes::Box>();
		table->size[0] = 0.8;
		table->size[1] = 1.2;
		table->size[2] = 0.1;
		moveit::Pose pose = moveit::Pose::Identity();
		pose.translation() = Eigen::Vector3d(0, 0, -table->size[2]/2.0);
		scenario->staticObstacles.push_back({table, pose});
	}
	else
	{
		ROS_ERROR_STREAM("Failed to look up transform between '" << mapServer->getWorldFrame() << "' and '" << "world_origin" << "'. Unable to set safety barriers");
		throw std::runtime_error("Safety reference frame failure.");
	}

	for (const auto& manip : scenario->manipulators)
	{
		std::shared_ptr<Motion> motion = std::make_shared<Motion>();
		motion->state = std::make_shared<State>();
		motion->action = std::make_shared<CompositeAction>();
		auto pregraspAction = std::make_shared<GripperCommandAction>();
		std::static_pointer_cast<CompositeAction>(motion->action)->actions.push_back(pregraspAction);

		// Pregrasp
		pregraspAction->grasp.finger_a_command.position = 0.0;
		pregraspAction->grasp.finger_a_command.speed = 1.0;
		pregraspAction->grasp.finger_a_command.force = 1.0;
		pregraspAction->grasp.finger_b_command.position = 0.0;
		pregraspAction->grasp.finger_b_command.speed = 1.0;
		pregraspAction->grasp.finger_b_command.force = 1.0;
		pregraspAction->grasp.finger_c_command.position = 0.0;
		pregraspAction->grasp.finger_c_command.speed = 1.0;
		pregraspAction->grasp.finger_c_command.force = 1.0;
		pregraspAction->grasp.scissor_command.position = 0.1;
		pregraspAction->grasp.scissor_command.speed = 1.0;
		pregraspAction->grasp.scissor_command.force = 1.0;
		pregraspAction->jointGroupName = manip->gripper->getName();

		executeMotion(motion, *scenario->homeState);
	}


	externalVideoClient = nh.serviceClient<arm_video_recorder::TriggerVideoRecording>("recording/video_recorder");
	if (!externalVideoClient.waitForExistence(ros::Duration(3)))
	{
		ROS_FATAL("External video server not connected.");
//		throw std::runtime_error("External video server not connected.");
	}
	else
	{
		arm_video_recorder::TriggerVideoRecordingRequest req;
		req.filename = experiment_dir + "/external.mp4";
		req.timeout_in_sec = 3600;
		req.record = true;
		arm_video_recorder::TriggerVideoRecordingResponse resp;
		externalVideoClient.call(req, resp);
	}

	mapColor.a = 1.0;
	mapColor.r = 1.0;
	mapColor.g = 1.0;
	mapColor.b = 0.0;

	std::string topic_prefix = "/kinect2_victor_head/hd";
//	ros::Subscriber camInfoSub = nh.subscribe("kinect2_victor_head/qhd/camera_info", 1, camera_cb);

//	message_filters::Subscriber<sensor_msgs::PointCloud2> point_sub(nh, topic_prefix+"/points", 10);
//	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, topic_prefix+"/camera_info", 10);
//
//	message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), point_sub, info_sub);
//	sync.registerCallback(cloud_cb);

	namedWindow("Target Mask", cv::WINDOW_GUI_NORMAL);
	namedWindow("rgb", cv::WINDOW_GUI_NORMAL);
	namedWindow("segmentation", cv::WINDOW_GUI_NORMAL);

	Tracker::SubscriptionOptions options(topic_prefix);

	visualization_msgs::MarkerArray ma;
	ma.markers.push_back(tracker->track_options.roi.getMarker());
	vizPub.publish(ma);

	image_transport::ImageTransport it(nh);

	segmentationPub = std::make_unique<image_transport::Publisher>(it.advertise("segmentation", 1));
	targetPub = std::make_unique<image_transport::Publisher>(it.advertise("target", 1));

	rgb_sub = std::make_unique<image_transport::SubscriberFilter>(it, options.rgb_topic, options.buffer, options.hints);
	depth_sub = std::make_unique<image_transport::SubscriberFilter>(it, options.depth_topic, options.buffer, options.hints);
	cam_sub = std::make_unique<message_filters::Subscriber<sensor_msgs::CameraInfo>>(options.nh, options.cam_topic, options.buffer);

    auto man_sub_options = ros::SubscribeOptions::create<std_msgs::Int64>("/segment_rgbd/man", 2, manCallback, ros::VoidPtr(), sensor_queue.get());
//    ros::SubscribeOptions man_sub_options;
//    man_sub_options.init<std_msgs::Int64>("/segment_rgbd/man", 1, &SceneExplorer::manCallback);
    indexofinterestSub = nh.subscribe(man_sub_options);
//    indexofinterestSub = nh.subscribe("/segment_rgbd/man", 1, &SceneExplorer::manCallback, this);

	sync = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(options.buffer), *rgb_sub, *depth_sub, *cam_sub);
	sync->registerCallback(boost::bind(&SceneExplorer::cloud_cb, this, _1, _2, _3));

}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "scene_explorer");
	ros::NodeHandle nh, pnh("~");

	auto se = std::make_unique<SceneExplorer>(nh, pnh);

	ros::spin();
//	sensor_spinner.stop();

	se.reset();

	std::cerr << "Shutting down now." << std::endl;

	return 0;
}

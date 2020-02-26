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
#include "mps_voxels/util/map_nearest.hpp"
#include "mps_voxels/image_output.h"
#include "mps_voxels/SegmentationTreeSampler.h"
#include "mps_voxels/planning/MotionPlanner.h"
#include "mps_voxels/MarkerSet.h"
#include "mps_voxels/ObjectLogger.h"
#include "mps_voxels/util/assert.h"
#include "mps_voxels/ObjectActionModel.h"
#include "mps_voxels/logging/DataLog.h"
#include "mps_voxels/logging/log_sensor_history.h"
#include "mps_voxels/logging/log_segmentation_info.h"
#include "mps_voxels/logging/log_cv_roi.h"
#include "mps_voxels/logging/log_occupancy_data.h"
#include "mps_voxels/ParticleFilter.h"
#include <mps_voxels/JaccardMatch.h>
#include "mps_voxels/visualization/visualize_occupancy.h"
#include "mps_voxels/visualization/visualize_bounding_spheres.h"

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

#include "mps_voxels/VoxelRegion.h"

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

using namespace mps;

sensor_msgs::JointState::ConstPtr latestJoints;
std::mutex joint_mtx;

void handleJointState(const sensor_msgs::JointState::ConstPtr& js)
{
	std::lock_guard<std::mutex> lk(joint_mtx);
	latestJoints = js;
	ROS_DEBUG_ONCE("Joint joints!");
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

const std::string DEFAULT_TRAJECTORY_SERVER = "follow_joint_trajectory";

class SceneExplorer
{
public:
	using SyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>;
	using TrajectoryClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

	std::shared_ptr<LocalOctreeServer> mapServer;
	std::shared_ptr<VoxelCompleter> completionClient;
	std::shared_ptr<RGBDSegmenter> segmentationClient;
	std::unique_ptr<SensorHistorian> historian;
	std::unique_ptr<Tracker> sparseTracker;
	std::unique_ptr<DenseTracker> denseTracker;
	std::unique_ptr<TargetDetector> targetDetector;
	std::unique_ptr<TrajectoryClient> trajectoryClient;
	std::unique_ptr<ParticleFilter> particleFilter;
	std::unique_ptr<realtime_tools::RealtimePublisher<victor_hardware_interface::Robotiq3FingerCommand>> gripperLPub;
	std::unique_ptr<realtime_tools::RealtimePublisher<victor_hardware_interface::Robotiq3FingerCommand>> gripperRPub;
	ros::Publisher visualPub;
	ros::Publisher displayPub;
	ros::Publisher pcPub;

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
	std::shared_ptr<Scene> scene;
	std::unique_ptr<MotionPlanner> planner;
	std::map<std::string, std::shared_ptr<MotionModel>> selfModels;

	double planning_time = 60.0;
	int planning_samples = 25;
	ros::ServiceClient externalVideoClient;
	MarkerSet allMarkers;

	std_msgs::ColorRGBA mapColor;

	std::default_random_engine rng;

	SceneExplorer(ros::NodeHandle& nh, ros::NodeHandle& pnh);

	robot_state::RobotState getCurrentRobotState();
	trajectory_msgs::JointTrajectory fillMissingJointTrajectories(const trajectory_msgs::JointTrajectory& trajIn);
	std::map<ObjectIndex, moveit::Pose> followObjects(const std::shared_ptr<Motion>& motion, const SensorHistoryBuffer& buffer);
	bool executeMotion(const std::shared_ptr<Motion>& motion, const robot_state::RobotState& recoveryState);

	void cloud_cb (const sensor_msgs::ImageConstPtr& rgb_msg,
	               const sensor_msgs::ImageConstPtr& depth_msg,
	               const sensor_msgs::CameraInfoConstPtr& cam_msg);

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

std::map<ObjectIndex, moveit::Pose> SceneExplorer::followObjects(const std::shared_ptr<Motion>& motion, const SensorHistoryBuffer& buffer)
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
	auto msgStart = find_nearest(buffer.joint, buffer.rgb.begin()->first)->second;
	auto msgEnd = find_nearest(buffer.joint, buffer.rgb.rbegin()->first)->second;

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

	std::unique_ptr<CaptureGuard> captureGuard; ///< RAII-style stopCapture() for various exit paths

	if (compositeAction->primaryAction >= 0)
	{
		captureGuard = std::make_unique<CaptureGuard>(historian.get());
		auto cache = std::dynamic_pointer_cast<CachingRGBDSegmenter>(scenario->segmentationClient);
		if (cache) { cache->cache.clear(); }
		historian->startCapture();
	}

	for (size_t a = 0; a < compositeAction->actions.size(); ++a)
	{
		std::cerr << "Start Action " << a << std::endl;
		const auto& action = compositeAction->actions[a];


		if ((compositeAction->primaryAction >= 0) && (a == 1 || compositeAction->primaryAction == static_cast<int>(a) || a == compositeAction->actions.size()-1))
		{
			historian->startCapture();
		}
		else
		{
			historian->stopCapture();
		}

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
			goal.goal_time_tolerance = ros::Duration(10.0);

			std::cerr << "Sending joint trajectory." << std::endl;

			auto res = trajectoryClient->sendGoalAndWait(goal, ros::Duration(10.0) + totalTime, ros::Duration(1.0));
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
				ROS_INFO_STREAM("Sending left gripper command.");
				gripperPub = gripperLPub.get();
			}
			else if (gripAction->jointGroupName.find("right") != std::string::npos)
			{
				ROS_INFO_STREAM("Sending right gripper command.");
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
	} // end of these sequence of actions

	if (compositeAction->primaryAction >= 0)
	{
		std::cerr << "Start Historian!" << std::endl;
		historian->stopCapture();

		/////////////////////////////////////////////
		//// log historian->buffer & scene->segInfo & scene->roi
		/////////////////////////////////////////////
		bool ifLog = false;
		if (ifLog)
		{
			std::string worldname = "experiment_world_02_21";
			{
				std::cerr << "start logging historian->buffer" << std::endl;
				DataLog logger(scenario->experiment->experiment_dir + "/explorer_buffer_" + worldname + ".bag");
				logger.activeChannels.insert("buffer");
				logger.log<SensorHistoryBuffer>("buffer", historian->buffer);
				std::cerr << "Successfully logged." << std::endl;
			}
			{
				std::cerr << "start logging scene->segInfo" << std::endl;
				DataLog logger(scenario->experiment->experiment_dir + "/explorer_segInfo_" + worldname + ".bag");
				logger.activeChannels.insert("segInfo");
				logger.log<SegmentationInfo>("segInfo", *scene->segInfo);
				std::cerr << "Successfully logged." << std::endl;
			}
		}
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

	scene = std::make_shared<Scene>();
	scene->scenario = scenario;
	scene->selfModels = selfModels;

	{
		visualization_msgs::MarkerArray ma;
		ma.markers.push_back(sparseTracker->track_options.roi.getMarker());
//		ma.markers.back().ns = "roi";
		allMarkers["roi"] = ma;
	}

	PROFILE_START("Preprocess Scene");

	bool convertImages = scene->convertImages(rgb_msg, depth_msg, *cam_msg);
	if (!convertImages)
	{
		return;
	}

	bool getScene = SceneProcessor::loadAndFilterScene(*scene, scenario->transformBuffer);
	if (!getScene)
	{
		return;
	}

	PROFILE_RECORD("Preprocess Scene");

	if (scenario->shouldVisualize("pointclouds"))
	{
		scene->cloud->header.frame_id = scene->cameraFrame;
		pcl_conversions::toPCL(ros::Time::now(), scene->cloud->header.stamp);
		pcPub.publish(*scene->cloud);
		std::cerr << "cloud." << std::endl;
		sleep(3);

		scene->cropped_cloud->header.frame_id = scene->cameraFrame;
		pcl_conversions::toPCL(ros::Time::now(), scene->cropped_cloud->header.stamp);
		pcPub.publish(*scene->cropped_cloud);
		std::cerr << "cropped_cloud." << std::endl;
		sleep(3);

		scene->pile_cloud->header.frame_id = scene->cameraFrame;
		pcl_conversions::toPCL(ros::Time::now(), scene->pile_cloud->header.stamp);
		pcPub.publish(*scene->pile_cloud);
		std::cerr << "pile_cloud." << std::endl;
		sleep(3);
	}

	// Get Octree
	octomap::OcTree* octree = mapServer->getOctree();
	const std::string globalFrame = mapServer->getWorldFrame();
	const std::string cameraFrame = cam_msg->header.frame_id;


	// Show robot collision
	if (scenario->shouldVisualize("collision"))
	{
		std_msgs::Header header; header.frame_id = globalFrame; header.stamp = ros::Time::now();
		this->allMarkers["collision"] = mps::visualize(this->selfModels, header, scenario->rng());
		this->visualPub.publish(this->allMarkers.flatten());
	}

	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	PROFILE_START("Segment Scene");

	bool getSegmentation = SceneProcessor::callSegmentation(*scene);
	if (!getSegmentation)
	{
		std::cerr << "Segmentation generation failed." << std::endl;
		return;
	}

	bool getOcclusion = SceneProcessor::computeOcclusions(*scene);
	if (!getOcclusion)
	{
		std::cerr << "Occlusion generation failed." << std::endl;
		return;
	}

	/////////////////////////////////////////////
	//// Sample state particles
	/////////////////////////////////////////////
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;


	if (particleFilter->particles.empty())
	{
		particleFilter->initializeParticles(scene);
	}
	else
	{
//		particleFilter->computeAndApplyActionModel(historian->buffer, sparseTracker, denseTracker);
		particleFilter->applyMeasurementModel(scene);
	}

	{
		double bestWeight = -std::numeric_limits<double>::infinity();
		int bestParticle = -1;
		for (size_t p = 0; p < particleFilter->particles.size(); ++p)
		{
			if (particleFilter->particles[p].weight > bestWeight)
			{
				bestWeight = particleFilter->particles[p].weight;
				bestParticle = static_cast<int>(p);
			}
		}
		scene->bestGuess = particleFilter->particles[bestParticle].state;
	}

	if (scenario->shouldVisualize("particles"))
	{
		for (const auto& particle : particleFilter->particles)
		{

			// Clear visualization
			{
				visualization_msgs::MarkerArray ma;
				ma.markers.resize(1);
				ma.markers.front().action = visualization_msgs::Marker::DELETEALL;
				visualPub.publish(ma);
			}

			IMSHOW("segmentation", colorByLabel(particle.state->segInfo->objectness_segmentation->image));

			IMSHOW("orig_segmentation", colorByLabel(scene->segInfo->objectness_segmentation->image));
			WAIT_KEY(0);

			{
				visualization_msgs::MarkerArray ma = visualizeOctree(octree, globalFrame);
				for (visualization_msgs::Marker& m : ma.markers)
				{
					m.ns = "";
				}
				allMarkers["map"] = ma;
				visualPub.publish(allMarkers.flatten());
			}

			{
				std_msgs::Header header;
				header.frame_id = globalFrame;
				header.stamp = cam_msg->header.stamp;
				this->allMarkers["particle"] = mps::visualize(*particle.state, header, scenario->rng());
				this->visualPub.publish(this->allMarkers.flatten());
			}
		}
	}

	{
		bool clearedVolume = SceneProcessor::removeAccountedForOcclusion(scenario.get(), scene->occludedPts, scene->occlusionTree, *scene->bestGuess);
		if (!clearedVolume)
		{
			std::cerr << "Occlusion deduction failed." << std::endl;
			return;
		}
	}




//	const long invalidGoalID = rand();
//	ObjectIndex goalSegmentID{invalidGoalID};
	{
		if (!ros::ok()) { return; }

		/////////////////////////////////////////////////////////////////
		// Search for target object
		/////////////////////////////////////////////////////////////////
		cv::Mat targetMask = targetDetector->getMask(scene->cv_rgb_cropped.image);
		IMSHOW("target_mask", targetMask);
		WAIT_KEY(10);

		IMSHOW("rgb", scene->cv_rgb_cropped.image);
		WAIT_KEY(10);
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
		IMSHOW("segmentation", labelColorsMap);
		WAIT_KEY(10);


//		int matchID = -1;
//		matchID = targetDetector->matchGoalSegment(targetMask, scene->segInfo->objectness_segmentation->image);
//		if (matchID >= 0)
//		{
//			scene->bestGuess->targetObjectID = std::make_shared<ObjectIndex>(scene->bestGuess->labelToIndexLookup.at((unsigned)matchID));
//			std::cerr << "**************************" << std::endl;
//			std::cerr << "Found target: " << matchID << " -> " << scene->bestGuess->targetObjectID->id << std::endl;
//			std::cerr << "**************************" << std::endl;
//		}
//        if (mostAmbNode >= 0)
//        {
//            ObjectIndex manidx;
//            manidx.id=(unsigned)mostAmbNode;
//            scene->bestGuess->targetObjectID = std::make_shared<ObjectIndex>(manidx);
//        }
//        if (mostAmbNode >= 0)
//        {
//            auto res = scene->bestGuess->labelToIndexLookup.find((unsigned)mostAmbNode);
//            if (res == scene->bestGuess->labelToIndexLookup.end())
//            {
//                ROS_ERROR_STREAM("Most ambiguous node '" << mostAmbNode << "' is not in the label to object map.");
//            }
//            else
//            {
//                //scene->bestGuess->targetObjectID = std::make_shared<ObjectIndex>(res->second);
////                std::cerr<<"Target is set to '"<< (scene->bestGuess->targetObjectID)->id << "' successfully. (label: "<<mostAmbNode <<")"<<std::endl;
//            }
//        }
//        else
//        {
//            ROS_ERROR_STREAM("Most ambiguous node '" << mostAmbNode << "' not received.");
//        }

		IMSHOW("target", targetMask);
	}

	PROFILE_RECORD("Segment Scene");

	// Clear visualization
	{
		visualization_msgs::MarkerArray ma;
		ma.markers.resize(1);
		ma.markers.front().action = visualization_msgs::Marker::DELETEALL;
		visualPub.publish(ma);
	}

	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	PROFILE_START("Complete Scene");



	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

/*
	// Remove objects from "map" tree visualization
	for (const auto& obj : scene->bestGuess->objects)
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

 */

	PROFILE_RECORD("Complete Scene");

	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

//	{//visualize the shadow
//		std_msgs::ColorRGBA shadowColor;
//		shadowColor.a = 1.0f;
//		shadowColor.r = 0.5f;
//		shadowColor.g = 0.5f;
//		shadowColor.b = 0.5f;
//		visualization_msgs::MarkerArray ma = visualizeOctree(scene->occlusionTree.get(), globalFrame, &shadowColor);
//		for (visualization_msgs::Marker& m : ma.markers)
//		{
//			m.ns = "hidden";
////			m.colors.clear();
////			m.color.a=1.0f;
////			m.color.r
//        }
//		allMarkers["hidden"] = ma;
//		visualPub.publish(allMarkers.flatten());
//	}
//    std::cerr << "Published " << scene->occludedPts.size() << " points." << std::endl;

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

		std::uniform_real_distribution<> uni(0.0, std::nextafter(1.0, std::numeric_limits<double>::max()));

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
            shadow_mk.color.r=uni(scenario->rng());
            shadow_mk.color.g=uni(scenario->rng());
            shadow_mk.color.b=uni(scenario->rng());
            count++;
            ma.markers.push_back(shadow_mk);
        }
        visualPub.publish(ma);
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

	scene->bestGuess->obstructions.clear();
	planner = std::make_unique<MotionPlanner>(scenario, scene->bestGuess);
	auto rs = getCurrentRobotState();

	std::shared_ptr<Motion> motion;
	MotionPlanner::Introspection pushInfo;
	if (scene->bestGuess->targetObjectID)
	{
		ROS_WARN_STREAM("Got target Object ID.");
//		motion = planner->pick(rs, *scene->bestGuess->targetObjectID, scene->bestGuess->obstructions);
		motion = planner->samplePush(rs, &pushInfo);
		if (motion)
		{
			ROS_WARN_STREAM("Pushing motion successfully computed.");
		}
		if (!motion)
		{
			ROS_WARN_STREAM("Saw target object, but failed to compute grasp plan.");
			std::cerr << "Saw target object, but failed to compute grasp plan." << " (" << scene->bestGuess->obstructions.size() << " obstructions)" << std::endl;
		}
//		MPS_ASSERT(scene->bestGuess->obstructions.find(*scene->bestGuess->targetObjectID) == scene->bestGuess->obstructions.end());
	}

	if (scene->bestGuess->targetObjectID && !motion && scenario->useShapeCompletion != FEATURE_AVAILABILITY::FORBIDDEN)
	{
		// We've seen the target, but can't pick it up
		bool hasVisualObstructions = planner->addVisualObstructions(*scene->bestGuess->targetObjectID, scene->bestGuess->obstructions);
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
				double reward = 10000000; // TODO: remove this line to use grasp
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

	WAIT_KEY(10);

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
				PROFILE_WRITE_SUMMARY_FOR_ALL(scenario->experiment->experiment_dir + "/profile.txt");
				PROFILE_WRITE_ALL(scenario->experiment->experiment_dir + "/profile.txt");

				if (externalVideoClient.waitForExistence(ros::Duration(3)))
				{
					arm_video_recorder::TriggerVideoRecordingRequest req;
					req.filename = scenario->experiment->experiment_dir + "/external.mp4";
					req.timeout_in_sec = 3600;
					req.record = false;
					arm_video_recorder::TriggerVideoRecordingResponse resp;
					externalVideoClient.call(req, resp);
				}

				ros::shutdown();
				return;
			}

			const auto& action = std::dynamic_pointer_cast<JointTrajectoryAction>(compositeAction->actions[compositeAction->primaryAction]);
			std::map<ObjectIndex, moveit::Pose> objTrajs = followObjects(motion, historian->buffer);
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

					insertDirect(scene->bestGuess->objects.at(p.first)->points, p.second, scene->sceneOctree);

					std_msgs::ColorRGBA positiveMemoryColor;
					positiveMemoryColor.a = 1.0;
					positiveMemoryColor.r = 20/255.0f;
					positiveMemoryColor.g = 90/255.0f;
					positiveMemoryColor.b = 200/255.0f;
					visualization_msgs::MarkerArray ma = visualizeOctree(scene->bestGuess->objects.at(p.first)->occupancy.get(), frameID, &positiveMemoryColor);
					for (visualization_msgs::Marker& m : ma.markers)
					{
						m.ns = frameID;
					}
					allMarkers[frameID] = ma;
					visualPub.publish(allMarkers.flatten());
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


	particleFilter->computeAndApplyActionModel(historian->buffer, sparseTracker, denseTracker);

	WAIT_KEY(10);

}

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
					broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), scenario->worldFrame, "slide_"+std::to_string(i++)));
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
	auto experiment = std::make_shared<Experiment>(nh, pnh);
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

	visualPub = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1, true);
	gripperLPub = std::make_unique<realtime_tools::RealtimePublisher<victor_hardware_interface::Robotiq3FingerCommand>>(nh, "/left_arm/gripper_command", 1, false);
	gripperRPub = std::make_unique<realtime_tools::RealtimePublisher<victor_hardware_interface::Robotiq3FingerCommand>>(nh, "/right_arm/gripper_command", 1, false);
	displayPub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, false);
	pcPub = pnh.advertise<pcl::PointCloud<PointT>>("segment_clouds", 1, true);
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

	scenario = std::make_shared<Scenario>(use_memory, useShapeCompletion);
	scenario->loadManipulators(pModel);

	std::cerr << scenario->manipulators.front()->isGrasping(getCurrentRobotState()) << std::endl;
	std::cerr << scenario->manipulators.back()->isGrasping(getCurrentRobotState()) << std::endl;

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

	scenario->listener = std::make_shared<tf2_ros::TransformListener>(scenario->transformBuffer);//listener.get();
	scenario->transformBuffer.setUsingDedicatedThread(true);
	scenario->broadcaster = broadcaster.get();
	scenario->mapServer = mapServer;
	scenario->completionClient = completionClient;
	scenario->segmentationClient = segmentationClient;
	scenario->experiment = experiment;

	scenario->minExtent = Eigen::Vector4f::Ones();
	scenario->maxExtent = Eigen::Vector4f::Ones();
	pnh.getParam("roi/min/x", scenario->minExtent.x());
	pnh.getParam("roi/min/y", scenario->minExtent.y());
	pnh.getParam("roi/min/z", scenario->minExtent.z());
	pnh.getParam("roi/max/x", scenario->maxExtent.x());
	pnh.getParam("roi/max/y", scenario->maxExtent.y());
	pnh.getParam("roi/max/z", scenario->maxExtent.z());
	pnh.getParam("frame_id", scenario->worldFrame);

	double resolution = 0.010;
	pnh.getParam("resolution", resolution);
	mps::VoxelRegion::vertex_descriptor dims = roiToVoxelRegion(resolution,
	                                                            scenario->minExtent.head<3>().cast<double>(),
	                                                            scenario->maxExtent.head<3>().cast<double>());
	particleFilter = std::make_unique<ParticleFilter>(scenario, dims, resolution,
	                                                  scenario->minExtent.head<3>().cast<double>(),
	                                                  scenario->maxExtent.head<3>().cast<double>(), 5);

	Tracker::TrackingOptions opts;
	opts.roi.minExtent = {scenario->minExtent.x(), scenario->minExtent.y(), scenario->minExtent.z()};
	opts.roi.maxExtent = {scenario->maxExtent.x(), scenario->maxExtent.y(), scenario->maxExtent.z()};
#ifdef USE_CUDA_SIFT
	sparseTracker = std::make_unique<CudaTracker>(opts);
#else
	sparseTracker = std::make_unique<Tracker>(opts);
	sparseTracker->track_options.featureRadius = 200.0f;
	sparseTracker->track_options.pixelRadius = 1000.0f;
	sparseTracker->track_options.meterRadius = 1.0f;
#endif
	denseTracker = std::make_unique<SiamTracker>();
	historian = std::make_unique<SensorHistorian>();
	historian->stopCapture();

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
		req.filename = scenario->experiment->experiment_dir + "/external.mp4";
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

	NAMED_WINDOW("target_mask", cv::WINDOW_GUI_NORMAL);
	NAMED_WINDOW("rgb", cv::WINDOW_GUI_NORMAL);
	NAMED_WINDOW("segmentation", cv::WINDOW_GUI_NORMAL);
	NAMED_WINDOW("orig_segmentation", cv::WINDOW_GUI_NORMAL);

	SensorHistorian::SubscriptionOptions options(topic_prefix);

	image_transport::ImageTransport it(nh);

	rgb_sub = std::make_unique<image_transport::SubscriberFilter>(it, options.rgb_topic, options.queue_size, options.hints);
	depth_sub = std::make_unique<image_transport::SubscriberFilter>(it, options.depth_topic, options.queue_size, options.hints);
	cam_sub = std::make_unique<message_filters::Subscriber<sensor_msgs::CameraInfo>>(options.nh, options.cam_topic, options.queue_size);

	sync = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(options.queue_size), *rgb_sub, *depth_sub, *cam_sub);
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

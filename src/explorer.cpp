//
// Created by arprice on 7/24/18.
//

#include "mps_voxels/Manipulator.h"
#include "mps_voxels/MotionModel.h"
#include "mps_voxels/Tracker.h"
#include "mps_voxels/TargetDetector.h"
#include "mps_voxels/LocalOctreeServer.h"
#include "mps_voxels/octree_utils.h"
#include "mps_voxels/pointcloud_utils.h"
#include "mps_voxels/image_utils.h"
#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/shape_utils.h"
#include "mps_voxels/planning/MotionPlanner.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayTrajectory.h>

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
//#include <pcl/visualization/cloud_viewer.h>

#include <geometric_shapes/shape_operations.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/ros.h>

#include <algorithm>
#include <memory>
#include <random>
#include <queue>


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

//MotionModel* matchModel(std::shared_ptr<octomap::OcTree> subtree, std::map<std::string, std::unique_ptr<MotionModel>> motionModels)
//{
//
//}
/*
double fitModels(const std::vector<std::pair<Tracker::Vector, Tracker::Vector>>& flow, const int K = 1)
{
	const int N = static_cast<int>(flow.size());
	std::vector<RigidMotionModel> mms;
	std::vector<MotionModel::MotionParameters> thetas;
	std::vector<unsigned int> indices(N);
	std::iota(indices.begin(), indices.end(), 0);
	std::random_shuffle(indices.begin(), indices.end());
	for (int s = 0; s < K; ++s)
	{
		RigidMotionModel mm;
		auto v = flow[indices[s]].first;
		mm.localTglobal.translation() = Eigen::Vector3d(v.x(), v.y(), v.z());
		thetas.emplace_back(MotionModel::MotionParameters::Zero(RigidMotionModel::MOTION_PARAMETERS_DIMENSION));
	}

	Eigen::MatrixXd assignmentProbability(N, K); // Probability that element n belongs to model k
	Eigen::VectorXi assignment(N);
	for (int n = 0; n < N; ++n)
	{
		for (int k = 0; k < K; ++k)
		{
			Tracker::Vector vExpect = mms[k].expectedVelocity(flow[n].first, thetas[k]);
			assignmentProbability(n, k) = mms[k].membershipLikelihood(flow[n].first);// + MotionModel::logLikelihood(exp(-(flow[n].second-vExpect).squaredNorm()));
		}
	}

	double L = NAN;
	for (int iter = 0; iter < 25; ++iter)
	{
		// Assign points to models
		Eigen::VectorXi assignmentCount = Eigen::VectorXi::Zero(K);
		double Lmax = 0.0;
		for (int n = 0; n < N; ++n)
		{
			Eigen::MatrixXf::Index max_index;
			assignmentProbability.row(n).maxCoeff(&max_index);
			assignment(n) = (int)max_index;
			assignmentCount((int)max_index)++;
			Lmax += assignmentProbability(n, (int)max_index);
		}

		if (Lmax - L > 0.1)
		{
			return Lmax;
		}
		L = Lmax;

		// M-Step
		for (int k = 0; k < K; ++k)
		{
			Eigen::Matrix3Xd ptsA(3, assignmentCount(k));
			Eigen::Matrix3Xd ptsB(3, assignmentCount(k));
			int idx = 0;
			for (int n = 0; n < N; ++n)
			{
				if (assignment(n) == k)
				{
					ptsA.col(idx) = flow[n].first;
					ptsB.col(idx) = flow[n].first + flow[n].second;
					++idx;
				}
			}

			thetas[k] = estimateRigidTransform3D(ptsA, ptsB);
		}

		// E-Step
		for (int n = 0; n < N; ++n)
		{
			for (int k = 0; k < K; ++k)
			{
				Tracker::Vector vExpect = mms[k].expectedVelocity(flow[n].first, thetas[k]);
				assignmentProbability(n, k) = mms[k].membershipLikelihood(flow[n].first) + MotionModel::logLikelihood(exp(-(flow[n].second-vExpect).squaredNorm()));
			}
		}
	}

	std::cerr << "BIC: " << log(N)*K - L << std::endl;

	return L;
}
*/


using TrajectoryClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

std::shared_ptr<LocalOctreeServer> mapServer;
std::shared_ptr<VoxelCompleter> completionClient;
std::shared_ptr<RGBDSegmenter> segmentationClient;
std::unique_ptr<Tracker> tracker;
std::unique_ptr<TargetDetector> targetDetector;
std::shared_ptr<TrajectoryClient> trajectoryClient;
std::unique_ptr<realtime_tools::RealtimePublisher<victor_hardware_interface::Robotiq3FingerCommand>> gripperLPub;
std::unique_ptr<realtime_tools::RealtimePublisher<victor_hardware_interface::Robotiq3FingerCommand>> gripperRPub;
ros::Publisher octreePub;
ros::Publisher displayPub;
std::unique_ptr<image_transport::Publisher> segmentationPub;
std::unique_ptr<image_transport::Publisher> targetPub;
std::shared_ptr<tf::TransformListener> listener;
std::shared_ptr<tf::TransformBroadcaster> broadcaster;
robot_model::RobotModelPtr pModel;
//std::vector<std::shared_ptr<Manipulator>> manipulators;
std::unique_ptr<Scene> planningEnvironment;
std::shared_ptr<MotionPlanner> planner;
sensor_msgs::JointState::ConstPtr latestJoints;
//std::map<std::string, std::shared_ptr<MotionModel>> motionModels;
std::mutex joint_mtx;

void handleJointState(const sensor_msgs::JointState::ConstPtr& js)
{
	std::lock_guard<std::mutex> lk(joint_mtx);
	latestJoints = js;
	ROS_DEBUG_ONCE("Joint joints!");
}

robot_state::RobotState getCurrentRobotState()
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

bool executeMotion(const std::shared_ptr<Motion>& motion, const robot_state::RobotState& recoveryState)
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
			tracker->startCapture();
		}

		auto armAction = std::dynamic_pointer_cast<JointTrajectoryAction>(action);
		if (armAction)
		{
			control_msgs::FollowJointTrajectoryGoal goal;
			goal.trajectory = armAction->cmd;

			std::cerr << "Sending joint trajectory." << std::endl;

			auto res = trajectoryClient->sendGoalAndWait(goal, ros::Duration(30.0) + totalTime);
			if (!res.isDone() || (res.state_!=actionlib::SimpleClientGoalState::SUCCEEDED))
			{
				if (!res.isDone())
				{
					ROS_ERROR_STREAM("Trajectory client timed out.");
				}
				else if (res.state_!=actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					ROS_ERROR_STREAM("Trajectory following failed: " << res.text_);
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
				throw std::runtime_error("Unknown gripper.");
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
//			tracker->track();
		}
	}
	return true;
}

void cloud_cb (const sensor_msgs::ImageConstPtr& rgb_msg,
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

	// NB: We do this every loop because we shrink the box during the crop/filter process
	Eigen::Vector4f maxExtent(0.4f, 0.6f, 0.5f, 1);
	Eigen::Vector4f minExtent(-0.4f, -0.6f, -0.020f, 1);
	planningEnvironment->minExtent = minExtent;//.head<3>().cast<double>();
	planningEnvironment->maxExtent = maxExtent;//.head<3>().cast<double>();

	bool convertImages = planningEnvironment->convertImages(rgb_msg, depth_msg, *cam_msg);
	if (!convertImages)
	{
		return;
	}

	bool getScene = planningEnvironment->loadAndFilterScene();
	if (!getScene)
	{
		return;
	}

	// Get Octree
	octomap::OcTree* octree = mapServer->getOctree();
	const std::string globalFrame = mapServer->getWorldFrame();
	const std::string cameraFrame = cam_msg->header.frame_id;

	// Clear visualization
	{
		visualization_msgs::MarkerArray ma;
		ma.markers.resize(1);
		ma.markers.front().action = visualization_msgs::Marker::DELETEALL;
		octreePub.publish(ma);
	}

	// Show robot collision
	{
		visualization_msgs::MarkerArray markers;
		int id = 0;
		for (const auto& model : planningEnvironment->selfModels)
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
			m.header.stamp = cam_msg->header.stamp;
			m.header.frame_id = globalFrame;
			markers.markers.push_back(m);
		}
		octreePub.publish(markers);
	}

	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	bool getSegmentation = planningEnvironment->performSegmentation();
	if (!getSegmentation)
	{
		return;
	}

	int goalSegmentID = -1;
	{
		if (!ros::ok()) { return; }

		/////////////////////////////////////////////////////////////////
		// Search for target object
		/////////////////////////////////////////////////////////////////
		cv::Mat targetMask = targetDetector->getMask(planningEnvironment->cv_rgb_cropped.image);
		imshow("Target Mask", targetMask);
		waitKey(10);

		imshow("rgb", planningEnvironment->cv_rgb_cropped.image);
		waitKey(10);
		std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

		if (!planningEnvironment->segInfo)
		{
			ROS_ERROR_STREAM("Segmentation failed.");
			return;
		}
		double alpha = 0.75;
		cv::Mat labelColorsMap = colorByLabel(planningEnvironment->segInfo->objectness_segmentation->image);
		labelColorsMap = alpha*labelColorsMap + (1.0-alpha)*planningEnvironment->cv_rgb_cropped.image;
		imshow("segmentation", labelColorsMap);
		waitKey(10);

		if (segmentationPub->getNumSubscribers() > 0)
		{
			segmentationPub->publish(cv_bridge::CvImage(cam_msg->header, "bgr8", labelColorsMap).toImageMsg());
		}


		int matchID = -1;
		matchID = targetDetector->matchGoalSegment(targetMask, planningEnvironment->segInfo->objectness_segmentation->image);
		if (matchID >= 0)
		{
			goalSegmentID = planningEnvironment->labelToIndexLookup.at((unsigned)matchID);
			std::cerr << "**************************" << std::endl;
			std::cerr << "Found target: " << matchID << " -> " << goalSegmentID << std::endl;
			std::cerr << "**************************" << std::endl;
		}

		if (targetPub->getNumSubscribers() > 0)
		{
			targetPub->publish(cv_bridge::CvImage(cam_msg->header, "mono8", targetMask).toImageMsg());
		}
	}

	bool getCompletion = planningEnvironment->completeShapes();
	if (!getCompletion)
	{
		return;
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
	std::cerr << __FILE__ << ": " << __LINE__ << std::endl;

	for (const auto& subtree : planningEnvironment->completedSegments)
	{
		if (!ros::ok()) { return; }

		std_msgs::ColorRGBA colorRGBA;
		colorRGBA.a = 1.0f;
		colorRGBA.r = rand()/(float)RAND_MAX;
		colorRGBA.g = rand()/(float)RAND_MAX;
		colorRGBA.b = rand()/(float)RAND_MAX;
		visualization_msgs::MarkerArray occupiedNodesVis = visualizeOctree(subtree.get(), globalFrame, &colorRGBA);
		for (auto& m : occupiedNodesVis.markers)
		{
//			m.colors.clear();
//			m.color = colorRGBA;
			m.ns = "completed_"+std::to_string(planningEnvironment->completedSegments.size());
		}
		octreePub.publish(occupiedNodesVis);

		// Visualize approximate shape
		visualization_msgs::Marker m;
		auto approx = approximateShape(subtree.get());
		planningEnvironment->approximateSegments.push_back(approx);
		shapes::constructMarkerFromShape(approx.get(), m, true);
		m.id = planningEnvironment->completedSegments.size();
		m.ns = "bounds";
		m.header.frame_id = globalFrame;
		m.header.stamp = ros::Time::now();
		m.pose.orientation.w = 1;
		m.color = colorRGBA; m.color.a = 0.6;
		m.frame_locked = true;
		visualization_msgs::MarkerArray ms;
		ms.markers.push_back(m);
		octreePub.publish(ms);
		waitKey(10);

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
	std::vector<int> occludedBySegmentCount(planningEnvironment->segments.size(), 0);
	for (int i = 0; i < static_cast<int>(planningEnvironment->segments.size()); ++i)
	{
		const pcl::PointCloud<PointT>::Ptr& pc = planningEnvironment->segments[i];
		for (const PointT& pt : *pc)
		{
			Eigen::Vector3f worldPt = planningEnvironment->worldTcamera.cast<float>()*pt.getVector3fMap();
			octomap::point3d coord = octree->keyToCoord(octree->coordToKey(octomap::point3d(worldPt.x(), worldPt.y(), worldPt.z())));
			coordToSegment.insert({coord, i});
		}
	}

	octomap::point3d cameraOrigin((float)planningEnvironment->worldTcamera.translation().x(),
		                          (float)planningEnvironment->worldTcamera.translation().y(),
		                          (float)planningEnvironment->worldTcamera.translation().z());
	#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(planningEnvironment->occludedPts.size()); ++i)
	{
		const auto& pt_world = planningEnvironment->occludedPts[i];
		octomap::point3d collision;
		bool hit = octree->castRay(cameraOrigin, pt_world-cameraOrigin, collision);
		assert(hit); _unused(hit);
		collision = octree->keyToCoord(octree->coordToKey(collision)); // regularize

		const auto& iter = coordToSegment.find(collision);
		if (iter != coordToSegment.end())
		{
			#pragma omp critical
			{
				occludedBySegmentCount[iter->second]++;
				planningEnvironment->coordToObject.insert({pt_world, iter->second});
				planningEnvironment->objectToShadow[iter->second].push_back(pt_world);
			}
		}
	}
	planningEnvironment->surfaceCoordToObject = coordToSegment;

	int mostOccludingSegmentIdx = (int)std::distance(occludedBySegmentCount.begin(),
		std::max_element(occludedBySegmentCount.begin(), occludedBySegmentCount.end()));
	if (goalSegmentID >= 0) { mostOccludingSegmentIdx = goalSegmentID; }
	visualization_msgs::MarkerArray objToMoveVis = visualizeOctree(planningEnvironment->completedSegments[mostOccludingSegmentIdx].get(), globalFrame);
	for (visualization_msgs::Marker& m : objToMoveVis.markers)
	{
		m.colors.clear();
		m.color.r = 1.0;
		m.color.a = 1.0;
		m.scale.x *= 1.2; m.scale.y *= 1.2; m.scale.z *= 1.2;
		m.ns = "worst";// + std::to_string(m.id);
	}
	octreePub.publish(objToMoveVis);
	waitKey(10);

	std::random_device rd;
	std::mt19937 g(rd());

	{
		visualization_msgs::MarkerArray occupiedNodesVis = visualizeOctree(planningEnvironment->occlusionTree.get(), globalFrame);
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
	std::cerr << "Published " << planningEnvironment->occludedPts.size() << " points." << std::endl;

	// Reach out and touch target

	assert(!planningEnvironment->manipulators.empty());

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
	planningEnvironment->worldTrobot = robotTworld.inverse(Eigen::Isometry);


//	trajectory_msgs::JointTrajectory cmd;
//	cmd.header.frame_id = pModel->getRootLinkName();

	if (!ros::ok()) { return; }

	using RankedMotion = std::pair<double, std::shared_ptr<Motion>>;
	auto comp = [](const RankedMotion& a, const RankedMotion& b ) { return a.first < b.first; };
	std::priority_queue<RankedMotion, std::vector<RankedMotion>, decltype(comp)> motionQueue(comp);

	planningEnvironment->visualize = false;
	planningEnvironment->obstructions.clear();
	planningEnvironment->computeCollisionWorld();
	planner->computePlanningScene();
	auto rs = getCurrentRobotState();

	std::shared_ptr<Motion> motion;
	if (goalSegmentID >= 0)
	{
		motion = planner->pick(rs, goalSegmentID, planningEnvironment->obstructions);
		if (!motion)
		{
			ROS_WARN_STREAM("Saw target object, but failed to compute grasp plan.");
			std::cerr << "Saw target object, but failed to compute grasp plan." << " (" << planningEnvironment->obstructions.size() << " obstructions)" << std::endl;
		}
	}

	if (!motion)
	{
		#pragma omp parallel for private(planningEnvironment)
		for (int i = 0; i<50; ++i)
		{
			std::shared_ptr<Motion> motionSlide = planner->sampleSlide(rs);
			if (motionSlide)
			{
				double reward = planner->reward(rs, motionSlide.get());
				#pragma omp critical
				{
					motionQueue.push({reward, motionSlide});
				}
			}

			std::shared_ptr<Motion> motionPush = planner->samplePush(rs);
			if (motionPush)
			{
				double reward = planner->reward(rs, motionPush.get());
				#pragma omp critical
				{
					motionQueue.push({reward, motionPush});
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
		motion = motionQueue.top().second;
	}

	moveit_msgs::DisplayTrajectory dt;
	moveit::core::robotStateToRobotStateMsg(getCurrentRobotState(), dt.trajectory_start);
	dt.model_id = pModel->getName();
	ros::Duration totalTime(0.0);

	if (!ros::ok()) { return; }

	if (motion && motion->action && std::dynamic_pointer_cast<CompositeAction>(motion->action))
	{
		auto actions = std::dynamic_pointer_cast<CompositeAction>(motion->action);
		for (size_t idx = 0; idx < actions->actions.size(); ++idx)
		{
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
					broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), globalFrame, "slide_"+std::to_string(i++)));
				}
			}
		}
	}
	else
	{
		ROS_WARN("Unable to find a solution to any occluded point from any arm.");
		return;
	}

	displayPub.publish(dt);

	waitKey(10);

	auto compositeAction = std::dynamic_pointer_cast<CompositeAction>(motion->action);
	// Allow some visualization time
//	ros::Duration(3.0+compositeAction->actions.size()).sleep();
	if (!ros::ok()) { return; }
	if (trajectoryClient->isServerConnected())
	{
		// For safety sake
		for (auto& manip : planningEnvironment->scenario->manipulators)
		{
			manip->configureHardware();
			ros::Duration(0.5).sleep();
		}
		if (!ros::ok()) { return; }

		bool success = executeMotion(motion, getCurrentRobotState());
		if (success)
		{
			static int actionCount = 0;
			std::cerr << "Actions: " << ++actionCount << std::endl;
		}
	}
	else
	{
		ros::Duration(5).sleep();
	}

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


//using SyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo>;
using SyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "scene_explorer");
	ros::NodeHandle nh, pnh("~");

	ros::CallbackQueue sensor_queue;
	ros::AsyncSpinner sensor_spinner(1, &sensor_queue);

	listener = std::make_shared<tf::TransformListener>();
	broadcaster = std::make_shared<tf::TransformBroadcaster>();

	auto joint_sub_options = ros::SubscribeOptions::create<sensor_msgs::JointState>("joint_states", 2, handleJointState, ros::VoidPtr(), &sensor_queue);
	ros::Subscriber joint_sub = nh.subscribe(joint_sub_options);

	setIfMissing(pnh, "frame_id", "table_surface");
	setIfMissing(pnh, "resolution", 0.010);
	setIfMissing(pnh, "latch", false);
	setIfMissing(pnh, "filter_ground", false);
	setIfMissing(pnh, "filter_speckles", true);
	setIfMissing(pnh, "publish_free_space", false);
	setIfMissing(pnh, "sensor_model/max_range", 8.0);
	setIfMissing(pnh, "track_color", "green");

	tracker = std::make_unique<Tracker>();
	tracker->stopCapture();

	// Get target color
	std::string track_color;
	pnh.getParam("track_color", track_color);
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

	octreePub = nh.advertise<visualization_msgs::MarkerArray>("occluded_points", 1, true);
	gripperLPub = std::make_unique<realtime_tools::RealtimePublisher<victor_hardware_interface::Robotiq3FingerCommand>>(nh, "/left_arm/gripper_command", 1, false);
	gripperRPub = std::make_unique<realtime_tools::RealtimePublisher<victor_hardware_interface::Robotiq3FingerCommand>>(nh, "/right_arm/gripper_command", 1, false);
//	commandPub = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_trajectory", 1, false);
	displayPub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, false);
	auto vizPub = nh.advertise<visualization_msgs::MarkerArray>("roi", 10, true);
	mapServer = std::make_shared<LocalOctreeServer>(pnh);
	ros::Duration(1.0).sleep();
//	mapClient = std::make_shared<OctreeRetriever>(nh);
	completionClient = std::make_shared<VoxelCompleter>(nh);
	segmentationClient = std::make_shared<RGBDSegmenter>(nh);
	trajectoryClient = std::make_shared<TrajectoryClient>("follow_joint_trajectory", true);
	if (!trajectoryClient->waitForServer(ros::Duration(3.0)))
	{
		ROS_WARN("Trajectory server not connected.");
	}

	std::shared_ptr<robot_model_loader::RobotModelLoader> mpLoader
		= std::make_shared<robot_model_loader::RobotModelLoader>();
	pModel = mpLoader->getModel();

	std::shared_ptr<Scenario> scenario = std::make_shared<Scenario>();

	planningEnvironment = std::make_unique<Scene>();
	planningEnvironment->scenario = scenario;

	if (!loadLinkMotionModels(pModel.get(), planningEnvironment->selfModels))
	{
		ROS_ERROR("Model loading failed.");
	}
	planningEnvironment->selfModels.erase("victor_base_plate"); // HACK: camera always collides
	planningEnvironment->selfModels.erase("victor_pedestal");
	planningEnvironment->selfModels.erase("victor_left_arm_mount");
	planningEnvironment->selfModels.erase("victor_right_arm_mount");

	assert(!pModel->getJointModelGroupNames().empty());

	scenario->loadManipulators(pModel);

	planner = std::make_shared<MotionPlanner>();
	planner->env = planningEnvironment.get();
	planner->objectSampler.env = planningEnvironment.get();
	planningEnvironment->worldFrame = mapServer->getWorldFrame();
	planningEnvironment->visualize = true;
	scenario->listener = listener;
	scenario->broadcaster = broadcaster;
	scenario->mapServer = mapServer;
	scenario->completionClient = completionClient;
	scenario->segmentationClient = segmentationClient;

	const std::string mocapFrame = "world_origin";
	if (listener->waitForTransform(mapServer->getWorldFrame(), mocapFrame, ros::Time(0), ros::Duration(5.0)))
	{
		tf::StampedTransform tableFrameInMocapCoordinates;
		listener->lookupTransform(mapServer->getWorldFrame(), mocapFrame, ros::Time(0), tableFrameInMocapCoordinates);
		Eigen::Affine3d tableTmocap;
		tf::transformTFToEigen(tableFrameInMocapCoordinates, tableTmocap);

		for (int i = 0; i < 2; ++i)
		{
			auto wall = std::make_shared<shapes::Box>();
			wall->size[0] = 5;
			wall->size[1] = 0.1;
			wall->size[2] = 3;
			Eigen::Affine3d pose = Eigen::Affine3d::Identity();
			pose.translation() = Eigen::Vector3d(2.0, 1.0, ((0==i)?1.0:-1.0));
			scenario->staticObstacles.push_back({wall, tableTmocap*pose});
		}
		auto table = std::make_shared<shapes::Box>();
		table->size[0] = 0.8;
		table->size[1] = 1.2;
		table->size[2] = 0.1;
		Eigen::Affine3d pose = Eigen::Affine3d::Identity();
		pose.translation() = Eigen::Vector3d(0, 0, -table->size[2]/2.0);
		scenario->staticObstacles.push_back({table, pose});
	}
	else
	{
		ROS_ERROR_STREAM("Failed to look up transform between '" << mapServer->getWorldFrame() << "' and '" << "world_origin" << "'. Unable to set safety barriers");
		throw std::runtime_error("Safety reference frame failure.");
	}

	std::string topic_prefix = "/kinect2_victor_head/qhd";
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

	auto rgb_sub = std::make_unique<image_transport::SubscriberFilter>(it, options.rgb_topic, options.buffer, options.hints);
	auto depth_sub = std::make_unique<image_transport::SubscriberFilter>(it, options.depth_topic, options.buffer, options.hints);
	auto cam_sub = std::make_unique<message_filters::Subscriber<sensor_msgs::CameraInfo>>(options.nh, options.cam_topic, options.buffer);

	auto sync = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(options.buffer), *rgb_sub, *depth_sub, *cam_sub);
	sync->registerCallback(cloud_cb);

//	ros::Subscriber sub = nh.subscribe ("kinect2_roof/qhd/points", 1, cloud_cb);


	//(new octomap::OcTree(d));


	sensor_spinner.start();
	ros::spin();
	sensor_spinner.stop();

//	spinner.spin();

	return 0;
}
//
// Created by arprice on 9/4/18.
//

#include "mps_voxels/planning/MotionPlanner.h"
#include "mps_voxels/octree_utils.h"
#include "mps_voxels/LocalOctreeServer.h"

#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/StdVector>

#include <tf_conversions/tf_eigen.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/circulator.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/min_quadrilateral_2.h>
#include <boost/heap/priority_queue.hpp>

#define _unused(x) ((void)(x))

const std::string PlanningEnvironment::CLUTTER_NAME = "clutter";


std::priority_queue<MotionPlanner::RankedPose, std::vector<MotionPlanner::RankedPose>, ComparePoses> getGraspPoses(const octomap::OcTree* tree)
{
	using K =         CGAL::Exact_predicates_inexact_constructions_kernel;
	using Point_2 =   K::Point_2;
	using Polygon_2 = CGAL::Polygon_2<K>;
	using RankedPose = MotionPlanner::RankedPose;

	// Get a point cloud representing the (shape-completed) segment
	octomap::point3d_collection segmentPoints = getPoints(tree);
	double minZ = std::numeric_limits<double>::infinity();
	double maxZ = -std::numeric_limits<double>::infinity();
	for (const auto& pt : segmentPoints)
	{
		minZ = std::min(minZ, (double)pt.z());
		maxZ = std::max(maxZ, (double)pt.z());
	}

	std::vector<Point_2> cgal_points;
	for (const auto& pt : segmentPoints)
	{
		cgal_points.emplace_back(Point_2(pt.x(), pt.y()));
	}

	Polygon_2 hull;
//	std::vector<Point_2> hull;
	CGAL::convex_hull_2(cgal_points.begin(), cgal_points.end(), std::back_inserter(hull)); // O(nh) vs O(nlogn) ch_bykat
	assert(hull.is_convex());

//	auto comp = [](const RankedPose& a, const RankedPose& b ) { return a.first < b.first; };
	std::priority_queue<RankedPose, std::vector<RankedPose>, ComparePoses> graspPoses;

	const size_t nPts = hull.size();
	for (size_t i = 0; i < nPts; ++i)
	{
		const Point_2& a_cgal = hull.vertex(i);
		const Point_2& b_cgal = hull.vertex((i+1)%nPts);
		const Eigen::Vector2d a(a_cgal.x(), a_cgal.y());
		const Eigen::Vector2d b(b_cgal.x(), b_cgal.y());
		const Eigen::Vector2d v = (hull.is_clockwise_oriented() ? b-a : a-b);
		const Eigen::Vector2d vHat = v.normalized(); ///< Edge direction
		const Eigen::Vector2d nHat(vHat.y(), -vHat.x()); ///< Normal direction to edge

		double minV = std::numeric_limits<double>::infinity();
		double maxV = -std::numeric_limits<double>::infinity();
		double maxN = -std::numeric_limits<double>::infinity();
		Eigen::Vector2d maxNpt;

		for (size_t j_step = 0; j_step < nPts-2; ++j_step)
		{
			size_t j = (i+j_step)%nPts;
			const Point_2& p_cgal = hull.vertex(j);
			const Eigen::Vector2d p = Eigen::Vector2d(p_cgal.x(), p_cgal.y())-a;

			double projN = nHat.dot(p);
			double projV = vHat.dot(p);

			minV = std::min(minV, projV);
			maxV = std::max(maxV, projV);

			assert(projN > -1e-6);
			if (projN > maxN)
			{
				maxN = projN;
				maxNpt = p;
			}
		}

//		const Eigen::Vector2d c = maxNpt - (maxN/2.0*nHat) + a; ///< Midpoint between active edge and distal point
		const Eigen::Vector2d c = (a+b)/2.0 + (maxN/2.0*nHat);// + vHat/2.0;
		Eigen::Affine3d graspPose = Eigen::Affine3d::Identity();
		graspPose.translation().head<2>() = c;
		graspPose.translation().z() = maxZ;
		graspPose.linear().topLeftCorner<2,2>() << nHat, vHat;

		graspPoses.push({v.norm(), graspPose});
	}

	return graspPoses;
}

collision_detection::WorldPtr
PlanningEnvironment::computeCollisionWorld()
{
	auto world = std::make_shared<collision_detection::World>();

	Pose robotTworld = worldTrobot.inverse(Eigen::Isometry);

	for (const auto& obstacle : staticObstacles)
	{
		world->addToObject(CLUTTER_NAME, obstacle.first, robotTworld * obstacle.second);
	}

	// Use aliasing shared_ptr constructor
	world->addToObject(CLUTTER_NAME,
	                   std::make_shared<shapes::OcTree>(std::shared_ptr<octomap::OcTree>(std::shared_ptr<octomap::OcTree>{}, sceneOctree)),
	                   robotTworld);

	for (const auto& completedSegment : completedSegments)
	{
		world->addToObject(CLUTTER_NAME, std::make_shared<shapes::OcTree>(completedSegment), robotTworld);
	}

//	for (auto& approxSegment : approximateSegments)
//	{
//		world->addToObject(CLUTTER_NAME, approxSegment, robotTworld);
//	}

	collisionWorld = world;
	return world;
}


bool
ObjectSampler::sampleObject(int& id, Pose& pushFrame) const
{
	octomap::point3d_collection* shadowPoints = &env->occludedPts;
//	if (!env->obstructions.empty() && ((rand()/RAND_MAX) < 0.8))
//	{
//		std::uniform_int_distribution<size_t> distr(0, env->obstructions.size()-1);
//		std::set<int>::const_iterator it(env->obstructions.begin());
//
//		size_t idx = distr(env->rng);
//
//		std::advance(it, idx);
//		id = *it;
//
//		shadowPoints = &(env->objectToShadow[id]);
//	}

	const int N = static_cast<int>(shadowPoints->size());
	std::vector<unsigned int> indices(N);
	std::iota(indices.begin(), indices.end(), 0);
	std::shuffle(indices.begin(), indices.end(), env->rng);

	for (const unsigned int index : indices)
	{
		const auto& pt_original = (*shadowPoints)[index];

		// Get a randomly selected shadowed point
		Eigen::Vector3d pt(pt_original.x(), pt_original.y(), pt_original.z());
		if (pt.z()<0.05) { continue; }

		octomap::point3d cameraOrigin((float) env->worldTcamera.translation().x(), (float) env->worldTcamera.translation().y(),
		                              (float) env->worldTcamera.translation().z());
		octomath::Vector3 ray = pt_original-cameraOrigin;
		octomap::point3d collision;
		bool hit = env->sceneOctree->castRay(cameraOrigin, ray, collision, true);
		assert(hit); _unused(hit);
		collision = env->sceneOctree->keyToCoord(env->sceneOctree->coordToKey(collision)); // regularize

		Eigen::Vector3d gHat = -Eigen::Vector3d::UnitZ();
		Eigen::Vector3d pHat = -Eigen::Map<const Eigen::Vector3f>(&ray(0)).cast<double>().cross(gHat).normalized();
		Eigen::Vector3d nHat = gHat.cross(pHat).normalized();

		pushFrame.linear() << pHat.normalized(), nHat.normalized(), gHat.normalized();
		pushFrame.translation() = Eigen::Map<const Eigen::Vector3f>(&collision(0)).cast<double>();

		if (env->visualize)
		{
			// Display occluded point and push frame
			tf::Transform t = tf::Transform::getIdentity();
			Eigen::Vector3d pt_camera = env->worldTcamera.inverse(Eigen::Isometry)*pt;
			t.setOrigin({pt_camera.x(), pt_camera.y(), pt_camera.z()});
			env->broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), env->cameraFrame, "occluded_point"));

			tf::poseEigenToTF(pushFrame, t);
			env->broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), env->worldFrame, "push_frame"));
		}

		const auto& iter = env->surfaceCoordToObject.find(collision);
		if (iter==env->surfaceCoordToObject.end()) { continue; }
		int pushSegmentID = iter->second;

		id = pushSegmentID;
		return true;
	}

	return false;
}

double
MotionPlanner::reward(const Motion* motion) const
{
	const size_t nClusters = motion->state->poses.size();
	assert(nClusters == env->completedSegments.size());

	Eigen::Matrix3Xd centroids(3, nClusters);
	for (size_t i = 0; i < nClusters; ++i)
	{
		assert(env->completedSegments[i].get());
		assert(env->completedSegments[i]->size() > 0);
		octomap::point3d_collection segmentPoints = getPoints(env->completedSegments[i].get());
		Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
		for (const auto& pt : segmentPoints)
		{
			centroid += Eigen::Vector3d(pt.x(), pt.y(), pt.z());
		}
		centroid /= static_cast<double>(segmentPoints.size());

		centroids.col(i) = motion->state->poses[i] * centroid;
	}

	Eigen::Vector3d centroid = centroids.rowwise().sum()/static_cast<double>(nClusters);
	Eigen::Matrix3Xd deltas = centroids.colwise() - centroid;
	double spread = deltas.colwise().squaredNorm().sum();

	int changeCount = 0;

	// Compute change in umbras
	for (size_t objIdx = 0; objIdx < motion->state->poses.size(); ++objIdx)
	{
		const Pose& worldTobj_prime = motion->state->poses[objIdx];
		if (worldTobj_prime.matrix().isIdentity(1e-6)) { continue; }

		const std::shared_ptr<octomap::OcTree>& segment = env->completedSegments[objIdx];

//		const Pose worldTobj_init = Pose::Identity();
		const Pose worldTcamera_prime = /*worldTobj_init * */ worldTobj_prime.inverse(Eigen::Isometry) * env->worldTcamera;

		octomap::point3d cameraOrigin((float) worldTcamera_prime.translation().x(), (float) worldTcamera_prime.translation().y(),
		                              (float) worldTcamera_prime.translation().z());
//
//		octomap::point3d min = segment->getBBXMin(), max = segment->getBBXMax();
//		for (int i = 0; i < 3; ++i)
//		{
//			min(i) = std::min(min(i), (float)worldTcamera_prime.translation()[i]);
//			max(i) = std::max(max(i), (float)worldTcamera_prime.translation()[i]);
//		}
//		setBBox(min, max, segment);

		// Check occlusion of points attached to body
		const std::vector<octomap::point3d>& umbra = env->objectToShadow[objIdx];
		for (size_t i = 0; i < umbra.size(); ++i)
		{
			// Check occlusion of points attached to world
			octomath::Vector3 ray = umbra[i]-cameraOrigin;
			octomap::point3d collision;
			bool occluded = segment->castRay(cameraOrigin, ray, collision, true, 2.0);
			if (!occluded)
			{
				// This world-attached point is now seen
				++changeCount;
			}

			// TODO: Verify
			Eigen::Vector3d pt_moved = worldTobj_prime * Eigen::Vector3d(umbra[i].x(), umbra[i].y(), umbra[i].z());
			ray = octomap::point3d((float)pt_moved.x(), (float)pt_moved.y(), (float)pt_moved.z())-cameraOrigin;
			occluded = segment->castRay(cameraOrigin, ray, collision, true, 2.0);
			if (!occluded)
			{
				// This body-attached point is now seen
				++changeCount;
			}

		}

		std::cerr << "Revealed " << changeCount << " voxels" << std::endl;
	}

	// Compute direction of push/slide
	double dir = 0;
	auto compositeAction = std::dynamic_pointer_cast<CompositeAction>(motion->action);
	auto jointTraj = std::dynamic_pointer_cast<JointTrajectoryAction>(compositeAction->actions[compositeAction->primaryAction]);
	if (jointTraj)
	{
		if (jointTraj->palm_trajectory.size() > 1)
		{
			const Pose& begin = jointTraj->palm_trajectory.front();
			const Pose& end = jointTraj->palm_trajectory.back();
			Eigen::Vector3d cb = (begin.translation()-centroid).normalized(); ///< Vector from centroid to beginning of move
			Eigen::Vector3d ce = end.translation()-centroid; ///< Vector of move
			dir = cb.dot(ce);
		}
	}

	return spread + 3.0*dir + (double)changeCount/100000.0;
}

planning_scene::PlanningSceneConstPtr
MotionPlanner::computePlanningScene()
{
	planningScene = std::make_shared<planning_scene::PlanningScene>(env->manipulators.front()->pModel, env->computeCollisionWorld());
	return planningScene;
}

collision_detection::AllowedCollisionMatrix
MotionPlanner::gripperEnvironmentACM(const std::shared_ptr<Manipulator>& manipulator) const
{
	collision_detection::AllowedCollisionMatrix acm;
//	acm.setEntry(true); // Only check things we're explicitly requesting
	std::set<std::string> gripperLinks(manipulator->gripper->getLinkModelNames().begin(), manipulator->gripper->getLinkModelNames().end());
	for (const auto& linkName : manipulator->pModel->getLinkModelNames())
	{
		// Not sure why this is necessary, but it seems to be
		if (gripperLinks.find(linkName) == gripperLinks.end())
		{
			acm.setDefaultEntry(linkName, true);
		}
	}
	for (const auto& link1Name : gripperLinks)
	{
		for (const auto& link2Name : gripperLinks)
		{
			if (link1Name == link2Name) { continue; }

			acm.setEntry(link1Name, link2Name, true);
		}
	}
	acm.setEntry(env->CLUTTER_NAME, manipulator->gripper->getLinkModelNames(), false);

	return acm;
}

bool
MotionPlanner::gripperEnvironmentCollision(const std::shared_ptr<Manipulator>& manipulator, const robot_state::RobotState& collisionState) const
{
	assert(env->collisionWorld);

	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
//	collision_request.contacts = true;

	collision_detection::AllowedCollisionMatrix acm = gripperEnvironmentACM(manipulator);
	planningScene->checkCollision(collision_request, collision_result, collisionState, acm);

	return collision_result.collision;
//	if (collision_result.collision)
//	{
//		for (size_t i = 0; i < manipulator->gripper->getVariableCount(); ++i)
//		{
//			std::cerr << manipulator->gripper->getVariableNames()[i] << ": " << gripperJoints[i] << std::endl;
//		}
//		for (const auto& p : collision_result.contacts)
//		{
//			std::cerr << p.first.first << " X " << p.first.second << std::endl;
//			for (const auto& c : p.second)
//			{
//				std::cerr << c.pos.transpose() << std::endl;
//			}
//		}
//		return true;
//	}
//	return false;
}

std::shared_ptr<Motion>
MotionPlanner::samplePush(const robot_state::RobotState& robotState) const
{
	Pose robotTworld = env->worldTrobot.inverse(Eigen::Isometry);
	State objectState{State::Poses(env->completedSegments.size(), State::Pose::Identity())};

	// Get an object to slide
	int slideSegmentID = -1;
	Pose pushFrame;
	if (!objectSampler.sampleObject(slideSegmentID, pushFrame))
	{
		return std::shared_ptr<Motion>();
	}
	const octomap::OcTree* tree = env->completedSegments[slideSegmentID].get();

//	trajectory_msgs::JointTrajectory cmd;
	PoseSequence pushGripperFrames(2, Eigen::Affine3d::Identity());
	octomap::point3d_collection segmentPoints = getPoints(tree);

	Eigen::Vector3d minProj = Eigen::Vector3d::Ones() * std::numeric_limits<float>::max();
	Eigen::Vector3d maxProj = Eigen::Vector3d::Ones() * std::numeric_limits<float>::lowest();
	for (const auto& segPt : segmentPoints)
	{
		Eigen::Vector3d pushFramePt = pushFrame.linear().transpose() * Eigen::Map<const Eigen::Vector3f>(&segPt(0)).cast<double>();
		for (int d=0; d < 3; ++d)
		{
			minProj[d] = std::min(minProj[d], pushFramePt[d]);
			maxProj[d] = std::max(maxProj[d], pushFramePt[d]);
		}
	}

	const double GRIPPER_HALFWIDTH = 4.0*2.54/100.0;
	pushGripperFrames[0].translation() = pushFrame.linear() * Eigen::Vector3d(maxProj.x(), (maxProj.y()+minProj.y())/2.0, maxProj.z())
	                                     + GRIPPER_HALFWIDTH*Eigen::Vector3d::UnitZ();
	pushGripperFrames[1].translation() = pushFrame.linear() * Eigen::Vector3d(minProj.x(), (maxProj.y()+minProj.y())/2.0, maxProj.z())
	                                     + GRIPPER_HALFWIDTH*Eigen::Vector3d::UnitZ();
	pushGripperFrames[0].linear() = pushFrame.linear()
	                                *(Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitY())
	                                  *Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ())).matrix();
	pushGripperFrames[1].linear() = pushFrame.linear()
	                                *(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())
	                                  *Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ())).matrix();

	// Display occluded point and push frame
	if (env->visualize)
	{
		tf::Transform t = tf::Transform::getIdentity();
		tf::poseEigenToTF(pushGripperFrames[0], t);
		env->broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), env->worldFrame, "push_gripper_frame_0"));
		tf::poseEigenToTF(pushGripperFrames[1], t);
		env->broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), env->worldFrame, "push_gripper_frame_1"));
	}

	// Shuffle manipulators (without shuffling underlying array)
	std::vector<unsigned int> manip_indices(env->manipulators.size());
	std::iota(manip_indices.begin(), manip_indices.end(), 0);
	std::shuffle(manip_indices.begin(), manip_indices.end(), env->rng);

	// Shuffle push directions (without shuffling underlying array)
	std::vector<unsigned int> push_indices(pushGripperFrames.size());
	std::iota(push_indices.begin(), push_indices.end(), 0);
	std::shuffle(push_indices.begin(), push_indices.end(), env->rng);

	std::uniform_int_distribution<int> stepDistr(5, 15);

	const int INTERPOLATE_STEPS = 25;
	const int TRANSIT_INTERPOLATE_STEPS = 50;
//	const int SAMPLE_ATTEMPTS = 100;
//	const double PALM_DISTANCE = 0.035;
	const double APPROACH_HEIGHT = 0.25;
//	const double TABLE_BUFFER = 0.20;
//	const double Z_SAFETY_HEIGHT = 0.18;

	for (unsigned int push_idx : push_indices)
	{
		const auto& pushGripperFrame = pushGripperFrames[push_idx];
		const double stepSize = 0.015;
		const int nSteps = stepDistr(env->rng);
		PoseSequence pushTrajectory;
		for (int s = -15; s < nSteps; ++s)
		{
			Eigen::Affine3d step = pushGripperFrame;
			step.translation() += s*stepSize*pushGripperFrame.linear().col(2);
			pushTrajectory.push_back(step);
		}

		for (unsigned int manip_idx : manip_indices)
		{
			auto& manipulator = env->manipulators[manip_idx];

			std::vector<std::vector<double>> sln;

			sln = manipulator->IK(pushTrajectory.back(), robotTworld, robotState);
			if (sln.empty()) { continue; }

			sln = manipulator->IK(pushTrajectory.front(), robotTworld, robotState);
			if (sln.empty()) { continue; }

			// Check whether the hand collides with the scene
			{
				robot_state::RobotState collisionState(robotState);
				collisionState.setJointGroupPositions(manipulator->arm, sln.front());
				collisionState.update();
				if (gripperEnvironmentCollision(manipulator, collisionState))
				{
					continue;
				}
			}

			// Allocate the action sequence needed for a slide
			std::shared_ptr<CompositeAction> compositeAction = std::make_shared<CompositeAction>();
			auto pregraspAction = std::make_shared<GripperCommandAction>();  compositeAction->actions.push_back(pregraspAction);
			auto transitAction = std::make_shared<JointTrajectoryAction>();  compositeAction->actions.push_back(transitAction);
			auto approachAction = std::make_shared<JointTrajectoryAction>(); compositeAction->actions.push_back(approachAction);
			auto slideAction = std::make_shared<JointTrajectoryAction>();    compositeAction->actions.push_back(slideAction);
			auto retractAction = std::make_shared<JointTrajectoryAction>();  compositeAction->actions.push_back(retractAction);
			auto homeAction = std::make_shared<JointTrajectoryAction>();     compositeAction->actions.push_back(homeAction);
			compositeAction->primaryAction = 3;

			// Lay out the Cartesian Path
			Eigen::Affine3d gripperApproachPose = pushTrajectory.front();
			gripperApproachPose.translation() += APPROACH_HEIGHT*Eigen::Vector3d::UnitZ();

			Eigen::Affine3d gripperRetractPose = pushTrajectory.back();
			gripperRetractPose.translation() += APPROACH_HEIGHT*Eigen::Vector3d::UnitZ();

			PoseSequence approachTrajectory, slideTrajectory, retractTrajectory, fullTrajectory;

			manipulator->interpolate(gripperApproachPose, pushTrajectory.front(), approachTrajectory, INTERPOLATE_STEPS/2);
			manipulator->interpolate(pushTrajectory.front(), pushTrajectory.back(), slideTrajectory, INTERPOLATE_STEPS);
			manipulator->interpolate(pushTrajectory.back(), gripperRetractPose, retractTrajectory, INTERPOLATE_STEPS/2);

			fullTrajectory.reserve(approachTrajectory.size()+slideTrajectory.size()+retractTrajectory.size());
			fullTrajectory.insert(fullTrajectory.end(), approachTrajectory.begin(), approachTrajectory.end());
			fullTrajectory.insert(fullTrajectory.end(), slideTrajectory.begin(), slideTrajectory.end());
			fullTrajectory.insert(fullTrajectory.end(), retractTrajectory.begin(), retractTrajectory.end());

			trajectory_msgs::JointTrajectory cmd;
			if (manipulator->cartesianPath(fullTrajectory, robotTworld, robotState, cmd))
			{
				std::shared_ptr<Motion> motion = std::make_shared<Motion>();
				motion->state = std::make_shared<State>(objectState);
				motion->action = compositeAction;

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
				pregraspAction->jointGroupName = manipulator->gripper->getName();

				// Move to start
				robot_state::RobotState approachState(robotState);
				approachState.setJointGroupPositions(manipulator->arm, cmd.points.front().positions);
				approachState.updateCollisionBodyTransforms();
				if (!manipulator->interpolate(robotState, approachState, transitAction->cmd, TRANSIT_INTERPOLATE_STEPS, planningScene))
				{
					continue;
				}

				// Approach
				approachAction->palm_trajectory = approachTrajectory;
				approachAction->cmd.joint_names = cmd.joint_names;
				approachAction->cmd.points.insert(approachAction->cmd.points.end(), cmd.points.begin(), cmd.points.begin()+approachTrajectory.size()+1);

				// Slide
				slideAction->palm_trajectory = slideTrajectory;
				slideAction->cmd.joint_names = cmd.joint_names;
				slideAction->cmd.points.insert(slideAction->cmd.points.end(), cmd.points.begin()+approachTrajectory.size(), cmd.points.begin()+approachTrajectory.size()+slideTrajectory.size()+1);
				motion->state->poses[slideSegmentID] = pushTrajectory.front().inverse(Eigen::Isometry) * pushTrajectory.back();

				// Retract
				retractAction->palm_trajectory = retractTrajectory;
				retractAction->cmd.joint_names = cmd.joint_names;
				retractAction->cmd.points.insert(retractAction->cmd.points.end(), cmd.points.begin()+approachTrajectory.size()+slideTrajectory.size(), cmd.points.end());

				// Move to home
				robot_state::RobotState retractState(robotState);
				retractState.setJointGroupPositions(manipulator->arm, cmd.points.back().positions);
				retractState.updateCollisionBodyTransforms();
				if (!manipulator->interpolate(retractState, robotState, homeAction->cmd, TRANSIT_INTERPOLATE_STEPS, planningScene))
				{
					continue;
				}

				assert(motion->state->poses.size() == env->completedSegments.size());
				return motion;
			}
		}
	}

	return std::shared_ptr<Motion>();
}

std::shared_ptr<Motion>
MotionPlanner::sampleSlide(const robot_state::RobotState& robotState) const
{
	Pose robotTworld = env->worldTrobot.inverse(Eigen::Isometry);
	State objectState{State::Poses(env->completedSegments.size(), State::Pose::Identity())};

	// Get an object to slide
	int slideSegmentID = -1;
	Pose pushFrame;
	if (!objectSampler.sampleObject(slideSegmentID, pushFrame))
	{
		return std::shared_ptr<Motion>();
	}
	const octomap::OcTree* tree = env->completedSegments[slideSegmentID].get();

	// Get potential grasps
	auto graspPoses = getGraspPoses(tree);

	const int INTERPOLATE_STEPS = 25;
	const int TRANSIT_INTERPOLATE_STEPS = 50;
	const int SAMPLE_ATTEMPTS = 100;
	const double PALM_DISTANCE = 0.025;
	const double APPROACH_DISTANCE = 0.15;
	const double TABLE_BUFFER = 0.20;
	const double Z_SAFETY_HEIGHT = 0.16;

	trajectory_msgs::JointTrajectory cmd;
	std::uniform_real_distribution<double> xDistr(env->minExtent.x()+TABLE_BUFFER, env->maxExtent.x()-TABLE_BUFFER);
	std::uniform_real_distribution<double> yDistr(env->minExtent.y()+TABLE_BUFFER, env->maxExtent.y()-TABLE_BUFFER);
	std::uniform_real_distribution<double> thetaDistr(0.0, 2.0*M_PI);

	// Shuffle manipulators (without shuffling underlying array)
	std::vector<unsigned int> manip_indices(env->manipulators.size());
	std::iota(manip_indices.begin(), manip_indices.end(), 0);
	std::shuffle(manip_indices.begin(), manip_indices.end(), env->rng);

	while (!graspPoses.empty())
	{
		const auto& pair = graspPoses.top();
		Eigen::Affine3d gripperPose = pair.second;
		gripperPose.linear() = gripperPose.linear() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).matrix();
		gripperPose.translation() -= PALM_DISTANCE*gripperPose.linear().col(2);
		gripperPose.translation().z() = std::max(gripperPose.translation().z(), Z_SAFETY_HEIGHT);
		if (env->visualize)
		{
			tf::Transform t = tf::Transform::getIdentity();
			tf::poseEigenToTF(gripperPose, t);
			env->broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), env->worldFrame, "putative_start"));
		}

		for (unsigned int manip_idx : manip_indices)
		{
			auto& manipulator = env->manipulators[manip_idx];
			std::vector<std::vector<double>> sln = manipulator->IK(gripperPose, robotTworld, robotState);
			if (!sln.empty())
			{
				// Check whether the hand collides with the scene
				{
					robot_state::RobotState collisionState(robotState);
					collisionState.setJointGroupPositions(manipulator->arm, sln.front());
					collisionState.update();
					if (gripperEnvironmentCollision(manipulator, collisionState))
					{
						continue;
					}
				}

				Eigen::Affine3d goalPose;
				for (int attempt = 0; attempt < SAMPLE_ATTEMPTS; ++attempt)
				{
					goalPose = Eigen::Affine3d::Identity();
					goalPose.linear() = goalPose.linear() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).matrix();
					goalPose.linear() = goalPose.linear() * Eigen::AngleAxisd(thetaDistr(env->rng), Eigen::Vector3d::UnitZ()).matrix();
					goalPose.translation() = Eigen::Vector3d(xDistr(env->rng), yDistr(env->rng), gripperPose.translation().z());
					goalPose.translation() -= PALM_DISTANCE/10.0*goalPose.linear().col(2); // Go up very slightly during drag
					sln = manipulator->IK(goalPose, robotTworld, robotState);

					if (env->visualize)
					{
						tf::Transform temp; tf::poseEigenToTF(goalPose, temp);
						env->broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), env->worldFrame, "putative_goal"));
					}

					if (!sln.empty())
					{
						// Allocate the action sequence needed for a slide
						std::shared_ptr<CompositeAction> compositeAction = std::make_shared<CompositeAction>();
						auto pregraspAction = std::make_shared<GripperCommandAction>();  compositeAction->actions.push_back(pregraspAction);
						auto transitAction = std::make_shared<JointTrajectoryAction>();  compositeAction->actions.push_back(transitAction);
						auto approachAction = std::make_shared<JointTrajectoryAction>(); compositeAction->actions.push_back(approachAction);
						auto graspAction = std::make_shared<GripperCommandAction>();     compositeAction->actions.push_back(graspAction);
						auto slideAction = std::make_shared<JointTrajectoryAction>();    compositeAction->actions.push_back(slideAction);
						auto releaseAction = std::make_shared<GripperCommandAction>();   compositeAction->actions.push_back(releaseAction);
						auto retractAction = std::make_shared<JointTrajectoryAction>();  compositeAction->actions.push_back(retractAction);
						auto homeAction = std::make_shared<JointTrajectoryAction>();     compositeAction->actions.push_back(homeAction);
						compositeAction->primaryAction = 4;

						// Lay out the Cartesian Path
						Eigen::Affine3d gripperApproachPose = gripperPose;
						gripperApproachPose.translation() -= APPROACH_DISTANCE*gripperApproachPose.linear().col(2);

						Eigen::Affine3d gripperRetractPose = goalPose;
						gripperRetractPose.translation() -= APPROACH_DISTANCE*gripperRetractPose.linear().col(2);

						PoseSequence approachTrajectory, slideTrajectory, retractTrajectory, fullTrajectory;

						manipulator->interpolate(gripperApproachPose, gripperPose, approachTrajectory, INTERPOLATE_STEPS/2);
						manipulator->interpolate(gripperPose, goalPose, slideTrajectory, INTERPOLATE_STEPS);
						manipulator->interpolate(goalPose, gripperRetractPose, retractTrajectory, INTERPOLATE_STEPS/2);

						fullTrajectory.reserve(approachTrajectory.size()+slideTrajectory.size()+retractTrajectory.size());
						fullTrajectory.insert(fullTrajectory.end(), approachTrajectory.begin(), approachTrajectory.end());
						fullTrajectory.insert(fullTrajectory.end(), slideTrajectory.begin(), slideTrajectory.end());
						fullTrajectory.insert(fullTrajectory.end(), retractTrajectory.begin(), retractTrajectory.end());


						if (manipulator->cartesianPath(fullTrajectory, robotTworld, robotState, cmd))
						{
							std::shared_ptr<Motion> motion = std::make_shared<Motion>();
							motion->state = std::make_shared<State>(objectState);
							motion->action = compositeAction;

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
							pregraspAction->jointGroupName = manipulator->gripper->getName();

							// Move to start
							robot_state::RobotState approachState(robotState);
							approachState.setJointGroupPositions(manipulator->arm, cmd.points.front().positions);
							approachState.updateCollisionBodyTransforms();
							if (!manipulator->interpolate(robotState, approachState, transitAction->cmd, TRANSIT_INTERPOLATE_STEPS, planningScene))
							{
								continue;
							}

							// Approach
							approachAction->palm_trajectory = approachTrajectory;
							approachAction->cmd.joint_names = cmd.joint_names;
							approachAction->cmd.points.insert(approachAction->cmd.points.end(), cmd.points.begin(), cmd.points.begin()+approachTrajectory.size()+1);

							// Grasp
							graspAction->grasp.finger_a_command.position = 0.4;
							graspAction->grasp.finger_a_command.speed = 1.0;
							graspAction->grasp.finger_a_command.force = 1.0;
							graspAction->grasp.finger_b_command.position = 0.4;
							graspAction->grasp.finger_b_command.speed = 1.0;
							graspAction->grasp.finger_b_command.force = 1.0;
							graspAction->grasp.finger_c_command.position = 0.4;
							graspAction->grasp.finger_c_command.speed = 1.0;
							graspAction->grasp.finger_c_command.force = 1.0;
							graspAction->grasp.scissor_command.position = 0.2;
							graspAction->grasp.scissor_command.speed = 1.0;
							graspAction->grasp.scissor_command.force = 1.0;
							graspAction->jointGroupName = manipulator->gripper->getName();


							// Slide
							slideAction->palm_trajectory = slideTrajectory;
							slideAction->cmd.joint_names = cmd.joint_names;
							slideAction->cmd.points.insert(slideAction->cmd.points.end(), cmd.points.begin()+approachTrajectory.size(), cmd.points.begin()+approachTrajectory.size()+slideTrajectory.size()+1);
							motion->state->poses[slideSegmentID] = gripperPose.inverse(Eigen::Isometry) * goalPose;


							// Release
							releaseAction->grasp.finger_a_command.position = 0.0;
							releaseAction->grasp.finger_a_command.speed = 1.0;
							releaseAction->grasp.finger_a_command.force = 1.0;
							releaseAction->grasp.finger_b_command.position = 0.0;
							releaseAction->grasp.finger_b_command.speed = 1.0;
							releaseAction->grasp.finger_b_command.force = 1.0;
							releaseAction->grasp.finger_c_command.position = 0.0;
							releaseAction->grasp.finger_c_command.speed = 1.0;
							releaseAction->grasp.finger_c_command.force = 1.0;
							releaseAction->grasp.scissor_command.position = 0.2;
							releaseAction->grasp.scissor_command.speed = 1.0;
							releaseAction->grasp.scissor_command.force = 1.0;
							releaseAction->jointGroupName = manipulator->gripper->getName();

							// Retract
							retractAction->palm_trajectory = retractTrajectory;
							retractAction->cmd.joint_names = cmd.joint_names;
							retractAction->cmd.points.insert(retractAction->cmd.points.end(), cmd.points.begin()+approachTrajectory.size()+slideTrajectory.size(), cmd.points.end());

							// Move to home
							robot_state::RobotState retractState(robotState);
							retractState.setJointGroupPositions(manipulator->arm, cmd.points.back().positions);
							retractState.updateCollisionBodyTransforms();
							if (!manipulator->interpolate(retractState, robotState, homeAction->cmd, TRANSIT_INTERPOLATE_STEPS, planningScene))
							{
								continue;
							}

							assert(motion->state->poses.size() == env->completedSegments.size());
							return motion;
						}
					}
				}
			}
		}

		graspPoses.pop();
	}


	return std::shared_ptr<Motion>();
}


std::shared_ptr<Motion> MotionPlanner::pick(const robot_state::RobotState& robotState, const int targetID,
                                            std::set<int>& collisionObjects) const
{
	Pose robotTworld = env->worldTrobot.inverse(Eigen::Isometry);
	State objectState{State::Poses(env->completedSegments.size(), State::Pose::Identity())};

	// Get an object to slide
	const octomap::OcTree* tree = env->completedSegments[targetID].get();

	// Get potential grasps
	auto graspPoses = getGraspPoses(tree);

	const int INTERPOLATE_STEPS = 25;
	const int TRANSIT_INTERPOLATE_STEPS = 50;
	const double PALM_DISTANCE = 0.025;
	const double APPROACH_DISTANCE = 0.15;
	const double Z_SAFETY_HEIGHT = 0.16;

	trajectory_msgs::JointTrajectory cmd;

	// Shuffle manipulators (without shuffling underlying array)
	std::vector<unsigned int> manip_indices(env->manipulators.size());
	std::iota(manip_indices.begin(), manip_indices.end(), 0);
	std::shuffle(manip_indices.begin(), manip_indices.end(), env->rng);

	while (!graspPoses.empty())
	{
		const auto& pair = graspPoses.top();
		Eigen::Affine3d gripperPose = pair.second;
		gripperPose.linear() = gripperPose.linear() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).matrix();
		gripperPose.translation() -= PALM_DISTANCE*gripperPose.linear().col(2);
		gripperPose.translation().z() = std::max(gripperPose.translation().z(), Z_SAFETY_HEIGHT);
		if (env->visualize)
		{
			tf::Transform t = tf::Transform::getIdentity();
			tf::poseEigenToTF(gripperPose, t);
			env->broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), env->worldFrame, "putative_start"));
		}

		for (unsigned int manip_idx : manip_indices)
		{
			auto& manipulator = env->manipulators[manip_idx];
			std::vector<std::vector<double>> sln = manipulator->IK(gripperPose, robotTworld, robotState);
			if (!sln.empty())
			{
				// Check whether the hand collides with the scene
				{
					robot_state::RobotState collisionState(robotState);
					collisionState.setJointGroupPositions(manipulator->arm, sln.front());
					collisionState.update();
					if (gripperEnvironmentCollision(manipulator, collisionState))
					{
						std::cerr << "Grasp collided with environment." << std::endl;
						collision_detection::CollisionRequest collision_request;
						collision_detection::CollisionResult collision_result;
						collision_request.contacts = true;

						collision_detection::AllowedCollisionMatrix acm = gripperEnvironmentACM(manipulator);
						planningScene->checkCollision(collision_request, collision_result, collisionState, acm);

						octomap::point3d cameraOrigin((float) env->worldTcamera.translation().x(), (float) env->worldTcamera.translation().y(),
						                              (float) env->worldTcamera.translation().z());
						std::cerr << collision_result.contacts.size() << std::endl;
						for (auto& cts : collision_result.contacts)
						{
							std::cerr << cts.second.size() << std::endl;
							for (auto& c : cts.second)
							{
								Eigen::Vector3d p = env->worldTrobot * c.pos;
								std::cerr << p.transpose() << std::endl;

								octomap::point3d collision = octomap::point3d((float)p.x(), (float)p.y(), (float)p.z());
								collision = env->sceneOctree->keyToCoord(env->sceneOctree->coordToKey(collision));
								const auto& iter = env->coordToObject.find(collision);
								if (iter != env->coordToObject.end())
								{
									auto res = collisionObjects.insert(iter->second);
									if (res.second) { std::cerr << iter->second << std::endl; }
								}
								else
								{
									std::cerr << "raycasting" << std::endl;
									octomath::Vector3 ray = collision-cameraOrigin;
									bool hit = env->sceneOctree->castRay(cameraOrigin, ray, collision, true);
									if (hit)
									{
										collision = env->sceneOctree->keyToCoord(env->sceneOctree->coordToKey(collision));
										const auto& iter = env->coordToObject.find(collision);
										if (iter != env->coordToObject.end())
										{
											auto res = collisionObjects.insert(iter->second);
											if (res.second) { std::cerr << iter->second << std::endl; }
										}
									}
								}
							}
						}

						continue; // Continue checking possible poses and manipulators
					}
				}

				// Allocate the action sequence needed for a slide
				std::shared_ptr<CompositeAction> compositeAction = std::make_shared<CompositeAction>();
				auto pregraspAction = std::make_shared<GripperCommandAction>();  compositeAction->actions.push_back(pregraspAction);
				auto transitAction = std::make_shared<JointTrajectoryAction>();  compositeAction->actions.push_back(transitAction);
				auto approachAction = std::make_shared<JointTrajectoryAction>(); compositeAction->actions.push_back(approachAction);
				auto graspAction = std::make_shared<GripperCommandAction>();     compositeAction->actions.push_back(graspAction);
				auto retractAction = std::make_shared<JointTrajectoryAction>();  compositeAction->actions.push_back(retractAction);
				auto homeAction = std::make_shared<JointTrajectoryAction>();     compositeAction->actions.push_back(homeAction);
				compositeAction->primaryAction = 3;

				// Lay out the Cartesian Path
				Eigen::Affine3d gripperApproachPose = gripperPose;
				gripperApproachPose.translation() -= APPROACH_DISTANCE*gripperApproachPose.linear().col(2);

				Eigen::Affine3d gripperRetractPose = gripperPose;
				gripperRetractPose.translation() -= APPROACH_DISTANCE*gripperRetractPose.linear().col(2);

				PoseSequence approachTrajectory, retractTrajectory, fullTrajectory;

				manipulator->interpolate(gripperApproachPose, gripperPose, approachTrajectory, INTERPOLATE_STEPS/2);
				manipulator->interpolate(gripperPose, gripperRetractPose, retractTrajectory, INTERPOLATE_STEPS/2);

				fullTrajectory.reserve(approachTrajectory.size()+retractTrajectory.size());
				fullTrajectory.insert(fullTrajectory.end(), approachTrajectory.begin(), approachTrajectory.end());
				fullTrajectory.insert(fullTrajectory.end(), retractTrajectory.begin(), retractTrajectory.end());


				if (manipulator->cartesianPath(fullTrajectory, robotTworld, robotState, cmd))
				{
					std::shared_ptr<Motion> motion = std::make_shared<Motion>();
					motion->state = std::make_shared<State>(objectState);
					motion->action = compositeAction;

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
					pregraspAction->grasp.scissor_command.position = 0.5;
					pregraspAction->grasp.scissor_command.speed = 1.0;
					pregraspAction->grasp.scissor_command.force = 1.0;
					pregraspAction->jointGroupName = manipulator->gripper->getName();

					// Move to start
					robot_state::RobotState approachState(robotState);
					approachState.setJointGroupPositions(manipulator->arm, cmd.points.front().positions);
					approachState.updateCollisionBodyTransforms();
					if (!manipulator->interpolate(robotState, approachState, transitAction->cmd, TRANSIT_INTERPOLATE_STEPS, planningScene))
					{
						continue;
					}

					// Approach
					approachAction->palm_trajectory = approachTrajectory;
					approachAction->cmd.joint_names = cmd.joint_names;
					approachAction->cmd.points.insert(approachAction->cmd.points.end(), cmd.points.begin(), cmd.points.begin()+approachTrajectory.size()+1);

					// Grasp
					graspAction->grasp.finger_a_command.position = 0.4;
					graspAction->grasp.finger_a_command.speed = 1.0;
					graspAction->grasp.finger_a_command.force = 1.0;
					graspAction->grasp.finger_b_command.position = 0.4;
					graspAction->grasp.finger_b_command.speed = 1.0;
					graspAction->grasp.finger_b_command.force = 1.0;
					graspAction->grasp.finger_c_command.position = 0.4;
					graspAction->grasp.finger_c_command.speed = 1.0;
					graspAction->grasp.finger_c_command.force = 1.0;
					graspAction->grasp.scissor_command.position = 0.5;
					graspAction->grasp.scissor_command.speed = 1.0;
					graspAction->grasp.scissor_command.force = 1.0;
					graspAction->jointGroupName = manipulator->gripper->getName();

					// Retract
					retractAction->palm_trajectory = retractTrajectory;
					retractAction->cmd.joint_names = cmd.joint_names;
					retractAction->cmd.points.insert(retractAction->cmd.points.end(), cmd.points.begin()+approachTrajectory.size(), cmd.points.end());

					// Move to home
					robot_state::RobotState retractState(robotState);
					retractState.setJointGroupPositions(manipulator->arm, cmd.points.back().positions);
					retractState.updateCollisionBodyTransforms();
					if (!manipulator->interpolate(retractState, robotState, homeAction->cmd, TRANSIT_INTERPOLATE_STEPS, planningScene))
					{
						continue;
					}

					assert(motion->state->poses.size() == env->completedSegments.size());
					return motion;
				}
			}
			else
			{
				std::cerr << "No solution to pick pose found." << std::endl;
			}
		}

		graspPoses.pop();
	}


	return std::shared_ptr<Motion>();
}
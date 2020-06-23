/*
 * Copyright (c) 2020 Andrew Price
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "mps_voxels/planning/MotionPlanner.h"
#include "mps_voxels/octree_utils.h"
#include "mps_voxels/LocalOctreeServer.h"
#include "mps_voxels/util/assert.h"

#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/StdVector>

#include <tf_conversions/tf_eigen.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/circulator.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/min_quadrilateral_2.h>
#include <boost/heap/priority_queue.hpp>
#include <mps_voxels/planning/MotionPlanner.h>

#define _unused(x) ((void)(x))

namespace mps
{

const std::string MotionPlanner::CLUTTER_NAME = "clutter";

State stateFactory(const std::map<ObjectIndex, std::unique_ptr<Object>>& objects)
{
	State objectState;
	for (const auto& obj : objects)
	{
		objectState.poses.emplace_hint(objectState.poses.end(), std::make_pair(obj.first, State::Pose::Identity()));
	}
	return objectState;
}

bool arePointsInHandRegion(const octomap::OcTree* tree, const moveit::Pose& palmTworld)
{
	Eigen::Vector3d aabbMin(-0.05, -0.05, 0.0);
	Eigen::Vector3d aabbMax(0.05, 0.05, 0.1);

	for (const auto& p : getPoints(tree))
	{
		Eigen::Vector3d p_palm = palmTworld * Eigen::Vector3d(p.x(), p.y(), p.z());

		bool contained = true;
		for (int d = 0; d < 3; ++d)
		{
			contained = contained && (aabbMin[d] < p_palm[d]) && (p_palm[d] < aabbMax[d]);
		}

		if (contained)
		{
			return true;
		}
	}

	return false;
}

bool arePointsInHandRegion(const Manipulator* manip, const octomap::OcTree* tree, const robot_state::RobotState& state, const moveit::Pose& robotTworld)
{
	const moveit::Pose& robotTpalm = state.getFrameTransform(manip->palmName);
	const moveit::Pose palmTworld = robotTpalm.inverse(Eigen::Isometry) * robotTworld;

	return arePointsInHandRegion(tree, palmTworld);
}

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
	MPS_ASSERT(hull.is_convex());

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

			MPS_ASSERT(projN > -1e-6);
			if (projN > maxN)
			{
				maxN = projN;
				maxNpt = p;
			}
		}

//		const Eigen::Vector2d c = maxNpt - (maxN/2.0*nHat) + a; ///< Midpoint between active edge and distal point
		const Eigen::Vector2d c = (a+b)/2.0 + (maxN/2.0*nHat);// + vHat/2.0;
		moveit::Pose graspPose = moveit::Pose::Identity();
		graspPose.translation().head<2>() = c;
		graspPose.translation().z() = maxZ;
		graspPose.linear().topLeftCorner<2,2>() << nHat, vHat;

		graspPoses.push({v.norm(), graspPose});
	}

	return graspPoses;
}


ObjectSampler::ObjectSampler(const Scenario* scenario, const Scene* scene, const OccupancyData* env)
	: succeeded(false)
{
	// By default, search through all shadow points
	const octomap::point3d_collection* shadowPoints = &scene->occludedPts;
	if (!shadowPoints || shadowPoints->empty()) { throw std::runtime_error("No points shadowed in scene?!"); }

	if (!env->obstructions.empty())
	{
		std::cerr << "Target is obstructed!" << std::endl;
		for (const auto& i : env->obstructions)
		{
			std::cerr << i.first.id << "\t";
		}
		std::cerr << std::endl;
	}

	{
		std::uniform_int_distribution<> uni(0, env->objects.size()-1);
		auto objIter = env->objects.begin();
		std::advance(objIter, uni(scenario->rng()));
		id = objIter->first;
		succeeded = true;
		cameraOrigin = octomap::point3d((float) scene->worldTcamera.translation().x(),
		                                (float) scene->worldTcamera.translation().y(),
		                                (float) scene->worldTcamera.translation().z());
		samplePoint = cameraOrigin;
		ray = samplePoint-cameraOrigin;
		return;
	}

	std::uniform_real_distribution<> uni(0.0, std::nextafter(1.0, std::numeric_limits<double>::max()));

	// If the target object grasp is obstructed somehow
	if (!env->obstructions.empty() && uni(scenario->rng()) < 0.8)
	{
		// Select one obstructing object with uniform probability
//		std::uniform_int_distribution<size_t> distr(0, env->obstructions.size()-1)

		// Select with weighted probability
		std::vector<double> weights;
		for (const auto& i : env->obstructions) { weights.push_back(i.second); }
		std::discrete_distribution<size_t> distr(weights.begin(), weights.end());

		size_t idx = distr(scenario->rng());

		auto it(env->obstructions.begin());
		std::advance(it, idx);
		id = it->first;

		// Set that object's shadow to use for remainder of algorithm
		shadowPoints = &(env->objects.at(id)->shadow);
		std::cerr << shadowPoints->size() << " Shadow points in cell " << id.id << std::endl;
		if (shadowPoints->empty()) { throw std::runtime_error("No points shadowed by shape?!"); }
	}

	const int N = static_cast<int>(shadowPoints->size());
	std::vector<unsigned int> indices(N);
	std::iota(indices.begin(), indices.end(), 0);
	std::shuffle(indices.begin(), indices.end(), scenario->rng());

	cameraOrigin = octomap::point3d((float) scene->worldTcamera.translation().x(),
	                                (float) scene->worldTcamera.translation().y(),
	                                (float) scene->worldTcamera.translation().z());

	for (const unsigned int index : indices)
	{
		samplePoint = (*shadowPoints)[index];

		// Get a randomly selected shadowed point
		if (samplePoint.z()<0.05) { continue; }
		ray = samplePoint-cameraOrigin;
		bool hit = scene->sceneOctree->castRay(cameraOrigin, ray, collision, true);
		MPS_ASSERT(hit);
		collision = scene->sceneOctree->keyToCoord(scene->sceneOctree->coordToKey(collision)); // regularize

		if (scenario->shouldVisualize("object_sampling"))
		{
			// Display occluded point and push frame
			std::vector<tf::StampedTransform> tfs;
			const auto time = ros::Time::now();

			Eigen::Vector3d pt(samplePoint.x(), samplePoint.y(), samplePoint.z());
			tf::Transform t = tf::Transform::getIdentity();
			Eigen::Vector3d pt_camera = scene->worldTcamera.inverse(Eigen::Isometry)*pt;
			t.setOrigin({pt_camera.x(), pt_camera.y(), pt_camera.z()});
			tfs.emplace_back(tf::StampedTransform(t, time, scene->cameraFrame, "occluded_point"));

			t.setOrigin({collision.x(), collision.y(), collision.z()});
			tfs.emplace_back(tf::StampedTransform(t, time, scenario->worldFrame, "collision_point"));

			scenario->broadcaster->sendTransform(tfs);
		}

//		if (!env->obstructions.empty())
//		{
//			return;
//		}

		const auto& objIdx = env->coordToObject({collision.x(), collision.y(), collision.z()});
		if (objIdx.id == VoxelRegion::FREE_SPACE)
		{
			ROS_ERROR("Voxel was occluded by free space?");
			continue;
		}
		ObjectIndex pushSegmentID = objIdx;

		id = pushSegmentID;
		succeeded = true;
		return;
	}
}

double
MotionPlanner::reward(const robot_state::RobotState& robotState, const Motion* motion) const
{
	const size_t nClusters = motion->state->poses.size();
	MPS_ASSERT(nClusters == env->objects.size());

	Eigen::Matrix3Xd centroids(3, nClusters);
	int colIndex = 0;
	for (const auto& obj : env->objects)
	{
		MPS_ASSERT(obj.second);
		MPS_ASSERT(obj.second->occupancy->size() > 0);
		octomap::point3d_collection segmentPoints = obj.second->points;
		Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
		for (const auto& pt : segmentPoints)
		{
			centroid += Eigen::Vector3d(pt.x(), pt.y(), pt.z());
		}
		centroid /= static_cast<double>(segmentPoints.size());

		centroids.col(colIndex) = motion->state->poses[obj.first] * centroid;
		++colIndex;
	}

	Eigen::Vector3d centroid = centroids.rowwise().sum()/static_cast<double>(nClusters);
	Eigen::Matrix3Xd deltas = centroids.colwise() - centroid;
	double spread = deltas.colwise().squaredNorm().sum();

	double changeScore = 0;

	// Compute change in umbras
	for (const auto& obj : env->objects)
	{
		auto objIdx = obj.first;
		const Pose& worldTobj_prime = motion->state->poses[objIdx];
		if (worldTobj_prime.matrix().isIdentity(1e-6)) { continue; }

		const std::shared_ptr<octomap::OcTree>& segment = obj.second->occupancy;

//		const Pose worldTobj_init = Pose::Identity();
		const Pose worldTcamera_prime = /*worldTobj_init * */ worldTobj_prime.inverse(Eigen::Isometry) * scene->worldTcamera;

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
		const std::vector<octomap::point3d>& umbra = obj.second->shadow;
		for (size_t i = 0; i < umbra.size(); ++i)
		{
			// Check occlusion of points attached to world
			octomath::Vector3 ray = umbra[i]-cameraOrigin;
			octomap::point3d collision;
			bool occluded = segment->castRay(cameraOrigin, ray, collision, true, 2.0);
			if (!occluded)
			{
				// This world-attached point is now seen
				changeScore+=1;
			}

			// TODO: Verify
			Eigen::Vector3d pt_moved = worldTobj_prime * Eigen::Vector3d(umbra[i].x(), umbra[i].y(), umbra[i].z());
			ray = octomap::point3d((float)pt_moved.x(), (float)pt_moved.y(), (float)pt_moved.z())-cameraOrigin;
			occluded = segment->castRay(cameraOrigin, ray, collision, true, 2.0);
			if (!occluded)
			{
				// This body-attached point is now seen
				changeScore+=1;
			}

		}

		std::cerr << "Revealed " << changeScore << " voxels" << std::endl;
	}

	auto compositeAction = std::dynamic_pointer_cast<CompositeAction>(motion->action);

	// Compute number of collisions
	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
//	collision_request.contacts = true;

	int collisionCount = 0;
	for (size_t actionIdx = 0; actionIdx < compositeAction->actions.size(); ++actionIdx)
	{
		auto jointTraj = std::dynamic_pointer_cast<JointTrajectoryAction>(compositeAction->actions[actionIdx]);
		if (jointTraj)
		{
//			auto* arm = scene->manipulators.front()->pModel->getJointModelGroup(jointTraj->jointGroupName);
			const auto& manipulator = scenario->jointToManipulator.at(jointTraj->cmd.joint_names.front());
			collision_detection::AllowedCollisionMatrix acm = gripperEnvironmentACM(manipulator);

			if (motion->state->poses.size() !=  env->objects.size()) { throw std::runtime_error("Whoopsie."); }

			for (const auto& obj : env->objects)
			{
				// Since we're moving the objects, allow the gripper to touch moving objects
				if (!motion->state->poses[obj.first].matrix().isIdentity(1e-6))
				{
					acm.setEntry(std::to_string(obj.first.id), manipulator->gripper->getLinkModelNames(), true);
				}
			}

			for (size_t stepIdx = 0; stepIdx < jointTraj->cmd.points.size(); ++stepIdx)
			{
				jointTraj->cmd.points[stepIdx].positions;
				robot_state::RobotState collisionState(robotState);
				collisionState.setJointGroupPositions(manipulator->arm, jointTraj->cmd.points[stepIdx].positions);
				collisionState.setJointGroupPositions(manipulator->gripper, manipulator->getGripperOpenJoints());
				collisionState.update();
				planningScene->checkCollision(collision_request, collision_result, collisionState, acm);
				if (collision_result.collision)
				{
					// This step collided with the world
					++collisionCount;

//					for (const auto& p : collision_result.contacts)
//					{
//						std::cerr << p.first.first << " X " << p.first.second << std::endl;
//						for (const auto& c : p.second)
//						{
//							std::cerr << c.pos.transpose() << std::endl;
//						}
//					}
				}
			}
		}
	}

	// Compute direction of push/slide
	double dir = 0;
	if (compositeAction->primaryAction >= 0)
	{
		auto jointTraj = std::dynamic_pointer_cast<JointTrajectoryAction>(
			compositeAction->actions[compositeAction->primaryAction]);
		if (jointTraj)
		{
			if (jointTraj->palm_trajectory.size()>1)
			{
				const Pose& begin = jointTraj->palm_trajectory.front();
				const Pose& end = jointTraj->palm_trajectory.back();
				Eigen::Vector3d cb = (begin.translation()
				                      -centroid).normalized(); ///< Vector from centroid to beginning of move
				Eigen::Vector3d be = end.translation()-begin.translation(); ///< Vector of move
				dir = cb.dot(be);
			}
		}
	}

	return spread + 3.0*dir - 5.0*collisionCount + changeScore/2000.0;
}

collision_detection::WorldPtr
computeCollisionWorld(const Scene* scene, const OccupancyData& occupancy)
{
	auto world = std::make_shared<collision_detection::World>();

	moveit::Pose robotTworld = scene->worldTrobot.inverse(Eigen::Isometry);

	for (const auto& obstacle : scene->scenario->staticObstacles)
	{
		world->addToObject(MotionPlanner::CLUTTER_NAME, obstacle.first, robotTworld * obstacle.second);
	}

	// Use aliasing shared_ptr constructor
//	world->addToObject(CLUTTER_NAME,
//	                   std::make_shared<shapes::OcTree>(std::shared_ptr<octomap::OcTree>(std::shared_ptr<octomap::OcTree>{}, sceneOctree)),
//	                   robotTworld);

	if (occupancy.objects.empty())
	{
		ROS_WARN("No objects found in current occupancy state.");
	}

	for (const auto& obj : occupancy.objects)
	{
		const std::shared_ptr<octomap::OcTree>& segment = obj.second->occupancy;
		world->addToObject(std::to_string(obj.first.id), std::make_shared<shapes::OcTree>(segment), robotTworld);
	}

//	for (auto& approxSegment : approximateSegments)
//	{
//		world->addToObject(CLUTTER_NAME, approxSegment, robotTworld);
//	}

	return world;
}

planning_scene::PlanningSceneConstPtr
MotionPlanner::computePlanningScene(bool useCollisionObjects)
{
	collision_detection::WorldPtr world;
	if (useCollisionObjects)
	{
		assert(env);
		world = computeCollisionWorld(scene.get(), *env);
	}
	else
	{
		world = std::make_shared<collision_detection::World>();
	}
	collisionWorld = world; // NB: collisionWorld is const, but the planning scene constructor is not
	planningScene = std::make_shared<planning_scene::PlanningScene>(scene->scenario->manipulators.front()->pModel, world);
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
	acm.setEntry(CLUTTER_NAME, manipulator->gripper->getLinkModelNames(), false);
	for (size_t s = 0; s < env->objects.size(); ++s)
	{
		acm.setEntry(std::to_string(s), manipulator->gripper->getLinkModelNames(), false);
	}

	return acm;
}

bool
MotionPlanner::gripperEnvironmentCollision(const std::shared_ptr<Manipulator>& manipulator, const robot_state::RobotState& collisionState) const
{
	MPS_ASSERT(collisionWorld);

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
//			const auto& name = manipulator->gripper->getVariableNames()[i];
//			Eigen::Map<const Eigen::VectorXd> pos(collisionState.getJointPositions(name), manipulator->gripper->getJointModel(name)->getVariableCount());
//			std::cerr << name << ": " << pos.transpose() << std::endl;
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


bool MotionPlanner::addPhysicalObstructions(const std::shared_ptr<Manipulator>& manipulator,
                                            const robot_state::RobotState& collisionState,
                                            OccupancyData::ObstructionList& collisionObjects) const
{
	if (gripperEnvironmentCollision(manipulator, collisionState))
	{
		std::cerr << "Grasp collided with environment." << std::endl;
		collision_detection::CollisionRequest collision_request;
		collision_detection::CollisionResult collision_result;
		collision_request.contacts = true;

		collision_detection::AllowedCollisionMatrix acm = gripperEnvironmentACM(manipulator);
		planningScene->checkCollision(collision_request, collision_result, collisionState, acm);

		octomap::point3d cameraOrigin((float) scene->worldTcamera.translation().x(),
		                              (float) scene->worldTcamera.translation().y(),
		                              (float) scene->worldTcamera.translation().z());
		for (auto& cts : collision_result.contacts)
		{
			std::cerr << cts.second.size() << std::endl;

			for (const auto& obj : env->objects)
			{
				auto objIdx = obj.first;
				const auto& segmentOctree = obj.second->occupancy;

				// For each contact point, cast ray from camera to point; see if it hits completed shape
				for (auto& c : cts.second)
				{
					Eigen::Vector3d p = scene->worldTrobot * c.pos;
					std::cerr << p.transpose() << std::endl;

					octomap::point3d collision = octomap::point3d((float)p.x(), (float)p.y(), (float)p.z());
					//								collision = scene->sceneOctree->keyToCoord(scene->sceneOctree->coordToKey(collision));

					std::cerr << "raycasting" << std::endl;
					octomath::Vector3 ray = collision-cameraOrigin;

					bool hit = segmentOctree->castRay(cameraOrigin, ray, collision, true);
					if (hit)
					{
						if (env->targetObjectID && env->targetObjectID->id == objIdx.id)
						{
							std::cerr << "Attempt to grasp object resulted in collision with object. (Are the grippers open?)" << std::endl;
						}

						collisionObjects[objIdx] += 1.0;
//						collisionObjects.insert(objIdx);
						std::cerr << "hit " << objIdx.id << std::endl;
						break;
					}
				}
			}
		}
		return true;
	}
	return false;
}

bool MotionPlanner::addVisualObstructions(const ObjectIndex target, OccupancyData::ObstructionList& collisionObjects) const
{
	bool hasVisualObstruction = false;
	for (const auto& targetPt : env->objects.at(target)->points)
	{
		octomap::point3d cameraOrigin((float) scene->worldTcamera.translation().x(), (float) scene->worldTcamera.translation().y(),
		                              (float) scene->worldTcamera.translation().z());
		octomath::Vector3 ray = targetPt-cameraOrigin;
		octomap::point3d collision;
		bool hit = scene->sceneOctree->castRay(cameraOrigin, ray, collision);
		if (hit)
		{
			collision = scene->sceneOctree->keyToCoord(scene->sceneOctree->coordToKey(collision));
			const auto& objIdx = env->coordToObject({collision.x(), collision.y(), collision.z()});
			if (objIdx.id == VoxelRegion::FREE_SPACE)
			{
				if (objIdx.id!=target.id)
				{
					collisionObjects[objIdx] += 1.0;
					hasVisualObstruction = true;
				}
			}
		}
	}
	return hasVisualObstruction;
}

std::shared_ptr<Motion>
MotionPlanner::samplePush(const robot_state::RobotState& robotState, Introspection* info) const
{
	// TODO: Set hand posture before planning
	// Check whether the hand collides with the scene
	for (const auto& manipulator : scenario->manipulators)
	{
		if (gripperEnvironmentCollision(manipulator, robotState))
		{
			std::cerr << manipulator->gripper->getName() << " starts in collision. No solutions will be possible." << std::endl;
			return std::shared_ptr<Motion>();
		}
	}

	Pose robotTworld = scene->worldTrobot.inverse(Eigen::Isometry);
	State objectState = stateFactory(env->objects);

	// Get an object to slide
	const ObjectSampler sampleInfo(scenario.get(), scene.get(), env.get());
	if (info) { info->objectSampleInfo = sampleInfo; }
	if (!sampleInfo)
	{
		return std::shared_ptr<Motion>();
	}
	const octomap::OcTree* tree = env->objects.at(sampleInfo.id)->occupancy.get();

	Pose pushFrame;
	Eigen::Vector3d gHat = -Eigen::Vector3d::UnitZ();
	Eigen::Vector3d pHat = -Eigen::Map<const Eigen::Vector3f>(&sampleInfo.ray(0)).cast<double>().cross(gHat).normalized();
	Eigen::Vector3d nHat = gHat.cross(pHat).normalized();

	pushFrame.linear() << pHat.normalized(), nHat.normalized(), gHat.normalized();
	pushFrame.translation() = Eigen::Map<const Eigen::Vector3f>(&sampleInfo.collision(0)).cast<double>();

//	trajectory_msgs::JointTrajectory cmd;
	PoseSequence pushGripperFrames(2, moveit::Pose::Identity());
	const octomap::point3d_collection& segmentPoints = env->objects.at(sampleInfo.id)->points;

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
	if (scenario->shouldVisualize("push_sampling"))
	{
		tf::Transform t = tf::Transform::getIdentity();
		tf::poseEigenToTF(pushFrame, t);
		scenario->broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), scenario->worldFrame, "push_frame"));
		tf::poseEigenToTF(pushGripperFrames[0], t);
		scenario->broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), scenario->worldFrame, "push_gripper_frame_0"));
		tf::poseEigenToTF(pushGripperFrames[1], t);
		scenario->broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), scenario->worldFrame, "push_gripper_frame_1"));
	}

	pushGripperFrames.erase(std::remove_if(pushGripperFrames.begin(), pushGripperFrames.end(),
	                                 [&](const moveit::Pose& p){ return !arePointsInHandRegion(tree, p.inverse(Eigen::Isometry));}),
	                        pushGripperFrames.end());
	if (pushGripperFrames.empty())
	{
		return std::shared_ptr<Motion>();
	}

	// Shuffle manipulators (without shuffling underlying array)
	std::vector<unsigned int> manip_indices(scenario->manipulators.size());
	std::iota(manip_indices.begin(), manip_indices.end(), 0);
	std::shuffle(manip_indices.begin(), manip_indices.end(), scenario->rng());

	// Shuffle push directions (without shuffling underlying array)
	std::vector<unsigned int> push_indices(pushGripperFrames.size());
	std::iota(push_indices.begin(), push_indices.end(), 0);
	std::shuffle(push_indices.begin(), push_indices.end(), scenario->rng());

	std::uniform_int_distribution<int> stepDistr(15, 20);

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
		const int nSteps = stepDistr(scenario->rng());
		PoseSequence pushTrajectory;
		for (int s = -15; s < nSteps; ++s)
		{
			moveit::Pose step = pushGripperFrame;
			step.translation() += s*stepSize*pushGripperFrame.linear().col(2);
			pushTrajectory.push_back(step);
		}

		for (unsigned int manip_idx : manip_indices)
		{
			auto& manipulator = scenario->manipulators[manip_idx];

			std::vector<std::vector<double>> sln;

			sln = manipulator->IK(pushTrajectory.back(), robotTworld, robotState);
			if (sln.empty()) { continue; }

			sln = manipulator->IK(pushTrajectory.front(), robotTworld, robotState);
			if (sln.empty()) { continue; }

			// Check whether the hand collides with the scene
			{
				robot_state::RobotState collisionState(robotState);
				collisionState.setJointGroupPositions(manipulator->arm, sln.front());
				collisionState.setJointGroupPositions(manipulator->gripper, manipulator->getGripperOpenJoints());
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
			moveit::Pose gripperApproachPose = pushTrajectory.front();
			gripperApproachPose.translation() += APPROACH_HEIGHT*Eigen::Vector3d::UnitZ();

			moveit::Pose gripperRetractPose = pushTrajectory.back();
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
				motion->state->poses[sampleInfo.id] = pushTrajectory.front().inverse(Eigen::Isometry) * pushTrajectory.back();

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

				motion->targets.push_back(sampleInfo.id);

				MPS_ASSERT(motion->state->poses.size() == env->objects.size());
				return motion;
			}
		}
	}

	return std::shared_ptr<Motion>();
}

std::shared_ptr<Motion>
MotionPlanner::sampleSlide(const robot_state::RobotState& robotState, Introspection* info) const
{
	// TODO: Set hand posture before planning
	// Check whether the hand collides with the scene
	for (const auto& manipulator : scenario->manipulators)
	{
		if (gripperEnvironmentCollision(manipulator, robotState))
		{
			std::cerr << manipulator->gripper->getName() << " starts in collision. No solutions will be possible." << std::endl;
			return std::shared_ptr<Motion>();
		}
	}

	Pose robotTworld = scene->worldTrobot.inverse(Eigen::Isometry);
	State objectState = stateFactory(env->objects);

	// Get an object to slide
	const ObjectSampler sampleInfo(scenario.get(), scene.get(), env.get());
	if (info) { info->objectSampleInfo = sampleInfo; }
	if (!sampleInfo)
	{
		return std::shared_ptr<Motion>();
	}
	const octomap::OcTree* tree = env->objects.at(sampleInfo.id)->occupancy.get();

	// Get potential grasps
	auto graspPoses = getGraspPoses(tree);

	const int INTERPOLATE_STEPS = 25;
	const int TRANSIT_INTERPOLATE_STEPS = 50;
//	const int SAMPLE_ATTEMPTS = 100;
	const double PALM_DISTANCE = 0.025;
	const double APPROACH_DISTANCE = 0.15;
	const double TABLE_BUFFER = 0.20;
	const double Z_SAFETY_HEIGHT = 0.16;

	trajectory_msgs::JointTrajectory cmd;
	std::uniform_real_distribution<double> xDistr(scene->minExtent.x()+TABLE_BUFFER, scene->maxExtent.x()-TABLE_BUFFER);
	std::uniform_real_distribution<double> yDistr(scene->minExtent.y()+TABLE_BUFFER, scene->maxExtent.y()-TABLE_BUFFER);
	std::uniform_real_distribution<double> thetaDistr(0.0, 2.0*M_PI);

	// Shuffle manipulators (without shuffling underlying array)
	std::vector<unsigned int> manip_indices(scenario->manipulators.size());
	std::iota(manip_indices.begin(), manip_indices.end(), 0);
	std::shuffle(manip_indices.begin(), manip_indices.end(), scenario->rng());

	while (!graspPoses.empty())
	{
		moveit::Pose gripperPose = graspPoses.top().second;
		graspPoses.pop();
		gripperPose.linear() = gripperPose.linear() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).matrix();
		gripperPose.translation() -= PALM_DISTANCE*gripperPose.linear().col(2);
		gripperPose.translation().z() = std::max(gripperPose.translation().z(), Z_SAFETY_HEIGHT);

		if (!arePointsInHandRegion(tree, gripperPose.inverse(Eigen::Isometry)))
		{
			continue;
		}

		if (scenario->shouldVisualize("slide_sampling"))
		{
			tf::Transform t = tf::Transform::getIdentity();
			tf::poseEigenToTF(gripperPose, t);
			scenario->broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), scenario->worldFrame, "putative_start"));
		}

		for (unsigned int manip_idx : manip_indices)
		{
			const auto& manipulator = scenario->manipulators[manip_idx];
			std::vector<std::vector<double>> sln = manipulator->IK(gripperPose, robotTworld, robotState);
			if (!sln.empty())
			{
				// Check whether the hand collides with the scene
				{
					robot_state::RobotState collisionState(robotState);
					collisionState.setJointGroupPositions(manipulator->arm, sln.front());
					collisionState.setJointGroupPositions(manipulator->gripper, manipulator->getGripperOpenJoints());
					collisionState.update();
					if (gripperEnvironmentCollision(manipulator, collisionState))
					{
						continue;
					}
				}

				moveit::Pose goalPose;
//				for (int attempt = 0; attempt < SAMPLE_ATTEMPTS; ++attempt)
				{
					goalPose = moveit::Pose::Identity();
					goalPose.linear() = goalPose.linear() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).matrix();
					goalPose.linear() = goalPose.linear() * Eigen::AngleAxisd(thetaDistr(scenario->rng()), Eigen::Vector3d::UnitZ()).matrix();
					goalPose.translation() = Eigen::Vector3d(xDistr(scenario->rng()), yDistr(scenario->rng()), gripperPose.translation().z());
					goalPose.translation() -= PALM_DISTANCE/10.0*goalPose.linear().col(2); // Go up very slightly during drag
					sln = manipulator->IK(goalPose, robotTworld, robotState);

					if (scenario->shouldVisualize("slide_sampling"))
					{
						tf::Transform temp; tf::poseEigenToTF(goalPose, temp);
						scenario->broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), scenario->worldFrame, "putative_goal"));
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
						moveit::Pose gripperApproachPose = gripperPose;
						gripperApproachPose.translation() -= APPROACH_DISTANCE*gripperApproachPose.linear().col(2);

						moveit::Pose gripperRetractPose = goalPose;
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
							motion->state->poses[sampleInfo.id] = gripperPose.inverse(Eigen::Isometry) * goalPose;


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

							motion->targets.push_back(sampleInfo.id);

							MPS_ASSERT(motion->state->poses.size() == env->objects.size());
							return motion;
						}
					}
				}
			}
		}
	}


	return std::shared_ptr<Motion>();
}


std::shared_ptr<Motion> MotionPlanner::pick(const robot_state::RobotState& robotState, const ObjectIndex targetID,
                                            OccupancyData::ObstructionList& collisionObjects) const
{
	MPS_ASSERT(targetID.id == env->targetObjectID->id);
	// Check whether the hand collides with the scene
	for (const auto& manipulator : scenario->manipulators)
	{
		if (gripperEnvironmentCollision(manipulator, robotState))
		{
			std::cerr << manipulator->gripper->getName() << " starts in collision. No solutions will be possible." << std::endl;
			return std::shared_ptr<Motion>();
		}
	}

	Pose robotTworld = scene->worldTrobot.inverse(Eigen::Isometry);
	State objectState = stateFactory(env->objects);

	// Get an object to slide
	const octomap::OcTree* tree = env->objects.at(targetID)->occupancy.get();

	// Get potential grasps
	auto graspPoses = getGraspPoses(tree);

	const int INTERPOLATE_STEPS = 25;
	const int TRANSIT_INTERPOLATE_STEPS = 50;
	const double PALM_DISTANCE = 0.025;
	const double APPROACH_DISTANCE = 0.15;
	const double Z_SAFETY_HEIGHT = 0.16;

	// Shuffle manipulators (without shuffling underlying array)
	std::vector<unsigned int> manip_indices(scenario->manipulators.size());
	std::iota(manip_indices.begin(), manip_indices.end(), 0);
	std::shuffle(manip_indices.begin(), manip_indices.end(), scenario->rng());

	while (!graspPoses.empty())
	{
		moveit::Pose gripperPose = graspPoses.top().second;
		graspPoses.pop();
		gripperPose.linear() = gripperPose.linear() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).matrix();
		gripperPose.translation() -= PALM_DISTANCE*gripperPose.linear().col(2);
		gripperPose.translation().z() = std::max(gripperPose.translation().z(), Z_SAFETY_HEIGHT);

		if (!arePointsInHandRegion(tree, gripperPose.inverse(Eigen::Isometry)))
		{
			continue;
		}

		if (scenario->shouldVisualize("pick_sampling"))
		{
			tf::Transform t = tf::Transform::getIdentity();
			tf::poseEigenToTF(gripperPose, t);
			scenario->broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), scenario->worldFrame, "putative_start"));
		}

		for (unsigned int manip_idx : manip_indices)
		{
			const auto& manipulator = scenario->manipulators[manip_idx];
			std::vector<std::vector<double>> sln = manipulator->IK(gripperPose, robotTworld, robotState);
			if (sln.empty())
			{
				std::cerr << "No solution to pick pose found." << std::endl;
				continue;
			}

			// Check whether the hand collides with the scene
			{
				robot_state::RobotState collisionState(robotState);
				collisionState.setJointGroupPositions(manipulator->arm, sln.front());
				collisionState.setJointGroupPositions(manipulator->gripper, manipulator->getGripperOpenJoints());
				collisionState.update();

				if (addPhysicalObstructions(manipulator, collisionState, collisionObjects))
				{
					continue;
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
			moveit::Pose gripperApproachPose = gripperPose;
			gripperApproachPose.translation() -= APPROACH_DISTANCE*gripperApproachPose.linear().col(2);

			moveit::Pose gripperRetractPose = gripperPose;
			gripperRetractPose.translation() -= APPROACH_DISTANCE*gripperRetractPose.linear().col(2);

			PoseSequence approachTrajectory, retractTrajectory, fullTrajectory;

			manipulator->interpolate(gripperApproachPose, gripperPose, approachTrajectory, INTERPOLATE_STEPS/2);
			manipulator->interpolate(gripperPose, gripperRetractPose, retractTrajectory, INTERPOLATE_STEPS/2);

			fullTrajectory.reserve(approachTrajectory.size()+retractTrajectory.size());
			fullTrajectory.insert(fullTrajectory.end(), approachTrajectory.begin(), approachTrajectory.end());
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

				motion->targets.push_back(targetID);

				MPS_ASSERT(motion->state->poses.size() == env->objects.size());
				return motion;
			}
		}
	}


	return std::shared_ptr<Motion>();
}


std::shared_ptr<Motion>
MotionPlanner::recoverCrash(const robot_state::RobotState& currentState,
                            const robot_state::RobotState& recoveryState) const
{
	// Check whether the hand collides with the scene
	for (const auto& manipulator : scenario->manipulators)
	{
		if (gripperEnvironmentCollision(manipulator, currentState))
		{
			std::cerr << manipulator->gripper->getName() << " starts in collision. No solutions will be possible." << std::endl;
			return std::shared_ptr<Motion>();
		}
	}

	const double RETREAT_DISTANCE = 0.05;
	const int INTERPOLATE_STEPS = 10;
	const int TRANSIT_INTERPOLATE_STEPS = 50;

	for (const auto& manipulator : scenario->manipulators)
	{
		// See if needs recovery
		Eigen::VectorXd qCurrent, qRecovery;
		currentState.copyJointGroupPositions(manipulator->arm, qCurrent);
		recoveryState.copyJointGroupPositions(manipulator->arm, qRecovery);
		Eigen::VectorXd qErr = qCurrent - qRecovery;
		bool satisfied = true; for (int i = 0; i < qErr.size(); ++i) { if (fabs(qErr[i]) > 1e-1) { satisfied = false; } }
		if (satisfied)
		{
			std::cerr << manipulator->arm->getName() << " satisfies recovery." << std::endl;
			continue;
		}

		Pose worldThand = scene->worldTrobot*currentState.getFrameTransform(manipulator->palmName);
		Pose worldTretreat = worldThand;
		worldTretreat.translation().z() += RETREAT_DISTANCE;
		Pose robotTworld = scene->worldTrobot.inverse(Eigen::Isometry);

		std::shared_ptr<CompositeAction> compositeAction = std::make_shared<CompositeAction>();
		auto releaseAction = std::make_shared<GripperCommandAction>();
		compositeAction->actions.push_back(releaseAction);
		auto retractAction = std::make_shared<JointTrajectoryAction>();
		compositeAction->actions.push_back(retractAction);
		auto homeAction = std::make_shared<JointTrajectoryAction>();
		compositeAction->actions.push_back(homeAction);
		compositeAction->primaryAction = -1;

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
		releaseAction->grasp.scissor_command.position = 0.5;
		releaseAction->grasp.scissor_command.speed = 1.0;
		releaseAction->grasp.scissor_command.force = 1.0;
		releaseAction->jointGroupName = manipulator->gripper->getName();

		// Compute retraction
		PoseSequence retractTrajectory;
		if (!manipulator->interpolate(worldThand, worldTretreat, retractTrajectory, INTERPOLATE_STEPS))
		{
			std::cerr << "Failed to create Cartesian trajectory." << std::endl;
			continue;
		}

		trajectory_msgs::JointTrajectory cmd;
		if (manipulator->cartesianPath(retractTrajectory, robotTworld, currentState, cmd))
		{
			std::cerr << "Failed to follow Cartesian trajectory." << std::endl;
			continue;
		}

		// Retract
		retractAction->palm_trajectory = retractTrajectory;
		retractAction->cmd.joint_names = cmd.joint_names;
		retractAction->cmd.points.insert(retractAction->cmd.points.end(), cmd.points.begin(), cmd.points.end());

		// Move to home
		robot_state::RobotState retractState(currentState);
		retractState.setJointGroupPositions(manipulator->arm, cmd.points.back().positions);
		retractState.updateCollisionBodyTransforms();
		if (!manipulator->interpolate(retractState, recoveryState, homeAction->cmd, TRANSIT_INTERPOLATE_STEPS, planningScene))
		{
			std::cerr << "Failed to return home." << std::endl;
			continue;
		}

		State objectState = stateFactory(env->objects);
		std::shared_ptr<Motion> motion = std::make_shared<Motion>();
		motion->state = std::make_shared<State>(objectState);
		motion->action = compositeAction;

		std::cerr << "Got recovery motion." << std::endl;
		return motion;
	}
	return std::shared_ptr<Motion>();
}

MotionPlanner::MotionPlanner(std::shared_ptr<const Scenario> _scenario, std::shared_ptr<const Scene> _scene, std::shared_ptr<const OccupancyData> _occupancy)
	: scenario(std::move(_scenario)), scene(std::move(_scene)), env(std::move(_occupancy))
{
	computePlanningScene(true);
}

}

//
// Created by arprice on 9/4/18.
//

#include "mps_voxels/planning/MotionPlanner.h"
#include "mps_voxels/octree_utils.h"

#include <Eigen/StdVector>

#include <tf_conversions/tf_eigen.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/circulator.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/min_quadrilateral_2.h>
#include <boost/heap/priority_queue.hpp>

bool ObjectSampler::sampleObject(int& id, Pose& pushFrame) const
{
	const int N = static_cast<int>(env->occluded_pts.size());
	std::vector<unsigned int> indices(N);
	std::iota(indices.begin(), indices.end(), 0);
	std::shuffle(indices.begin(), indices.end(), env->rng);

	for (const unsigned int index : indices)
	{
		const auto& pt_original = env->occluded_pts[index];

		// Get a randomly selected shadowed point
		Eigen::Vector3d pt(pt_original.x(), pt_original.y(), pt_original.z());
		if (pt.z()<0.05) { continue; }

		octomap::point3d cameraOrigin((float) env->worldTcamera.translation().x(), (float) env->worldTcamera.translation().y(),
		                              (float) env->worldTcamera.translation().z());
		octomath::Vector3 ray = pt_original-cameraOrigin;
		octomap::point3d collision;
		bool hit = env->sceneOctree->castRay(cameraOrigin, ray, collision, true);
		assert(hit);
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

		const auto& iter = env->coordToObject.find(collision);
		if (iter==env->coordToObject.end()) { continue; }
		int pushSegmentID = iter->second;

		id = pushSegmentID;
		return true;
	}

	return false;
}

double
MotionPlanner::reward(const Motion* motion) const
{
	const int nClusters = static_cast<int>(motion->state->poses.size());
	assert(nClusters == static_cast<int>(env->completedSegments.size()));

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

	return spread;
}

trajectory_msgs::JointTrajectory
MotionPlanner::planPush(const octomap::OcTree* tree,
                        const robot_state::RobotState& currentState,
                        const Eigen::Affine3d& pushFrame)
{
	Pose robotTworld = env->worldTrobot.inverse(Eigen::Isometry);

	trajectory_msgs::JointTrajectory cmd;
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

	const double GRIPPER_HALFWIDTH = 5.0*2.54/100.0;
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

	std::sort(env->manipulators.begin(), env->manipulators.end(), [&](const std::shared_ptr<Manipulator>& A, const std::shared_ptr<Manipulator>& B)->bool
	{
		const robot_model::JointModelGroup* a = A->arm;
		const robot_model::JointModelGroup* b = B->arm;
		return (pushFrame.translation()-currentState.getFrameTransform(a->getOnlyOneEndEffectorTip()->getName()).translation()).squaredNorm()
		       < (pushFrame.translation()-currentState.getFrameTransform(b->getOnlyOneEndEffectorTip()->getName()).translation()).squaredNorm();
	});

	bool foundSolution = false;
	for (const auto& pushGripperFrame : pushGripperFrames)
	{
		const double stepSize = 0.015;
		PoseSequence pushTrajectory;
		for (int s = -15; s < 10; ++s)
		{
			Eigen::Affine3d step = pushGripperFrame;
			step.translation() += s*stepSize*pushGripperFrame.linear().col(2);
			pushTrajectory.push_back(step);
		}

		for (auto& manipulator : env->manipulators)
		{
			if (manipulator->cartesianPath(pushTrajectory, robotTworld, currentState, cmd))
			{
				foundSolution = true;
				break;
			}
		}

		if (foundSolution)
		{
			break;
		}
	}

	return cmd;
}

std::shared_ptr<Motion>
MotionPlanner::sampleSlide(const robot_state::RobotState& robotState)
{
	using K =         CGAL::Exact_predicates_inexact_constructions_kernel;
	using Point_2 =   K::Point_2;
	using Line_2 =    K::Line_2;
	using Polygon_2 = CGAL::Polygon_2<K>;
	using Segment_2 = Polygon_2::Segment_2;

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

	using RankedPose = std::pair<double, Eigen::Affine3d>;
	auto comp = [](const RankedPose& a, const RankedPose& b ) { return a.first < b.first; };
	std::priority_queue<RankedPose, std::vector<RankedPose>, decltype(comp)> graspPoses(comp);

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
		size_t maxNidx = i;
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
				maxNidx = j;
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

		if (env->visualize)
		{
			tf::Transform t = tf::Transform::getIdentity();
			tf::poseEigenToTF(graspPose, t);
			env->broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), env->worldFrame, "push_slide_frame_" + std::to_string(i)));
		}
	}

	const int INTERPOLATE_STEPS = 15;
	const int SAMPLE_ATTEMPTS = 100;

	trajectory_msgs::JointTrajectory cmd;
	bool foundSolution = false;
	std::uniform_real_distribution<double> xDistr(env->minExtent.x(), env->maxExtent.x());
	std::uniform_real_distribution<double> yDistr(env->minExtent.y(), env->maxExtent.y());
	std::uniform_real_distribution<double> thetaDistr(0.0, 2.0*M_PI);

	while (!graspPoses.empty())
	{
		const auto& pair = graspPoses.top();
		Eigen::Affine3d gripperPose = pair.second;
		gripperPose.linear() = gripperPose.linear() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).matrix();
		if (env->visualize)
		{
			tf::Transform t = tf::Transform::getIdentity();
			tf::poseEigenToTF(gripperPose, t);
			env->broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), env->worldFrame, "putative_start"));
		}

		for (auto& manipulator : env->manipulators)
		{
			std::vector<std::vector<double>> sln = manipulator->IK(gripperPose, robotTworld, robotState);
			if (!sln.empty())
			{
				Eigen::Affine3d goalPose;
				for (int attempt = 0; attempt < SAMPLE_ATTEMPTS; ++attempt)
				{
					goalPose = Eigen::Affine3d::Identity();
					goalPose.linear() = goalPose.linear() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).matrix();
					goalPose.linear() = goalPose.linear() * Eigen::AngleAxisd(thetaDistr(env->rng), Eigen::Vector3d::UnitZ()).matrix();
					goalPose.translation() = Eigen::Vector3d(xDistr(env->rng), yDistr(env->rng), gripperPose.translation().z());
					sln = manipulator->IK(goalPose, robotTworld, robotState);

					if (env->visualize)
					{
						tf::Transform temp; tf::poseEigenToTF(goalPose, temp);
						env->broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), env->worldFrame, "putative_goal"));
					}

					if (!sln.empty())
					{
						PoseSequence slideTrajectory; slideTrajectory.reserve(INTERPOLATE_STEPS);
						Eigen::Quaterniond qStart(gripperPose.linear()), qEnd(goalPose.linear());
						for (int i = 0; i<INTERPOLATE_STEPS; ++i)
						{
							double t = i/static_cast<double>(INTERPOLATE_STEPS-1);
							Eigen::Affine3d interpPose = Eigen::Affine3d::Identity();
							interpPose.translation() = ((1.0-t)*gripperPose.translation())+(t*goalPose.translation());
							interpPose.linear() = qStart.slerp(t, qEnd).matrix();
							slideTrajectory.push_back(interpPose);

							if (env->visualize)
							{
								tf::Transform temp; tf::poseEigenToTF(interpPose, temp);
								env->broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), env->worldFrame, "slide_"+std::to_string(i)));
							}
						}

						if (manipulator->cartesianPath(slideTrajectory, robotTworld, robotState, cmd))
						{
							std::shared_ptr<Motion> motion = std::make_shared<Motion>();
							motion->state = std::make_shared<State>(objectState);
							motion->action = std::make_shared<JointTrajectoryAction>();

							std::dynamic_pointer_cast<JointTrajectoryAction>(motion->action)->cmd = cmd;
							std::dynamic_pointer_cast<JointTrajectoryAction>(motion->action)->palm_trajectory = slideTrajectory;
							motion->state->poses[slideSegmentID] = gripperPose.inverse(Eigen::Isometry) * goalPose;

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
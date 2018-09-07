//
// Created by arprice on 9/4/18.
//

#include "mps_voxels/MotionPlanner.h"
#include "mps_voxels/octree_utils.h"

#include <tf_conversions/tf_eigen.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/circulator.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/min_quadrilateral_2.h>
#include <boost/heap/priority_queue.hpp>

trajectory_msgs::JointTrajectory
MotionPlanner::planPush(const octomap::OcTree* tree,
                        const robot_state::RobotState& currentState,
                        const Eigen::Affine3d& pushFrame)
{
	trajectory_msgs::JointTrajectory cmd;
	std::vector<Eigen::Affine3d> pushGripperFrames(2, Eigen::Affine3d::Identity());
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
	if (visualize)
	{
		tf::Transform t = tf::Transform::getIdentity();
		tf::poseEigenToTF(pushGripperFrames[0], t);
		broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), globalFrame, "push_gripper_frame_0"));
		tf::poseEigenToTF(pushGripperFrames[1], t);
		broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), globalFrame, "push_gripper_frame_1"));
	}

	std::sort(manipulators.begin(), manipulators.end(), [&](const std::shared_ptr<Manipulator>& A, const std::shared_ptr<Manipulator>& B)->bool
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
		std::vector<Eigen::Affine3d> pushTrajectory;
		for (int s = -15; s < 10; ++s)
		{
			Eigen::Affine3d step = pushGripperFrame;
			step.translation() += s*stepSize*pushGripperFrame.linear().col(2);
			pushTrajectory.push_back(step);
		}

		for (auto& manipulator : manipulators)
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

trajectory_msgs::JointTrajectory
MotionPlanner::planDrag(const octomap::OcTree* tree,
                        const robot_state::RobotState& currentState,
                        const Eigen::Affine3d& pushFrame)
{
	using K =         CGAL::Exact_predicates_inexact_constructions_kernel;
	using Point_2 =   K::Point_2;
	using Line_2 =    K::Line_2;
	using Polygon_2 = CGAL::Polygon_2<K>;
	using Segment_2 = Polygon_2::Segment_2;

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
	CGAL::ch_bykat(cgal_points.begin(), cgal_points.end(), std::back_inserter(hull)); // O(nh) vs O(nlogn) convex_hull_2
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

		if (visualize)
		{
			tf::Transform t = tf::Transform::getIdentity();
			tf::poseEigenToTF(graspPose, t);
			broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), globalFrame, "push_slide_frame_" + std::to_string(i)));
		}
	}

	const int INTERPOLATE_STEPS = 15;
	const int SAMPLE_ATTEMPTS = 100;

	trajectory_msgs::JointTrajectory cmd;
	bool foundSolution = false;
	std::uniform_real_distribution<double> xDistr(minExtent.x(), maxExtent.x());
	std::uniform_real_distribution<double> yDistr(minExtent.y(), maxExtent.y());
	std::uniform_real_distribution<double> thetaDistr(0.0, 2.0*M_PI);

	while (!graspPoses.empty())
	{
		const auto& pair = graspPoses.top();
		Eigen::Affine3d gripperPose = pair.second;
		gripperPose.linear() = gripperPose.linear() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).matrix();
		if (visualize)
		{
			tf::Transform t = tf::Transform::getIdentity();
			tf::poseEigenToTF(gripperPose, t);
			broadcaster->sendTransform(tf::StampedTransform(t, ros::Time::now(), globalFrame, "putative_start"));
		}

		for (auto& manipulator : manipulators)
		{
			std::vector<std::vector<double>> sln = manipulator->IK(gripperPose, robotTworld, currentState);
			if (!sln.empty())
			{
				Eigen::Affine3d goalPose;
				for (int attempt = 0; attempt < SAMPLE_ATTEMPTS; ++attempt)
				{
					goalPose = Eigen::Affine3d::Identity();
					goalPose.linear() = goalPose.linear() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).matrix();
					goalPose.linear() = goalPose.linear() * Eigen::AngleAxisd(thetaDistr(rng), Eigen::Vector3d::UnitZ()).matrix();
					goalPose.translation() = Eigen::Vector3d(xDistr(rng), yDistr(rng), gripperPose.translation().z());
					sln = manipulator->IK(goalPose, robotTworld, currentState);

					if (visualize)
					{
						tf::Transform temp; tf::poseEigenToTF(goalPose, temp);
						broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), globalFrame, "putative_goal"));
					}

					if (!sln.empty())
					{
						std::vector<Eigen::Affine3d> slideTrajectory; slideTrajectory.reserve(INTERPOLATE_STEPS);
						Eigen::Quaterniond qStart(gripperPose.linear()), qEnd(goalPose.linear());
						for (int i = 0; i<INTERPOLATE_STEPS; ++i)
						{
							double t = i/static_cast<double>(INTERPOLATE_STEPS-1);
							Eigen::Affine3d interpPose = Eigen::Affine3d::Identity();
							interpPose.translation() = ((1.0-t)*gripperPose.translation())+(t*goalPose.translation());
							interpPose.linear() = qStart.slerp(t, qEnd).matrix();
							slideTrajectory.push_back(interpPose);

							if (visualize)
							{
								tf::Transform temp; tf::poseEigenToTF(interpPose, temp);
								broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), globalFrame, "slide_"+std::to_string(i)));
							}
						}

						if (manipulator->cartesianPath(slideTrajectory, robotTworld, currentState, cmd))
						{
							foundSolution = true;
							break;
						}
					}
				}
				if (foundSolution)
				{
					break;
				}
			}
			if (foundSolution)
			{
				break;
			}
		}

		if (foundSolution)
		{
			break;
		}

		graspPoses.pop();
	}

	std::cerr << nPts << std::endl;

	return cmd;
}
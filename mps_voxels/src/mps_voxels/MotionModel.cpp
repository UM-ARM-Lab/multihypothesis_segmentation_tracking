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

#include "mps_voxels/MotionModel.h"

#include <geometric_shapes/body_operations.h>

#include <CGAL/Cartesian_d.h>
#include <CGAL/Min_sphere_of_spheres_d.h>

namespace mps
{

static const double PADDING = 0.02;
static const double NEARLY_ZERO = 1e-9;

static inline double distanceSQR(const Eigen::Vector3d& p, const Eigen::Vector3d& origin, const Eigen::Vector3d& dir)
{
	Eigen::Vector3d a = p - origin;
	double d = dir.normalized().dot(a);
	return a.squaredNorm() - d * d;
}

bool sphereIntersectsRay(const bodies::BoundingSphere& sphere,
                         const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
                         EigenSTL::vector_Vector3d* intersections = nullptr, unsigned int count = 0)
{
	double radius2 = sphere.radius * sphere.radius;
	if (distanceSQR(sphere.center, origin, dir) > radius2)
	{
		return false;
	}

	bool result = false;

	Eigen::Vector3d cp = origin - sphere.center;
	double dpcpv = cp.dot(dir);

	Eigen::Vector3d w = cp - dpcpv * dir;
	Eigen::Vector3d Q = sphere.center + w;
	double x = radius2 - w.squaredNorm();

	if (fabs(x) < NEARLY_ZERO)
	{
		w = Q - origin;
		double dpQv = w.dot(dir);
		if (dpQv > NEARLY_ZERO)
		{
			if (intersections)
				intersections->push_back(Q);
			result = true;
		}
	}
	else if (x > 0.0)
	{
		x = sqrt(x);
		w = dir * x;
		Eigen::Vector3d A = Q - w;
		Eigen::Vector3d B = Q + w;
		w = A - origin;
		double dpAv = w.dot(dir);
		w = B - origin;
		double dpBv = w.dot(dir);

		if (dpAv > NEARLY_ZERO)
		{
			result = true;
			if (intersections)
			{
				intersections->push_back(A);
				if (count == 1)
					return result;
			}
		}

		if (dpBv > NEARLY_ZERO)
		{
			result = true;
			if (intersections)
				intersections->push_back(B);
		}
	}
	return result;
}

void MotionModel::updateMembershipStructures()
{
	typedef double FT;
	typedef CGAL::Cartesian_d<FT> K;
	typedef CGAL::Min_sphere_of_spheres_d_traits_d<K,FT,3> Traits;
	typedef CGAL::Min_sphere_of_spheres_d<Traits> Min_sphere;
	typedef K::Point_d Point;
	typedef Traits::Sphere Sphere;

	std::vector<bodies::BoundingSphere> bSpheres(membershipBodies.size());
	for (size_t i = 0; i < membershipBodies.size(); ++i)
	{
		// For meshes, we want to use CGAL's more sophisticated sphere-fitting algorithm
		const auto& mesh = std::dynamic_pointer_cast<const shapes::Mesh>(membershipShapes[i]);
		if (mesh)
		{
			std::vector<Sphere> S;
			for (int pIdx = 0; pIdx < static_cast<int>(mesh->vertex_count); ++pIdx)
			{
				Point pt(3, mesh->vertices+(pIdx*3), mesh->vertices+(pIdx*3) + 3);
				S.emplace_back(Sphere(pt, membershipBodies[i]->getPadding()));
			}
			Min_sphere min_sphere(S.begin(), S.end());
			if (min_sphere.is_valid())
			{
				bSpheres[i].center = Eigen::Map<const Eigen::Vector3d>(min_sphere.center_cartesian_begin());
				bSpheres[i].radius = min_sphere.radius();
				continue;
			}
		}

		membershipBodies[i]->computeBoundingSphere(bSpheres[i]);
	}
	bodies::mergeBoundingSpheres(bSpheres, boundingSphere);
}

const int RigidMotionModel::MOTION_PARAMETERS_DIMENSION;

double RigidMotionModel::membershipLikelihood(const Eigen::Vector3d& pt) const
{
	const Eigen::Vector3d pt_local = localTglobal*pt;

	if (membershipBodies.empty())
	{
		return logLikelihood(exp(-(pt_local.squaredNorm())));
	}

	// Check bounding sphere first
	const double r = this->boundingSphere.radius;// + PADDING; // Pretty sure padding is already included...
	const double radiusSquared = r*r;
	if ((this->boundingSphere.center - pt_local).squaredNorm() > radiusSquared)
	{
		return std::numeric_limits<double>::lowest();
	}
//	else { return std::numeric_limits<double>::max(); }

	// Check member shapes
	for (const auto& shape : membershipBodies)
	{
		if (shape->containsPoint(pt_local))
		{
			return std::numeric_limits<double>::max();
		}
	}
	return std::numeric_limits<double>::lowest();
}

double RigidMotionModel::membershipLikelihood(const Eigen::Vector3d& pt, const SensorModel& sensor) const
{
	const Eigen::Vector3d cam_local = localTglobal*sensor.cameraOrigin_world;
	const Eigen::Vector3d pt_local = localTglobal*pt;
	const Eigen::Vector3d dir = (pt_local - cam_local).normalized();

	if (!sphereIntersectsRay(this->boundingSphere, cam_local, dir))
	{
		return std::numeric_limits<double>::lowest();
	}
//
//	double L = membershipLikelihood(pt);
//	if (L > 0.0)
//	{
//		return L;
//	}

	for (const auto& shape : membershipBodies)
	{
		EigenSTL::vector_Vector3d intersections;
		if (shape->intersectsRay(cam_local, dir, &intersections))
		{
			// Project Intersections
			return std::numeric_limits<double>::max();
		}
	}

	return std::numeric_limits<double>::lowest();
}

Eigen::Vector3d RigidMotionModel::expectedVelocity(const Eigen::Vector3d& pt, const MotionParameters& theta) const
{
	assert(theta.size() == 6);

	return linear_part(theta) + angular_part(theta).cross(localTglobal*pt);
}

MotionModel::MotionParameters estimateRigidTransform3D(const Eigen::Matrix3Xd& A, const Eigen::Matrix3Xd& B)
{
	// Compute the centroids of the point sets
	Eigen::Vector3d centroidA = A.rowwise().mean();
	Eigen::Vector3d centroidB = B.rowwise().mean();

	// Center the point sets
	Eigen::Matrix3Xd AA = A.colwise() - centroidA;//.replicate(1, A.cols());
	Eigen::Matrix3Xd BB = B.colwise() - centroidB;//.replicate(1, B.cols());

	Eigen::Matrix3d H = AA * BB.transpose();
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d R = svd.matrixV() * svd.matrixU();
	Eigen::Vector3d t = -R*centroidA + centroidB;
	Eigen::AngleAxisd aa(R);
//	std::cerr << R.log() << std::endl;
	std::cerr << aa.axis() << std::endl;

	MotionModel::MotionParameters theta(RigidMotionModel::MOTION_PARAMETERS_DIMENSION);
	linear_part(theta) = t;
	angular_part(theta) = aa.axis()*aa.angle();

	return theta;
}

bool loadLinkMotionModels(const robot_model::RobotModel* pModel, std::map<std::string, std::shared_ptr<MotionModel>>& motionModels)
{
	for (const robot_model::LinkModel* link : pModel->getLinkModels())
	{
		std::shared_ptr<RigidMotionModel> rmm = std::make_shared<RigidMotionModel>();

		// Don't add no-geometry frames
		if (link->getShapes().empty()) { continue; }
		if (link->getShapes().size() > 1) { std::cerr << "NB: Link '" << link->getName() << "' has " << link->getShapes().size() << " shapes." << std::endl; }

		for (size_t s = 0; s < link->getShapes().size(); ++s)
		{
			const shapes::ShapeConstPtr& shape = link->getShapes()[s];
			auto* body = bodies::createBodyFromShape(shape.get());
			if (!body) {return false;}
			body->setScale(1.0);
			body->setPadding(PADDING);
			body->setPose(link->getCollisionOriginTransforms()[s]);
			rmm->membershipBodies.emplace_back(body);
			rmm->membershipShapes.emplace_back(shape);
		}
		motionModels[link->getName()] = std::move(rmm);
	}

	return true;
}

}

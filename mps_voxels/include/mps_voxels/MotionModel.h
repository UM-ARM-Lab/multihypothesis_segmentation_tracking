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

#ifndef MPS_VOXELS_MOTIONMODEL_H
#define MPS_VOXELS_MOTIONMODEL_H

#include <cmath>

#include <Eigen/Geometry>

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shapes.h>

namespace mps
{

struct SensorModel
{
	Eigen::Vector3d cameraOrigin_world;
};

class MotionModel
{
public:
	using AbstractShapeBase = bodies::Body;
	using MotionParameters = Eigen::VectorXd;
	using Pose = Eigen::Isometry3d;

	/// compute log-likelihood from probability:
	static inline double logLikelihood(double probability){
		return log(probability/(1-probability));
	}

	/// compute probability from logodds:
	static inline double probability(double log_likelihood){
		return 1. - ( 1. / (1. + exp(log_likelihood)));
	}

	virtual void updateMembershipStructures();
	virtual double membershipLikelihood(const Eigen::Vector3d& pt) const = 0;
	virtual double membershipLikelihood(const Eigen::Vector3d& pt, const SensorModel& sensor) const = 0;
	virtual Eigen::Vector3d expectedVelocity(const Eigen::Vector3d& pt, const MotionParameters& theta) const = 0;

	virtual ~MotionModel() = default;

	/// NB: pt_local = localTglobal * pt_global
	Pose localTglobal;

//protected:
	std::vector<std::unique_ptr<AbstractShapeBase>> membershipBodies;
	std::vector<shapes::ShapeConstPtr> membershipShapes;
	bodies::BoundingSphere boundingSphere;

	enum { NeedsToAlign = (sizeof(Pose)%16)==0 };
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
};

#ifndef linear_part
#define linear_part(screw) ((screw).tail<3>())
#endif

#ifndef angular_part
#define angular_part(screw) ((screw).head<3>())
#endif

#ifndef planar_part
#define planar_part(screw) ((screw).segment<3>(2))
#endif

class RigidMotionModel : public MotionModel
{
public:
	static const int MOTION_PARAMETERS_DIMENSION = 6;
	double membershipLikelihood(const Eigen::Vector3d& pt_global) const override;
	double membershipLikelihood(const Eigen::Vector3d& pt, const SensorModel& sensor) const override;
	Eigen::Vector3d expectedVelocity(const Eigen::Vector3d& pt_global, const MotionParameters& theta) const override;
};

MotionModel::MotionParameters estimateRigidTransform3D(const Eigen::Matrix3Xd& A, const Eigen::Matrix3Xd& B);

}
#include <moveit/robot_model/robot_model.h>

namespace mps
{

bool loadLinkMotionModels(const robot_model::RobotModel* pModel, std::map<std::string, std::shared_ptr<MotionModel>>& motionModels);

}

#endif // MPS_VOXELS_MOTIONMODEL_H

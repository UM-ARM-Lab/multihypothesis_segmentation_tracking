//
// Created by arprice on 8/2/18.
//

#ifndef MPS_VOXELS_MOTIONMODEL_H
#define MPS_VOXELS_MOTIONMODEL_H

#include <cmath>

#include <Eigen/Geometry>

#include <geometric_shapes/bodies.h>

class MotionModel
{
public:
	using AbstractShapeBase = bodies::Body;
	using MotionParameters = Eigen::VectorXd;
	using Pose = Eigen::Isometry3d;

	/// compute log-likelihood from probability:
	inline double logLikelihood(double probability){
		return log(probability/(1-probability));
	}

	/// compute probability from logodds:
	inline double probability(double log_likelihood){
		return 1. - ( 1. / (1. + exp(log_likelihood)));
	}

	virtual void updateMembershipStructures();
	virtual double membershipLikelihood(const Eigen::Vector3d& pt) const = 0;
	virtual Eigen::Vector3d expectedVelocity(const Eigen::Vector3d& pt, const MotionParameters& theta) const = 0;

	/// NB: pt_local = localTglobal * pt_global
	Pose localTglobal;

//protected:
	std::vector<std::unique_ptr<AbstractShapeBase>> membershipShapes;
	bodies::BoundingSphere boundingSphere;
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
	double membershipLikelihood(const Eigen::Vector3d& pt_global) const override;
	Eigen::Vector3d expectedVelocity(const Eigen::Vector3d& pt_global, const MotionParameters& theta) const override;
};

#include <moveit/robot_model/robot_model.h>

bool loadLinkMotionModels(const robot_model::RobotModel* pModel, std::map<std::string, std::unique_ptr<MotionModel>>& motionModels);

#endif // MPS_VOXELS_MOTIONMODEL_H

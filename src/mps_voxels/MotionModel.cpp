//
// Created by arprice on 8/2/18.
//

#include "mps_voxels/MotionModel.h"

#include <geometric_shapes/body_operations.h>

static const double PADDING = 0.025;

void MotionModel::updateMembershipStructures()
{
	std::vector<bodies::BoundingSphere> bspheres(membershipShapes.size());
	for (size_t i = 0; i < membershipShapes.size(); ++i)
	{
		membershipShapes[i]->computeBoundingSphere(bspheres[i]);
	}
	bodies::mergeBoundingSpheres(bspheres, boundingSphere);
}

double RigidMotionModel::membershipLikelihood(const Eigen::Vector3d& pt) const
{
	const Eigen::Vector3d& pt_local = localTglobal*pt;

	// Check bounding sphere first
	const double r = this->boundingSphere.radius;// + PADDING; // Pretty sure padding is already included...
	const double radiusSquared = r*r;
	if ((this->boundingSphere.center - pt_local).squaredNorm() > radiusSquared)
	{
		return std::numeric_limits<double>::lowest();
	}
//	else { return std::numeric_limits<double>::max(); }

	// Check member shapes
	for (const auto& shape : membershipShapes)
	{
		if (shape->containsPoint(pt_local))
		{
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

bool loadLinkMotionModels(const robot_model::RobotModel* pModel, std::map<std::string, std::unique_ptr<MotionModel>>& motionModels)
{
	for (const robot_model::LinkModel* link : pModel->getLinkModels())
	{
		std::unique_ptr<RigidMotionModel> rmm = std::make_unique<RigidMotionModel>();

		// Don't add no-geometry frames
		if (link->getShapes().empty()) { continue; }

		for (size_t s = 0; s < link->getShapes().size(); ++s)
		{
			const shapes::ShapeConstPtr& shape = link->getShapes()[s];
			auto* body = bodies::createBodyFromShape(shape.get());
			if (!body) {return false;}
			body->setScale(1.0);
			body->setPadding(PADDING);
			body->setPose(link->getCollisionOriginTransforms()[s]);
			rmm->membershipShapes.emplace_back(body);
		}
		motionModels[link->getName()] = std::move(rmm);
	}

	return true;
}
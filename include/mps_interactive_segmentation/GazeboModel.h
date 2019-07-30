//
// Created by kunhuang on 7/29/19.
//

#ifndef SRC_GAZEBOMODEL_H
#define SRC_GAZEBOMODEL_H

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/bodies.h"

namespace mps
{

class GazeboModel
{
public:
	std::string name;

	std::map<std::string, shapes::ShapePtr> shapes;
	std::map<std::string, bodies::BodyPtr> bodies;
	bodies::BoundingSphere bSphere;

	void computeBoundingSphere();
};

bool rayIntersectsSphere(const bodies::BoundingSphere& sphere,
                         const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
                         EigenSTL::vector_Vector3d* intersections = nullptr, unsigned int count = 0);

bool rayIntersectsBody(const Eigen::Vector3d& rayOrigin_link,
                       const Eigen::Vector3d& rayVector_link,
                       const bodies::Body* b,
                       const shapes::Shape* s,
                       EigenSTL::vector_Vector3d& intersections);

bool rayIntersectsModel(const Eigen::Vector3d& rayOrigin_model,
                        const Eigen::Vector3d& rayVector_model,
                        const GazeboModel& m,
                        EigenSTL::vector_Vector3d& intersections);

}

#endif //SRC_GAZEBOMODEL_H

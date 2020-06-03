//
// Created by kunhuang on 7/29/19.
//

#include "mps_simulation/GazeboModel.h"

#include <geometric_shapes/body_operations.h>

#include <CGAL/Cartesian_d.h>
#include <CGAL/Min_sphere_of_spheres_d.h>

namespace mps
{

static const double NEARLY_ZERO = 1e-9;

static inline double distanceSQR(const Eigen::Vector3d& p, const Eigen::Vector3d& origin, const Eigen::Vector3d& dir)
{
	Eigen::Vector3d a = p - origin;
	double d = dir.normalized().dot(a);
	return a.squaredNorm() - d * d;
}

inline
bool pointIntersectsSphere(const bodies::BoundingSphere& sphere,
                           const Eigen::Vector3d& pt)
{
	double radius2 = sphere.radius * sphere.radius;
	double dist2 = (sphere.center - pt).squaredNorm();
	return dist2 <= radius2;
}

bool rayIntersectsSphere(const bodies::BoundingSphere& sphere,
                         const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
                         EigenSTL::vector_Vector3d* intersections, unsigned int count)
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

// From: https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
bool rayIntersectsTriangle(const Eigen::Vector3d& rayOrigin,
                           const Eigen::Vector3d& rayVector,
                           const unsigned int triangleIdx,
                           const unsigned int* triangles,
                           const double* vertices,
                           Eigen::Vector3d& outIntersectionPoint)
{
	const double EPSILON = 0.0000001;

	Eigen::Map<const Eigen::Vector3d> vertex0(vertices + 3 * triangles[3 * triangleIdx + 0]);
	Eigen::Map<const Eigen::Vector3d> vertex1(vertices + 3 * triangles[3 * triangleIdx + 1]);
	Eigen::Map<const Eigen::Vector3d> vertex2(vertices + 3 * triangles[3 * triangleIdx + 2]);
	Eigen::Vector3d edge1, edge2, h, s, q;
	double a,f,u,v;
	edge1 = vertex1 - vertex0;
	edge2 = vertex2 - vertex0;
	h = rayVector.cross(edge2);
	a = edge1.dot(h);
	if (a > -EPSILON && a < EPSILON)
		return false;    // This ray is parallel to this triangle.
	f = 1.0/a;
	s = rayOrigin - vertex0;
	u = f * s.dot(h);
	if (u < 0.0 || u > 1.0)
		return false;
	q = s.cross(edge1);
	v = f * rayVector.dot(q);
	if (v < 0.0 || u + v > 1.0)
		return false;
	// At this stage we can compute t to find out where the intersection point is on the line.
	double t = f * edge2.dot(q); // t is correct
	if (t > EPSILON) // ray intersection
	{
		outIntersectionPoint = rayOrigin + rayVector * t;
		return true;
	}
	else // This means that there is a line intersection but not a ray intersection.
		return false;
}

bool rayIntersectsMesh(const Eigen::Vector3d& rayOrigin,
                       const Eigen::Vector3d& rayVector,
                       const shapes::Mesh* m,
                       EigenSTL::vector_Vector3d& intersections)
{
	Eigen::Vector3d rayVectorNormal = rayVector;
	rayVectorNormal.normalize();
//	#pragma omp parallel
	for (size_t tIdx = 0; tIdx < m->triangle_count; ++tIdx)
	{
		Eigen::Vector3d intersection;

		if (rayIntersectsTriangle(rayOrigin, rayVectorNormal, tIdx, m->triangles, m->vertices, intersection))
		{
//			#pragma omp critical
			{
				intersections.push_back(intersection);
			}
		}
	}
	return !intersections.empty();
}

bool rayIntersectsBody(const Eigen::Vector3d& rayOrigin_link,
                       const Eigen::Vector3d& rayVector_link,
                       const bodies::Body* b,
                       const shapes::Shape* s,
                       EigenSTL::vector_Vector3d& intersections)
{
	const auto* mesh = dynamic_cast<const shapes::Mesh*>(s);
	if (mesh)
	{
		const auto bodyTparent = b->getPose().inverse();
		const auto rayOrigin = bodyTparent * rayOrigin_link;
		const auto rayVector = bodyTparent.linear() * rayVector_link;

		return rayIntersectsMesh(rayOrigin, rayVector, mesh, intersections);
	}
	else
	{
		return b->intersectsRay(rayOrigin_link, rayVector_link, &intersections);
	}
}

bool rayIntersectsModel(const Eigen::Vector3d& rayOrigin_model,
                        const Eigen::Vector3d& rayVector_model,
                        const GazeboModel& model,
                        EigenSTL::vector_Vector3d& intersections)
{
//	return rayIntersectsSphere(model.bSphere, rayOrigin_model, rayVector_model, &intersections);
	if (rayIntersectsSphere(model.bSphere, rayOrigin_model, rayVector_model, nullptr))
	{
		intersections.clear();
		for (const auto& body_pair : model.bodies)
		{
			rayIntersectsBody(rayOrigin_model, rayVector_model, body_pair.second.get(), model.shapes.at(body_pair.first).get(), intersections);
		}
		return !intersections.empty();
	}
	else
	{
		return false;
	}
}

bool pointIntersectsModel(const Eigen::Vector3d& point_model,
                          const GazeboModel& model)
{
	if (pointIntersectsSphere(model.bSphere, point_model))
	{
		for (const auto& body_pair : model.bodies)
		{
			if (body_pair.second->containsPoint(point_model))
			{
				return true;
			}
		}
	}
	return false;
}

void GazeboModel::computeBoundingSphere()
{
	typedef double FT;
	typedef CGAL::Cartesian_d<FT> K;
	typedef CGAL::Min_sphere_of_spheres_d_traits_d<K,FT,3> Traits;
	typedef CGAL::Min_sphere_of_spheres_d<Traits> Min_sphere;
	typedef K::Point_d Point;
	typedef Traits::Sphere Sphere;

	std::vector<bodies::BoundingSphere> bSpheres;//bodies.size());
//	int i = 0;
	for (const auto& pair : bodies)
	{
		// For meshes, we want to use CGAL's more sophisticated sphere-fitting algorithm
		const auto& mesh = std::dynamic_pointer_cast<const shapes::Mesh>(shapes.at(pair.first));
		if (mesh)
		{
			std::vector<Sphere> S;
			for (int pIdx = 0; pIdx < static_cast<int>(mesh->vertex_count); ++pIdx)
			{
				Eigen::Map<Eigen::Vector3d> pt_mesh(mesh->vertices + (pIdx * 3));
				Eigen::Vector3d pt_model = pair.second->getPose() * pt_mesh;
//				std::cerr << "Mesh Point: " << pt_mesh.transpose() << std::endl;
//				std::cerr << "Mesh Pose:\n" << pair.second->getPose().matrix() << std::endl;
//				std::cerr << "Model Point: " << pt_mesh.transpose() << std::endl;

				Point pt(3, pt_model.data(), pt_model.data() + 3);
//				Point pt(3, mesh->vertices+(pIdx*3), mesh->vertices+(pIdx*3) + 3);
//				std::cerr << "pt: " << Eigen::Map<const Eigen::Vector3d>(pt.cartesian_begin()).transpose() << std::endl;
				S.emplace_back(Sphere(pt, pair.second->getPadding() + 1e-3));
			}
			Min_sphere min_sphere(S.begin(), S.end());
			if (min_sphere.is_valid())
			{
				bSpheres.emplace_back(bodies::BoundingSphere{Eigen::Map<const Eigen::Vector3d>(min_sphere.center_cartesian_begin()), min_sphere.radius()});
//				bSpheres[i].center = Eigen::Map<const Eigen::Vector3d>(min_sphere.center_cartesian_begin());
//				bSpheres[i].radius = min_sphere.radius();
//				std::cerr << bSpheres.back().center.transpose() << std::endl;
//				std::cerr << bSpheres.back().radius << std::endl;
//				++i;
				continue;
			}
			else
			{
				throw std::runtime_error("CGAL failed to compute bounding sphere for '" + name + "'.");
			}
		}

		bodies::BoundingSphere sphere;
		pair.second->computeBoundingSphere(sphere);
		bSpheres.push_back(sphere);
	}
	bodies::mergeBoundingSpheres(bSpheres, this->bSphere);
}

}

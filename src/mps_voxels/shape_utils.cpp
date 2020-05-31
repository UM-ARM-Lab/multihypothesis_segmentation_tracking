//
// Created by arprice on 8/31/18.
//

#include "mps_voxels/shape_utils.h"
#include "mps_voxels/shape_utils_impl.hpp"
#include "mps_voxels/octree_utils.h"
#include "mps_voxels/util/assert.h"

#include <octomap/octomap_types.h>

#include <geometric_shapes/shape_operations.h>

namespace mps
{

template std::shared_ptr<shapes::Mesh> convex_hull<octomap::point3d_collection, octomap::point3d>(const octomap::point3d_collection&);
template std::shared_ptr<shapes::Mesh> prism<octomap::point3d_collection, octomap::point3d>(const octomap::point3d_collection&);
template std::shared_ptr<shapes::Mesh> ZAMBB<octomap::point3d_collection, octomap::point3d>(const octomap::point3d_collection&);

std::shared_ptr<shapes::Mesh> approximateShape(const octomap::OcTree* tree)
{
	// Convex hull
	octomap::point3d_collection pts = getPoints(tree);
//	std::shared_ptr<shapes::Mesh> hull = convex_hull(pts);
	std::shared_ptr<shapes::Mesh> hull = prism(pts);
//	std::shared_ptr<shapes::Mesh> hull = ZAMBB(pts);
	return hull;
}

void getAABB(const shapes::Mesh& shape, Eigen::Vector3d& min, Eigen::Vector3d& max)
{
	min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
	max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
//	const int DIM = 3;
//	for (int d=0; d < DIM; ++d)
//	{
//		min[d] = std::numeric_limits<float>::infinity();
//		max[d] = -std::numeric_limits<float>::infinity();
//	}

//	#pragma omp parallel for reduction(max : max) reduction(min : min)
	for (unsigned i = 0; i < shape.vertex_count; ++i)
	{
		Eigen::Map<Eigen::Vector3d> vertex(shape.vertices + (3*i));
		min = min.cwiseMin(vertex);
		max = max.cwiseMax(vertex);
//		for (int d=0; d < DIM; ++d)
//		{
//			min[d] = std::min(min[d], static_cast<float>(vertex[d]));
//			max[d] = std::max(max[d], static_cast<float>(vertex[d]));
//		}
	}
}

}

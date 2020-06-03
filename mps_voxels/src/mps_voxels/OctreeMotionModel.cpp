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

#include "mps_voxels/OctreeMotionModel.h"
#include "mps_voxels/octree_utils.h"
#include "mps_voxels/LocalOctreeServer.h"

#include <CGAL/Cartesian_d.h>
#include <CGAL/Min_sphere_of_spheres_d.h>

namespace mps
{

void OctreeMotionModel::updateMembershipStructures()
{
//	typedef CGAL::Exact_rational              FT;
	typedef double                            FT;
	typedef CGAL::Cartesian_d<FT>             K;
	typedef CGAL::Min_sphere_of_spheres_d_traits_d<K,FT,3> Traits;
	typedef CGAL::Min_sphere_of_spheres_d<Traits> Min_sphere;
	typedef K::Point_d                        Point;
	typedef Traits::Sphere                    Sphere;

	std::vector<Sphere> S;

	for (octomap::OcTree::iterator it = octree->begin(octree->getTreeDepth()),
		     end = octree->end(); it != end; ++it)
	{
		if (octree->isNodeOccupied(*it))
		{
//			unsigned idx = it.getDepth();

			auto p = it.getCoordinate();
			Point center((FT)p.x(), (FT)p.y(), (FT)p.z());
			S.emplace_back(Sphere(center, it.getSize()));
		}
	}

	Min_sphere ms(S.begin(), S.end());
	if (ms.is_valid())
	{
		this->boundingSphere.center.x() = ms.center_cartesian_begin()[0];
		this->boundingSphere.center.y() = ms.center_cartesian_begin()[1];
		this->boundingSphere.center.z() = ms.center_cartesian_begin()[2];
		this->boundingSphere.radius = ms.radius();
	}
}

double OctreeMotionModel::membershipLikelihood(const Eigen::Vector3d& pt_global) const
{
	const Eigen::Vector3d pt_local = localTglobal*pt_global;

	octomap::OcTreeNode* node = octree->search(pt_local.x(), pt_local.y(), pt_local.z());
	if (!node)
	{
		return std::numeric_limits<double>::lowest();
	}
	else
	{
		return node->getOccupancy();
	}
}

double OctreeMotionModel::membershipLikelihood(const Eigen::Vector3d& pt, const SensorModel&) const
{
	return OctreeMotionModel::membershipLikelihood(pt);
}

bool OctreeMotionModel::observe(const pcl::PointCloud<PointT>::Ptr& cloud, const Eigen::Affine3d& globalTcamera)
{
	Eigen::Affine3d lTg = Eigen::Affine3d::Identity();
	lTg.linear() = localTglobal.linear();
	lTg.translation() = localTglobal.translation();

	Eigen::Affine3d localTcamera = lTg * globalTcamera;

	insertCloudInOctree(cloud, localTcamera, octree.get());

	return true;
}

bool OctreeMotionModel::observe(const pcl::PointCloud<PointT>::Ptr& cloud, const Eigen::Isometry3d& globalTcamera)
{
	const Pose localTcamera = this->localTglobal * globalTcamera;

	Eigen::Affine3d cameraPose = Eigen::Affine3d::Identity();
	cameraPose.linear() = localTcamera.linear();
	cameraPose.translation() = localTcamera.translation();

	insertCloudInOctree(cloud, localTcamera, octree.get());

	return true;
}

}

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

#ifndef MPS_LOCALOCTREESERVER_H
#define MPS_LOCALOCTREESERVER_H

#include "mps_voxels/PointT.h"

#include <octomap/OcTree.h>
#include <ros/node_handle.h>

namespace mps
{

template <typename T>
void setIfMissing(ros::NodeHandle& nh, const std::string& param_name, const T& param_val)
{
	if (!nh.hasParam(param_name))
	{
		nh.setParam(param_name, param_val);
	}
}

template <typename Point>
void setBBox(const Point& min, const Point& max, octomap::OcTree* ot)
{
	octomap::point3d om_min{(float)min.x(), (float)min.y(), (float)min.z()};
	octomap::point3d om_max{(float)max.x(), (float)max.y(), (float)max.z()};
	ot->setBBXMin(om_min);
	ot->setBBXMax(om_max);
	ot->useBBXLimit(true);
}

/**
 * @brief Takes a point cloud in camera coordinates (z=depth), a point of view, and incorporates it into a tree map
 * @param cloud
 * @param worldTcamera
 * @param tree
 */
void insertCloudInOctree(const pcl::PointCloud<PointT>::Ptr& cloud, const Eigen::Affine3d& worldTcamera, octomap::OcTree* tree);

class LocalOctreeServer// : public octomap_server::OctomapServer
{
public:
	using OcTreeT = octomap::OcTree;

	double m_res;
	std::string m_worldFrameId; // the map frame
	std::unique_ptr<OcTreeT> m_octree;

	LocalOctreeServer(const double res, const std::string& worldFrameId);

	void insertCloud(const pcl::PointCloud<PointT>::Ptr& cloud, const Eigen::Affine3d& worldTcamera);

	const std::string& getWorldFrame() const { return this->m_worldFrameId; }
	const OcTreeT* getOctree() const { return this->m_octree.get(); }
	OcTreeT* getOctree() { return this->m_octree.get(); }
};

}

#endif // MPS_LOCALOCTREESERVER_H

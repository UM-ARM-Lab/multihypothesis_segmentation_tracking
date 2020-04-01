//
// Created by pricear on 2020-03-26.
//

#ifndef MPS_VOXELREGIONBUILDER_HPP
#define MPS_VOXELREGIONBUILDER_HPP

#include "mps_voxels/parameters/ParameterBlock.hpp"
#include "mps_voxels/VoxelRegion.h"

namespace mps
{

struct VoxelRegionBuilder
{
	template <typename ParameterProvider>
	static VoxelRegion build(const ParameterProvider& p)
	{
		// TODO: Remove "roi/"
		Eigen::Vector3d minExtent(getParam<double>(p, "roi/min/x"),
		                          getParam<double>(p, "roi/min/y"),
		                          getParam<double>(p, "roi/min/z"));
		Eigen::Vector3d maxExtent(getParam<double>(p, "roi/max/x"),
		                          getParam<double>(p, "roi/max/y"),
		                          getParam<double>(p, "roi/max/z"));
		double resolution = getParam<double>(p, "roi/resolution");

		mps::VoxelRegion::vertex_descriptor dims = roiToVoxelRegion(resolution, minExtent, maxExtent);
		return {dims, resolution, minExtent, maxExtent};
	}
};

}
#endif //MPS_VOXELREGIONBUILDER_HPP

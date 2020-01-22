//
// Created by arprice on 10/18/18.
//

#ifndef MPS_VOXELS_CUDATRACKER_H
#define MPS_VOXELS_CUDATRACKER_H

#ifdef HAS_CUDA_SIFT

#include "mps_voxels/Tracker.h"

namespace mps
{

class CudaTracker : public Tracker
{
public:
	CudaTracker(tf::TransformListener* _listener,
	            TrackingOptions _track_options = TrackingOptions());

	void track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, LabelT label = 0) override;
};

}

#endif // HAS_CUDA_SIFT
#endif // MPS_VOXELS_CUDATRACKER_H

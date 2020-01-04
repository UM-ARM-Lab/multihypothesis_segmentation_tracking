//
// Created by arprice on 10/18/18.
//

#ifndef MPS_VOXELS_CUDATRACKER_H
#define MPS_VOXELS_CUDATRACKER_H

#ifdef HAS_CUDA_SIFT

#include "mps_voxels/Tracker.h"

class CudaTracker : public Tracker
{
public:
	CudaTracker(tf::TransformListener* _listener,
	            const size_t _buffer = 500,
	            SubscriptionOptions _options = SubscriptionOptions(),
	            TrackingOptions _track_options = TrackingOptions());

	void track(const std::vector<ros::Time>& steps) override;
};

#endif // HAS_CUDA_SIFT
#endif // MPS_VOXELS_CUDATRACKER_H

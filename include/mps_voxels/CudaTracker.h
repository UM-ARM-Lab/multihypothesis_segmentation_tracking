//
// Created by arprice on 10/18/18.
//

#ifndef MPS_VOXELS_CUDATRACKER_H
#define MPS_VOXELS_CUDATRACKER_H

#include "mps_voxels/Tracker.h"

class CudaTracker : public Tracker
{
public:
	CudaTracker(const size_t _buffer = 500,
	            SubscriptionOptions _options = SubscriptionOptions(),
	            TrackingOptions _track_options = TrackingOptions());

	void track() override;
};

#endif // MPS_VOXELS_CUDATRACKER_H

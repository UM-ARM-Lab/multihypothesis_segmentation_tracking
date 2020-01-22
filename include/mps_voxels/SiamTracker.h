//
// Created by kunhuang on 9/24/19.
//

#ifndef SRC_SIAMTRACKER_H
#define SRC_SIAMTRACKER_H

#include "mps_voxels/Tracker.h"
#include "mps_msgs/AABBox2d.h"

namespace mps
{

class SiamTracker : public Tracker
{
public:
	SiamTracker(TrackingOptions _track_options = TrackingOptions());

	void track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer) override;

	using LabelT = uint16_t;

	void siamtrack(LabelT label, const std::vector<ros::Time>& steps, mps_msgs::AABBox2d bbox, const SensorHistoryBuffer& buffer);
};

}

#endif //SRC_SIAMTRACKER_H

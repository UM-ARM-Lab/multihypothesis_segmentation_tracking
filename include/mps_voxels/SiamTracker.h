//
// Created by kunhuang on 9/24/19.
//

#ifndef SRC_SIAMTRACKER_H
#define SRC_SIAMTRACKER_H

#include "mps_voxels/Tracker.h"
#include "mps_msgs/AABBox2d.h"

class SiamTracker : public Tracker
{
public:
	SiamTracker(tf::TransformListener* _listener,
	const size_t _buffer = 500,
		SubscriptionOptions _options = SubscriptionOptions(),
		TrackingOptions _track_options = TrackingOptions());

	void track(const std::vector<ros::Time>& steps) override;

	void siamtrack(const std::vector<ros::Time>& steps, mps_msgs::AABBox2d bbox) override;
};


#endif //SRC_SIAMTRACKER_H

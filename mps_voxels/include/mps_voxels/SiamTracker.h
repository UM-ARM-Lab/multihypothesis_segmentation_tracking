//
// Created by kunhuang on 9/24/19.
//

#ifndef SRC_SIAMTRACKER_H
#define SRC_SIAMTRACKER_H

#include "mps_voxels/Tracker.h"
//#include "mps_msgs/AABBox2d.h"

#include <actionlib/client/simple_action_client.h>
#include <mps_msgs/TrackBBoxAction.h>

namespace mps
{

class DenseTracker
{
public:
	virtual
	bool track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, const cv::Mat& initMask, std::map<ros::Time, cv::Mat>& masks) = 0;

	virtual
	bool track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, const mps_msgs::AABBox2d& initRegion, std::map<ros::Time, cv::Mat>& masks) = 0;

	virtual
	~DenseTracker() = default;

};

class SiamTracker : public DenseTracker
{
public:
	SiamTracker();

	actionlib::SimpleActionClient<mps_msgs::TrackBBoxAction> actionClient;

	bool track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, const cv::Mat& initMask, std::map<ros::Time, cv::Mat>& masks) override;

	bool track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, const mps_msgs::AABBox2d& initRegion, std::map<ros::Time, cv::Mat>& masks) override;

	using LabelT = uint16_t;

//	void siamtrack(LabelT label, const std::vector<ros::Time>& steps, mps_msgs::AABBox2d bbox, const SensorHistoryBuffer& buffer);
};

}

#endif //SRC_SIAMTRACKER_H

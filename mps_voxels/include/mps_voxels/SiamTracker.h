//
// Created by kunhuang on 9/24/19.
//

#ifndef SRC_SIAMTRACKER_H
#define SRC_SIAMTRACKER_H

#include "mps_voxels/Tracker.h"
#include "mps_voxels/logging/DataLog.h"

#include <actionlib/client/simple_action_client.h>
#include <mps_msgs/TrackBBoxAction.h>

namespace mps
{

class DenseTracker
{
public:
	virtual
	bool track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, uint16_t label, const cv::Mat& initMask, std::map<ros::Time, cv::Mat>& masks) = 0;

	virtual
	bool track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, uint16_t label, const mps_msgs::AABBox2d& initRegion, std::map<ros::Time, cv::Mat>& masks) = 0;

	virtual
	bool isHistoryTracker(std::string& fname) = 0;

	virtual
	~DenseTracker() = default;

};

class SiamTracker : public DenseTracker
{
public:
	SiamTracker(const std::string& path = "");

	actionlib::SimpleActionClient<mps_msgs::TrackBBoxAction> actionClient;

	std::string trackingFilename;

	bool track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, uint16_t label, const cv::Mat& initMask, std::map<ros::Time, cv::Mat>& masks) override;

	bool track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, uint16_t label, const mps_msgs::AABBox2d& initRegion, std::map<ros::Time, cv::Mat>& masks) override;

	bool isHistoryTracker(std::string& fname) override;
};

class HistoryTracker : public DenseTracker
{
public:
	DataLog logger;

	std::string trackingFilename;

	HistoryTracker(const std::string& path);

	bool track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, uint16_t label, const cv::Mat& initMask, std::map<ros::Time, cv::Mat>& masks) override;

	bool track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, uint16_t label, const mps_msgs::AABBox2d& initRegion, std::map<ros::Time, cv::Mat>& masks) override;

	bool isHistoryTracker(std::string& fname) override;
};

}

#endif //SRC_SIAMTRACKER_H

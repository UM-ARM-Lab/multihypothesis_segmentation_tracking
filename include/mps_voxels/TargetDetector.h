//
// Created by arprice on 9/11/18.
//

#ifndef MPS_TARGETDETECTOR_H
#define MPS_TARGETDETECTOR_H

#include <opencv2/core.hpp>

namespace mps
{

void
getHSVMask(const cv::Mat& img, cv::Mat& mask, const cv::Scalar& minHSV, const cv::Scalar& maxHSV, const int morphSize);

class TargetDetector
{
public:
	enum class TRACK_COLOR
	{
		GREEN,
		PURPLE
	};
	cv::Scalar minHSV, maxHSV;

	explicit
	TargetDetector(TRACK_COLOR color = TRACK_COLOR::PURPLE);

	cv::Mat getMask(const cv::Mat& img) const;

	int matchGoalSegment(const cv::Mat& goalMask, const cv::Mat& labels) const;

	int sizeThreshold = 100;
	double overlapThreshold = 0.5;
};

}

#endif // MPS_TARGETDETECTOR_H

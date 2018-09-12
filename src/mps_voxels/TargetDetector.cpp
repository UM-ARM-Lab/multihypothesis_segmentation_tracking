//
// Created by arprice on 9/11/18.
//

#include "mps_voxels/TargetDetector.h"
#include "mps_voxels/image_utils.h"

#include <opencv2/imgproc.hpp>

void getHSVMask(const cv::Mat& img, cv::Mat& mask, const cv::Scalar& minHSV, const cv::Scalar& maxHSV, const int morphSize)
{
	cv::Mat imgHSV, morphKernel;
	cv::cvtColor(img, imgHSV, CV_BGR2HSV);
	cv::inRange(imgHSV, minHSV, maxHSV, mask);

	morphKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morphSize, morphSize));

	cv::erode(mask, mask, morphKernel);
	cv::dilate(mask, mask, morphKernel);
	cv::dilate(mask, mask, morphKernel);
	cv::erode(mask, mask, morphKernel);

}

TargetDetector::TargetDetector(TRACK_COLOR color)
{
	// NB: OpenCV's HSV H-range is 0-180 rather than 0-360, to keep 1-byte sizes, so we divide by 2
	if (TRACK_COLOR::PURPLE == color)
	{
		minHSV = cv::Scalar(240/2.0, 50, 50);
		maxHSV = cv::Scalar(270/2.0, 256, 256);
	}
	else if (TRACK_COLOR::GREEN == color)
	{
		minHSV = cv::Scalar(64/2.0, 100, 100);
		maxHSV = cv::Scalar(85/2.0, 256, 256);
	}
	else { throw std::logic_error("Unknown color to track."); }
}

cv::Mat TargetDetector::getMask(const cv::Mat& img) const
{
	cv::Mat mask;
	getHSVMask(img, mask, minHSV, maxHSV, 7);
	return mask;
}

int TargetDetector::matchGoalSegment(const cv::Mat& goalMask, const cv::Mat& labels) const
{
	assert(roi.width == labels.cols);
	assert(roi.height == labels.rows);
	using LabelT = uint16_t;

	std::set<LabelT> uniqueLabels = unique(labels);
	int goalID = -1;

	for (const auto label : uniqueLabels)
	{
		// Erode the boundary of the label slightly
		cv::Mat labelMask = (labels == label);
		cv::erode(labelMask, labelMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

		int labelNum = cv::countNonZero(labelMask);
		int overlapNum = cv::countNonZero(labelMask & goalMask);

		if ((overlapNum > sizeThreshold)
		    && (((double)overlapNum/(double)labelNum) > overlapThreshold))
		{
			goalID = label;
			break;
		}
	}

	return goalID;
}
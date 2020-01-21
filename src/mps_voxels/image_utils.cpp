//
// Created by arprice on 8/29/18.
//

#include "mps_voxels/image_utils.h"
#include "mps_voxels/SensorHistorian.h"
#include "mps_voxels/ROI.h"

pcl::PointCloud<PointT>::Ptr imagesToCloud(const cv::Mat& rgb, const cv::Mat& depth, const image_geometry::PinholeCameraModel& cameraModel)
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
	cloud->resize(cameraModel.cameraInfo().width * cameraModel.cameraInfo().height);

#pragma omp parallel for
	for (int v = 0; v < (int)cameraModel.cameraInfo().height; ++v)
	{
		for (int u = 0; u < (int)cameraModel.cameraInfo().width; ++u)
		{
			int idx = v * (int)cameraModel.cameraInfo().width + u;

			auto color = rgb.at<cv::Vec3b>(v, u);
			auto dVal = depth.at<uint16_t>(v, u);

			float depthVal = mps::SensorHistorian::DepthTraits::toMeters(dVal); // if (depth1 > maxZ || depth1 < minZ) { continue; }
			PointT pt(color[2], color[1], color[0]);
			pt.getVector3fMap() = toPoint3D<Eigen::Vector3f>(u, v, depthVal, cameraModel);
			cloud->points[idx] = pt;
		}
	}

	return cloud;
}

cv::Scalar randomColor( cv::RNG& rng )
{
	int icolor = (unsigned) rng;
	return cv::Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
}

std::set<uint16_t> unique(const cv::Mat& input)
{
	if (input.channels() > 1 || input.type() != CV_16U)
	{
		throw std::logic_error("unique !!! Only works with CV_16U 1-channel Mat");
	}

	std::set<uint16_t> out;

	#pragma omp declare reduction (merge : std::set<uint16_t> : omp_out.insert(omp_in.begin(), omp_in.end()))
	#pragma omp parallel for reduction(merge: out)
	for (int y = 0; y < input.rows; ++y)
	{
		const uint16_t* row_ptr = input.ptr<uint16_t>(y);
		for (int x = 0; x < input.cols; ++x)
		{
			out.insert(row_ptr[x]);
		}
	}

	return out;
}

cv::Mat colorByLabel(const cv::Mat& input)
{
	double min;
	double max;
	cv::minMaxIdx(input, &min, &max);
	cv::Mat labels;
	input.convertTo(labels, CV_8UC1);

	cv::Mat colormap(256, 1, CV_8UC3);
	cv::randu(colormap, 0, 256);

	cv::Mat output;
	cv::applyColorMap(labels, output, colormap);

	return output;
}
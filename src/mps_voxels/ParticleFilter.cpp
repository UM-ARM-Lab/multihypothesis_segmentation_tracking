//
// Created by kunhuang on 2/12/20.
//

#include "mps_voxels/ParticleFilter.h"

namespace mps
{

ParticleFilter::ParticleFilter(const VoxelRegion::vertex_descriptor& dims, const double& res, const Eigen::Vector3d& rmin, const Eigen::Vector3d& rmax, int n)
	: voxRegion(dims, res, rmin, rmax), numParticles(n)
{
	particles.resize(n);
}

Particle ParticleFilter::applyActionModel(const Particle& inputParticle, const image_geometry::PinholeCameraModel& cameraModel,
                                          const Eigen::Isometry3d& worldTcamera,
                                          SensorHistoryBuffer& buffer_out, SegmentationInfo& seg_out,
                                          std::unique_ptr<Tracker>& sparseTracker, std::unique_ptr<DenseTracker>& denseTracker,
                                          const int& segRes)
{
	cv::Mat segParticle = rayCastParticle(inputParticle, cameraModel, worldTcamera, segRes);
	std::cerr << "Segmentation based on particle generated!" << std::endl;

	cv::Rect roi = {0, 0, (int)cameraModel.cameraInfo().width, (int)cameraModel.cameraInfo().height};
	std::map<uint16_t, mps_msgs::AABBox2d> labelToBBoxLookup = getBBox(segParticle, roi);

	std::unique_ptr<objectActionModel> oam = std::make_unique<objectActionModel>(1);
	std::map<int, rigidTF> labelToMotionLookup;
	for (auto& pair : labelToBBoxLookup)
	{
		oam->sampleAction(buffer_out, seg_out, sparseTracker, denseTracker, pair.first, pair.second);
		labelToMotionLookup.insert({pair.first, *oam->actionSamples.begin()});
	}

	Particle outputParticle = moveParticle(inputParticle, labelToMotionLookup);
	return outputParticle;
}


}
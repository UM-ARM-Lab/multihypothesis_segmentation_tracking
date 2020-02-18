//
// Created by kunhuang on 2/12/20.
//

#include "mps_voxels/ParticleFilter.h"

namespace mps
{

ParticleFilter::ParticleFilter(const VoxelRegion::vertex_descriptor& dims, const double& res, const Eigen::Vector3d& rmin, const Eigen::Vector3d& rmax, int n)
	: numParticles(n)
{
	voxelRegion = std::make_shared<VoxelRegion>(dims, res, rmin, rmax);;
	particles.resize(n);
	for (int i=0; i<n; ++i)
	{
		particles[i].voxelRegion = voxelRegion;
		particles[i].init();
	}
}

Particle ParticleFilter::applyActionModel(const Particle& inputParticle, const image_geometry::PinholeCameraModel& cameraModel,
                                          const moveit::Pose& worldTcamera, SensorHistoryBuffer& buffer_out,
                                          std::unique_ptr<Tracker>& sparseTracker, std::unique_ptr<DenseTracker>& denseTracker,
                                          const int& segRes)
{
	cv::Mat segParticle = rayCastParticle(inputParticle, cameraModel, worldTcamera, segRes);
	std::cerr << "Segmentation based on particle generated!" << std::endl;
//	cv::imwrite("/home/kunhuang/Pictures/segParticle.jpg", colorByLabel(segParticle));

	cv::Rect roi = {0, 0, (int)cameraModel.cameraInfo().width, (int)cameraModel.cameraInfo().height};
	std::map<uint16_t, mps_msgs::AABBox2d> labelToBBoxLookup = getBBox(segParticle, roi);
	std::cerr << "number of bounding boxes in segParticle: " << labelToBBoxLookup.size() << std::endl;

	std::unique_ptr<ObjectActionModel> oam = std::make_unique<ObjectActionModel>(1);
	std::map<int, RigidTF> labelToMotionLookup;
	for (auto& pair : labelToBBoxLookup)
	{
		oam->sampleAction(buffer_out, segParticle, sparseTracker, denseTracker, pair.first, pair.second);
		labelToMotionLookup.insert({pair.first - 1, *oam->actionSamples.begin()});
	}

	Particle outputParticle = moveParticle(inputParticle, labelToMotionLookup);
	return outputParticle;
}


}
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

#include <std_msgs/ByteMultiArray.h>

#include <ros/ros.h>
#include <memory>
#include <cassert>

namespace om = octomap;

ros::ServiceClient mapClient;
ros::Publisher localMapPub;

void getOccupancy(const om::point3d& min, const om::point3d& max, const int resolution = 64)
{
	assert(resolution > 0);
	assert(max.x() > min.x());
	assert(max.y() > min.y());
	assert(max.z() > min.z());

	octomap_msgs::GetOctomapRequest req;
	octomap_msgs::GetOctomapResponse resp;
	bool callSucceeded = mapClient.call(req, resp);
	if (!callSucceeded)
	{
		ROS_ERROR("Unable to call Octomap service.");
		return;
	}

	std::shared_ptr<octomap::AbstractOcTree> abstractTree(octomap_msgs::msgToMap(resp.map));
	std::shared_ptr<om::OcTree> octree = std::dynamic_pointer_cast<om::OcTree>(abstractTree);

	if (!octree)
	{
		ROS_ERROR("Unable to downcast abstract octree to concrete tree.");
		return;
	}

	std::cerr << octree->getNumLeafNodes() << ", " << octree->getResolution() << std::endl;

	std_msgs::ByteMultiArray arrayMsg;
	std_msgs::MultiArrayDimension dim;

	dim.label = "x"; dim.size = (unsigned)resolution; dim.stride = (unsigned)resolution*resolution*resolution;
	arrayMsg.layout.dim.push_back(dim);
	dim.label = "y"; dim.size = (unsigned)resolution; dim.stride = (unsigned)resolution*resolution;
	arrayMsg.layout.dim.push_back(dim);
	dim.label = "z"; dim.size = (unsigned)resolution; dim.stride = (unsigned)resolution;
	arrayMsg.layout.dim.push_back(dim);

	int emptyCount = 0;
	int fullCount = 0;
	int unknownCount = 0;
	for (int i = 0; i < resolution; ++i)
	{
		double x = min.x() + (max.x() - min.x())*(i/static_cast<double>(resolution-1));
		for (int j = 0; j < resolution; ++j)
		{
			double y = min.y() + (max.y() - min.y())*(j/static_cast<double>(resolution-1));
			for (int k = 0; k < resolution; ++k)
			{
				double z = min.z() + (max.z() - min.z())*(k/static_cast<double>(resolution-1));

				om::OcTreeNode* node = octree->search(x, y, z);
				if (!node)
				{
					//This cell is unknown
					unknownCount++;
					arrayMsg.data.push_back(0);
				}
				else
				{
					double cellValue = node->getOccupancy() - 0.5;

					// Temp code for testing features
					for (unsigned int d = 0; d < octree->getTreeDepth(); ++d)
					{
						om::OcTreeKey key = octree->coordToKey(x, y, z, d);
						octree->search(key, d);
						octree->search(x, y, z, d);
						octree->getTreeType();
						octree->getNodeSize(d);
						om::point3d p = octree->keyToCoord(key, d);
					}

					if (cellValue > 0)
					{
						fullCount++;
						arrayMsg.data.push_back(1);
					}
					else if (cellValue < 0)
					{
						emptyCount++;
						arrayMsg.data.push_back(-1);
					}
					else
					{
						std::cerr << "Uncertain value at " << x << ", " << y << ", " << z << std::endl;
						arrayMsg.data.push_back(0);
					}
				}
			}
		}
	}
	std::cerr << "Results are " << fullCount<< ", " << emptyCount << ", " << unknownCount << std::endl;
	localMapPub.publish(arrayMsg);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "shape_completion_manager");
	ros::NodeHandle nh;

	localMapPub = nh.advertise<std_msgs::ByteMultiArray>("local_occupancy", 1, true);
	mapClient = nh.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
	if (!mapClient.waitForExistence(ros::Duration(10)))
	{
		ROS_WARN("Map server not connected.");
	}

	getOccupancy(om::point3d(-0.1f, -0.1f, -0.1f), om::point3d(0.1f, 0.1f, 0.1f));

	ros::spinOnce();

	return 0;
}
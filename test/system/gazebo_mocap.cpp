//
// Created by arprice on 2/25/19.
//

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelState.h>

Eigen::Isometry3d randomTransform(const double tScale = 1.0)
{
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	T.rotate(Eigen::Quaterniond(Eigen::Vector4d::Random()).normalized());
	T.translation() = tScale * Eigen::Vector3d::Random();
	return T;
}

struct GazeboFrame
{
	std::string modelName;
	std::string linkName;
	std::string gazeboName() const { return modelName; }
	std::string tfName() const { return modelName; }
	//std::string gazeboName() const { return modelName + "::" + linkName; }
	//std::string tfName() const { return modelName + "_" + linkName; }
};

bool operator<(const GazeboFrame& lhs, const GazeboFrame& rhs)
{
	if (lhs.modelName < rhs.modelName)
	{
		return true;
	}
	return lhs.linkName < rhs.linkName;
}

class GazeboMocap
{
public:
	const std::string mocap_frame_id = "mocap";

	std::unique_ptr<ros::NodeHandle> nh;
	std::unique_ptr<tf::TransformBroadcaster> tb;
	ros::ServiceClient worldStateClient;
	ros::ServiceClient modelStateClient;

	tf::StampedTransform gazeboTmocap;
	std::map<const GazeboFrame, tf::StampedTransform> markerOffsets;

	GazeboMocap();

	void sendTransforms(bool sendBaseFrame = false);
    std::vector<std::string> getModelNames();
};

GazeboMocap::GazeboMocap()
{
	nh = std::make_unique<ros::NodeHandle>();
	tb = std::make_unique<tf::TransformBroadcaster>();
	worldStateClient = nh->serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
	modelStateClient = nh->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    worldStateClient.waitForExistence(ros::Duration(5));
    modelStateClient.waitForExistence(ros::Duration(5));

    tf::transformEigenToTF(randomTransform(), gazeboTmocap);
	gazeboTmocap.frame_id_ = "world";
	gazeboTmocap.child_frame_id_ = mocap_frame_id;
}

void GazeboMocap::sendTransforms(bool sendBaseFrame)
{
	// For debugging
	if (sendBaseFrame)
	{
		gazeboTmocap.stamp_ = ros::Time::now();
		tb->sendTransform(gazeboTmocap);
	}

	for (const auto& pair : markerOffsets)
	{
		gazebo_msgs::GetModelStateRequest req;
		req.model_name = pair.first.gazeboName();
		//req.reference_frame = "";
		gazebo_msgs::GetModelStateResponse resp;
		bool result = modelStateClient.call(req, resp);

		if (!result) { throw std::runtime_error("Call to '" + modelStateClient.getService() + "' failed."); }
		if (!resp.success)
		{
			throw std::runtime_error("Call to '" + modelStateClient.getService() + "' unable to look up "
			                         + "->" + req.model_name);
		}

        tf::Transform gazeboTlink; tf::poseMsgToTF(resp.pose, gazeboTlink);
		const auto& linkTmarker = pair.second;

		tf::Transform mocapTmarker = gazeboTmocap.inverse() * gazeboTlink * linkTmarker;

		tb->sendTransform(tf::StampedTransform(mocapTmarker, ros::Time::now(), mocap_frame_id, "mocap_" + pair.first.tfName()));
	}
}

std::vector<std::string> GazeboMocap::getModelNames()
{
    gazebo_msgs::GetWorldPropertiesRequest req;
    gazebo_msgs::GetWorldPropertiesResponse resp;
    bool result = worldStateClient.call(req, resp);

    if (!result) { throw std::runtime_error("Call to '" + worldStateClient.getService() + "' failed."); }
	if (!resp.success)  { throw std::runtime_error("Call to '" + worldStateClient.getService() + "' reported failure: '" + resp.status_message + "'"); }

    return resp.model_names;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gazebo_mocap");
	ros::NodeHandle pnh("~");

	bool publish_base = pnh.param("publish_mocap_base", true);

	unsigned int seed = (unsigned int) time(nullptr);
	srand(seed);

	GazeboMocap mocap;

	std::vector<std::string> models = mocap.getModelNames();

	for (size_t i = 0; i < models.size(); ++i)
	{
		mocap.markerOffsets.insert({{models[i], std::string("link")}, tf::StampedTransform(tf::Transform::getIdentity(), ros::Time(0), mocap.mocap_frame_id, models[i])});
	}

	ros::Rate r(10.0);
	while(ros::ok())
	{
		r.sleep();
		ros::spinOnce();
		mocap.sendTransforms(publish_base);
	}

	return 0;
}

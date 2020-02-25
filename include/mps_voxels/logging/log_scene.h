//
// Created by kunhuang on 2/24/20.
//

#ifndef SRC_LOG_SCENE_H
#define SRC_LOG_SCENE_H

#include "mps_voxels/logging/DataLog.h"
#include "mps_voxels/Scene.h"
#include "mps_voxels/SensorHistorian.h"

namespace mps
{

template <>
void DataLog::log<Scene>(const std::string& channel, const Scene& msg);

template <>
bool DataLog::load<Scene>(const std::string& channel, Scene& msg);

Scene computeSceneFromSensorHistorian(const SensorHistoryBuffer& buffer, const ros::Time& t, const std::string& worldFrame);

}


#endif //SRC_LOG_SCENE_H

//
// Created by arprice on 1/31/19.
//

#ifndef MPS_OBJECTLOGGER_H
#define MPS_OBJECTLOGGER_H

#include "mps_voxels/Scene.h"

class ObjectLogger
{
public:
	static
	bool logObject(const Object* obj, const std::string& location, const std::string& name);
};

#endif // MPS_OBJECTLOGGER_H

//
// Created by arprice on 12/19/18.
//

#ifndef PROJECT_OBJECTINDEX_H
#define PROJECT_OBJECTINDEX_H

namespace mps
{

/// @brief An Object here is the result of the "objectness" segmentation in a single frame
struct ObjectIndex
{
	long id = -1;
	bool operator<(const ObjectIndex& other) const { return id < other.id; }
};

/// @brief Represents the trajectory of a (single-frame) object across many frames of a video
struct BundleIndex
{
	long id = -1;
	bool operator<(const BundleIndex& other) const { return id < other.id; }
};

}

#endif //PROJECT_OBJECTINDEX_H

//
// Created by arprice on 12/19/18.
//

#ifndef MPS_OBJECTINDEX_H
#define MPS_OBJECTINDEX_H

#include <ros/time.h>

#define IMPLEMENT_INDEX_INTERNALS(Type) \
using IDType = int; \
IDType id = -1; \
explicit Type(const IDType init = -1) : id(init) {} \
\
inline \
bool operator==(const Type& other) const \
{ \
    return this->id == other.id; \
} \
\
inline \
bool operator!=(const Type& other) const \
{ \
    return !(*this == other); \
} \
\
inline \
bool operator<(const Type& rhs) const \
{ \
    return this->id < rhs.id; \
} \
\
inline \
bool operator>(const Type& rhs) const \
{ \
    return this->id > rhs.id; \
} \


namespace mps
{

using TimeIndex = ros::Time;

/// @brief If a problem can be decomposed into disjoint e.g. ROIs, index them with this
struct SubproblemIndex
{
	IMPLEMENT_INDEX_INTERNALS(SubproblemIndex)
};

/// @brief An Object here is the result of the "objectness" segmentation in a single frame
struct ObjectIndex
{
	IMPLEMENT_INDEX_INTERNALS(ObjectIndex)
};

/// @brief If a belief is represented as discrete particles, index them with this
struct ParticleIndex
{
	IMPLEMENT_INDEX_INTERNALS(ParticleIndex)
};

/// @brief Similar to time, but without any absolute measurement
struct IterationIndex
{
	IMPLEMENT_INDEX_INTERNALS(IterationIndex)
};

/// @brief Represents the trajectory of a (single-frame) object across many frames of a video
struct BundleIndex
{
	IMPLEMENT_INDEX_INTERNALS(BundleIndex)
};

}

#endif //PROJECT_OBJECTINDEX_H

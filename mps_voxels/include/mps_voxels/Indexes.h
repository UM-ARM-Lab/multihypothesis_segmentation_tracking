//
// Created by arprice on 12/19/18.
//

#ifndef MPS_OBJECTINDEX_H
#define MPS_OBJECTINDEX_H

#include <ros/time.h>

#define IMPLEMENT_INDEX_INTERNALS(Type) \
struct Type \
{\
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
friend \
std::ostream& operator<<(std::ostream& os, const Type& t); \
}; \
\
inline \
std::ostream& operator<<(std::ostream& os, const Type& t) \
{ \
    os << t.id; \
    return os; \
} \



namespace mps
{

using TimeIndex = ros::Time;

/// @brief If a problem can be decomposed into disjoint e.g. ROIs, index them with this
IMPLEMENT_INDEX_INTERNALS(SubproblemIndex)

/// @brief An Object here is the result of the "objectness" segmentation in a single frame
IMPLEMENT_INDEX_INTERNALS(ObjectIndex)

/// @brief If a belief is represented as discrete particles, index them with this
IMPLEMENT_INDEX_INTERNALS(ParticleIndex)

/// @brief Similar to time, but without any absolute measurement
IMPLEMENT_INDEX_INTERNALS(IterationIndex)

/// @brief Represents the trajectory of a (single-frame) object across many frames of a video
IMPLEMENT_INDEX_INTERNALS(BundleIndex)

}

#endif //PROJECT_OBJECTINDEX_H

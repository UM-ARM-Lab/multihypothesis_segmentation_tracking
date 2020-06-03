/*
 * Copyright (c) 2020 Andrew Price
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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

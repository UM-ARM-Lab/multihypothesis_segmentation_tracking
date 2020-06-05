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

// Derived from IGL colormap:
//@misc{libigl,
//      title = {{libigl}: A simple {C++} geometry processing library},
//	    author = {Alec Jacobson and Daniele Panozzo and others},
//	    note = {http://libigl.github.io/libigl/},
//      year = {2017},
//}
//

#ifndef MPS_COLORMAP_H
#define MPS_COLORMAP_H

#include <algorithm>
#include <cmath>

namespace igl
{
extern const double inferno_cm[256][3];

extern const double magma_cm[256][3];

extern const double plasma_cm[256][3];

extern const double viridis_cm[256][3];

extern const double parula_cm[256][3];
}

template <typename T>
inline void colormap(
	const double palette[256][3], const T x_in, T & r, T & g, T & b)
{
	static const unsigned int pal = 256;
	const T zero = 0.0;
	const T one = 1.0;
	T x_in_clamped = static_cast<T>(std::max(zero, std::min(one, x_in)));

	// simple rgb lerp from palette
	unsigned int least = std::floor(x_in_clamped * static_cast<T>(pal - 1));
	unsigned int most = std::ceil(x_in_clamped * static_cast<T>(pal - 1));

	T _r[2] = { static_cast<T>(palette[least][0]), static_cast<T>(palette[most][0]) };
	T _g[2] = { static_cast<T>(palette[least][1]), static_cast<T>(palette[most][1]) };
	T _b[2] = { static_cast<T>(palette[least][2]), static_cast<T>(palette[most][2]) };

	T t = std::max(zero, std::min(one, static_cast<T>(fmod(x_in_clamped * static_cast<T>(pal), one))));

	r = std::max(zero, std::min(one, (one - t) * _r[0] + t * _r[1]));
	g = std::max(zero, std::min(one, (one - t) * _g[0] + t * _g[1]));
	b = std::max(zero, std::min(one, (one - t) * _b[0] + t * _b[1]));
}

#endif // MPS_COLORMAP_H

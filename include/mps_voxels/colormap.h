//
// Created by arprice on 8/9/18.
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

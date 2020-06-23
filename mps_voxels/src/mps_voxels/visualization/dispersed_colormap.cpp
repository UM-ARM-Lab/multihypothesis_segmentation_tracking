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

#include "mps_voxels/visualization/dispersed_colormap.h"

#include <mps_voxels/util/VoronoiTessellator.h>
#include <mps_voxels/util/geometry.h>

namespace mps
{

void hsv2rgb(float h, float s, float v, float& r, float& g, float& b)
{
	assert(0.0f <= s && s <= 1.0f);
	assert(0.0f <= v && v <= 1.0f);

	float c = 0.0, hp = 0.0, hpmod2 = 0.0, x = 0.0;
	float m = 0.0, r1 = 0.0, g1 = 0.0, b1 = 0.0;

	while (h > 360.0f)
	{
		h -= 360.0f;
	}
	while (h < 0.0f)
	{
		h += 360.0f;
	}

	c = v * s;   // chroma
	hp = h / 60;
	hpmod2 = hp - (float) ((int) (hp / 2)) * 2;

	x = c * (1 - fabs(hpmod2 - 1));
	m = v - c;

	if (0 <= hp && hp < 1)
	{
		r1 = c;
		g1 = x;
		b1 = 0;
	}
	else if (1 <= hp && hp < 2)
	{
		r1 = x;
		g1 = c;
		b1 = 0;
	}
	else if (2 <= hp && hp < 3)
	{
		r1 = 0;
		g1 = c;
		b1 = x;
	}
	else if (3 <= hp && hp < 4)
	{
		r1 = 0;
		g1 = x;
		b1 = c;
	}
	else if (4 <= hp && hp < 5)
	{
		r1 = x;
		g1 = 0;
		b1 = c;
	}
	else
	{
		r1 = c;
		g1 = 0;
		b1 = x;
	}

	r = (r1 + m);
	g = (g1 + m);
	b = (b1 + m);
}

std::vector<std_msgs::ColorRGBA> dispersedColormap(const int N)
{
	std::vector<std_msgs::ColorRGBA> result;

//	SupportPolygon poly{{1.0,  1.0},
//	                    {-1.0, 1.0},
//	                    {-1.0, -1.0},
//	                    {1.0,  -1.0}};
//	SupportPolygon poly{{1.0,  0.0},
//	                    {0.0, 1.0},
//	                    {-1.0, 0.0},
//	                    {0.0,  -1.0}};
	SupportPolygon poly{{1.0,            0.0},
	                    {0.70710678118,  0.70710678118},
	                    {0.0,            1.0},
	                    {-0.70710678118, 0.70710678118},
	                    {-1.0,           0.0},
	                    {-0.70710678118, -0.70710678118},
	                    {0.0,            -1.0},
	                    {0.70710678118,  -0.70710678118}};

	if (N <= static_cast<int>(poly.size()))
	{
		for (int i = 0; i < N; ++i)
		{
			std_msgs::ColorRGBA color;
			color.a = 1.0f;
			hsv2rgb(static_cast<float>(i) * 360.0f / static_cast<float>(N), 1.0f, 1.0f, color.r, color.g, color.b);
			result.push_back(color);
		}
	}
	else
	{
		VoronoiTessellator tess(poly);
		SupportPoints pts = tess.tessellate(N);

		for (auto& pt : pts)
		{
			if (pt.norm() > 1.0) { pt.normalize(); }
//			std::cerr << pt.x() << "\t" << pt.y() << ":\t" << 360.0f * std::atan2(pt.y(), pt.x()) / (2.0 * M_PI) << "\t"
//			          << pt.norm() << std::endl;

			std_msgs::ColorRGBA color;
			color.a = 1.0f;
			hsv2rgb(std::atan2(pt.y(), pt.x()) * 180.0 / (M_PI), pt.norm(), 1.0f, color.r, color.g, color.b);
			result.push_back(color);
		}
	}

//	cv::Mat display(1, N, CV_8UC3);
//	for (size_t i = 0; i < result.size(); ++i)
//	{
//		const auto& color = result[i];
//		display.at<cv::Vec3b>(i) = cv::Vec3b(255*color.b, 255*color.g, 255*color.r);
//	}
//	cv::namedWindow("Colormap", cv::WINDOW_NORMAL);
//	cv::imshow("Colormap", display);
//	cv::waitKey(0);

	return result;
}

std_msgs::ColorRGBA randomColorMsg(std::mt19937& rng)
{
	std::uniform_real_distribution<> uni(0.0, std::nextafter(1.0, std::numeric_limits<double>::max()));

	std_msgs::ColorRGBA color;
	color.r = uni(rng);
	color.g = uni(rng);
	color.b = uni(rng);
	color.a = 1.0;
	return color;
}

}

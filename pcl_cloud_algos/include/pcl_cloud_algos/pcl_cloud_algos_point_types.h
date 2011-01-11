/*
 * Copyright (c) 2010, Hozefa Indorewala <indorewala@ias.in.tum.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#ifndef PCL_CLOUD_ALGOS_POINT_TYPES_H_
#define PCL_CLOUD_ALGOS_POINT_TYPES_H_

#include <Eigen3/Core>
#include <bitset>
#include <vector>
#include "pcl/ros/register_point_struct.h"
#include <pcl/point_types.h>

namespace pcl
{
struct PointXYZINormalScanLine;
struct ColorCHLACSignature981;
/** Point structure representing the Global Radius-based Surface Descriptor (GRSD). */
struct ColorCHLACSignature117;
/** \brief A point structure representing the Global Radius-based Surface Descriptor (GRSD). */
struct GRSDSignature21;
}

#include <pcl_cloud_algos/pcl_cloud_algos_point_types.hpp>  // Include struct definitions

// ==============================
// =====POINT_CLOUD_REGISTER=====
// ==============================

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZINormalScanLine,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensities, intensities)
                                   (float, normal[0], normal_x)
                                   (float, normal[1], normal_y)
                                   (float, normal[2], normal_z)
                                   // represents the index of one XYZ triplet
                                   (float, index, index)
                                   // represents one complete 2D scan 
                                   (float, line, line)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::ColorCHLACSignature981,
                                   (float[981], histogram, colorCHLAC)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::ColorCHLACSignature117,
                                   (float[117], histogram, colorCHLAC_RI)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::GRSDSignature21,
                                   (float[21], histogram, grsd)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointNormalRADII,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (float, curvature, curvature)
                                   (float, r_min, r_min)
                                   (float, r_max, r_max)
);
#endif 


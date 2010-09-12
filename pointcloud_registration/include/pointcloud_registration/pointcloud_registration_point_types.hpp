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

namespace pcl
{

  //struct PointXYZINormal
  //{
  //  float x;
  //  float y;
  //  float z;
  //  float intensity;
  //  float normal[3];
  //  float curvature;
  //  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //} EIGEN_ALIGN_128;
  //inline std::ostream& operator<<(std::ostream& os, const PointXYZINormal& p)
  //{
  //  os << "("<<p.x<<","<<p.y<<","<<p.z<<" - "<<p.intensity<<" - "<<p.normal[0]<<","<<p.normal[1]<<","<<p.normal[2]<<" - "<<p.curvature<<")";
  //  return os;
  //}

struct PointXYZIRGBNormal
{
  PCL_ADD_POINT4D;
  float intensity;
  float rgb;
  PCL_ADD_NORMAL4D;
  float curvature;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN_128;
inline std::ostream& operator<<(std::ostream& os, const PointXYZIRGBNormal& p)
{
  os << "("<<p.x<<","<<p.y<<","<<p.z<<" - "<<p.intensity<<" - "<<p.rgb<<" - "<<p.normal[0]<<","<<p.normal[1]<<","<<p.normal[2]<<" - "<<p.curvature<<")";
  return os;
}

struct SpinImageLocal
{
  uint32_t histogram[100];
};
inline std::ostream& operator<<(std::ostream& os, const SpinImageLocal& p)
{
  for (int i=0; i<100; ++i) os<<(i==0?"(":"")<<p.histogram[i]<<(i<99 ? ", " : ")");
  return os;
}

}  // End namespace

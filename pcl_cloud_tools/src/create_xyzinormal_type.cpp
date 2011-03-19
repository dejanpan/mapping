/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/**

\author Dejan Pangercic

@b Takes in pcd with PointXYZRGB type and converts it to PointXYZINormal type.

**/

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/features/normal_3d.h>

int
  main (int argc, char** argv)
{
  //read input cloud
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  if (argc != 2)
    {
      ROS_ERROR("Usage: %s <input_cloud.pcd>", argv[0]);
      exit(2);
    }
  reader.read (argv[1], cloud_xyz);
  ROS_INFO ("PointCloud has: %zu data points.", cloud_xyz.points.size ());


  //construct PointXYZI cloud
  pcl::PointCloud<pcl::PointXYZI> cloud_a;
  cloud_a.points.resize(cloud_xyz.points.size ());
  for (unsigned int i = 0; i < cloud_xyz.points.size(); i++)
    {
      cloud_a.points[i].x = cloud_xyz.points[i].x;
      cloud_a.points[i].y = cloud_xyz.points[i].y;
      cloud_a.points[i].z = cloud_xyz.points[i].z;
      cloud_a.points[i].intensity = 0;
    }


  //estimate normals
  pcl::PointCloud<pcl::Normal> cloud_b;
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZI> ());
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_a.makeShared());
  ne.setKSearch (10);
  ne.compute (cloud_b);


  //concatenate fields
  pcl::PointCloud<pcl::PointXYZINormal> cloud_c;
  pcl::concatenateFields (cloud_a, cloud_b, cloud_c);
  

  //write result to disk as ASCII
  pcl::PCDWriter writer;
  ROS_INFO ("Writing PointCloud with: %zu data points.", cloud_c.points.size ());
  writer.write (std::string(argv[1]) + "_normals.pcd", cloud_c, false);
  return (0);
}

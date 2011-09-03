/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Bosch RTC
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
 * $Id: transform_pointcloud.cpp 30719 2010-07-09 20:28:41Z rusu $
 *
 */
/**

\author Dejan Pangercic

@b concatenates point clouds and saves the result into 1 pcd file.

**/
// ROS core
#include <ros/ros.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"

using namespace std;
typedef pcl::PointXYZ PointT;


int main (int argc, char** argv)
{
  if (argc != 3)
    {
      cerr << "Usage: " << argv[0] << " pointcloud1.pcd pointcloud2.pcd" << endl; 
      exit(0);
    }
  pcl::PointCloud<PointT> pcl_cloud1, pcl_cloud2;
  pcl::PointCloud<PointT> concatenated_pc;
  pcl::PCDReader reader;
  reader.read (argv[1], pcl_cloud1);
  reader.read (argv[2], pcl_cloud2);
  
  concatenated_pc += pcl_cloud1;
  concatenated_pc += pcl_cloud2;
  pcl::PCDWriter pcd_writer;
  std::string cloud_name = "concatenated_cloud.pcd";
  // std::string mesh_name = "mesh.vtk";
  //  cloud += *input_cloud;
  pcd_writer.write(cloud_name, concatenated_pc);
  ROS_INFO("[ConcatenatePointCloudNode:] concatenated cloud with %ld points", concatenated_pc.points.size());
  return (0);
}

/* ]--- */

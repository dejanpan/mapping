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

/*
 * Author: Hozefa Indorewala
 * Simple test file to test spin image estimation of a few points of a point cloud
 */

#include <ros/ros.h>
#include <pointcloud_registration/spin_image_estimation/spin_image_estimation.h>
#include <pointcloud_registration/pointcloud_registration_point_types.h>
#include <pcl/io/pcd_io.h>
#include "pcl/kdtree/kdtree_flann.h" //for the kdtree
#include <pcl/features/normal_3d_omp.h>

#include "fstream" //for writing a ppm image file

#define PCD_FILE "29_July_full_room_no_downsample.pcd"
#define MAX_NN 300
#define RADIUS 0.1
#define SUB_DIVISIONS 10
#define MAX_VALUE_PPM 20

int main()
{
  sensor_msgs::PointCloud2 cloud_blob;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::SpinImageEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::SpinImageLocal> s;
  pcl::PointCloud<pcl::SpinImageLocal> sil;
  std::vector<int> indices;

  int sil_indices_array[] = {277816, 205251, 198723, 221146, 271903};
  std::vector<int> sil_indices (sil_indices_array, sil_indices_array + sizeof(sil_indices_array) / sizeof(int) );
  if (pcl::io::loadPCDFile (PCD_FILE, cloud_blob) == -1)
  {
    ROS_ERROR ("Couldn't read pcd file.");
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from pcd files", (int)(cloud_blob.width * cloud_blob.height));

  // Converting from PointCloud2 msg format to pcl pointcloud format
  pcl::fromROSMsg(cloud_blob, cloud);

  //KdTree stuff
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree;
  tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
  tree->setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloud));

  //indices stuff
  indices.resize(cloud.points.size());
  for(size_t i = 0; i < cloud.points.size(); i++)
  {
    indices[i] = i;
  }
  // Estimate normals first
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> n;
  pcl::PointCloud<pcl::PointNormal> normals;
  // set parameters
  n.setInputCloud (boost::make_shared <const pcl::PointCloud<pcl::PointXYZ> > (cloud));
  n.setIndices (boost::make_shared <std::vector<int> > (indices));
  n.setSearchMethod (tree);
  n.setKSearch (30);    // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (normals);
  ROS_INFO("Normals computed.");

  //Spin Image Estimation
  //s.setSearchSurface(boost::make_shared< pcl::PointCloud<pcl::PointXYZ> > (cloud));
  s.setInputNormals(boost::make_shared< pcl::PointCloud<pcl::PointNormal> > (normals));
  s.setInputCloud(boost::make_shared< pcl::PointCloud<pcl::PointXYZ> > (cloud));
  s.setIndices(boost::make_shared< std::vector<int> > (sil_indices));
  s.setSearchMethod(tree);
  s.setKSearch(MAX_NN);
  s.setNrSubdivisions(SUB_DIVISIONS);
  s.setRadius(RADIUS);
  s.compute(sil);

  ROS_INFO("Spin Image Estimation Complete.");

  //Write the histogram values of a selected point to a PPM file
  std::ofstream ppm_file;

  //std::cout<<sil.points.size()<<std::endl;
  for(size_t idx = 0 ; idx < sil_indices.size(); idx++)
  {
    std::stringstream ss;
    ss <<PCD_FILE<<"_"<<sil_indices[idx]<<".ppm";
    ppm_file.open((ss.str()).c_str());
    ppm_file << "P3"<<std::endl;
    ppm_file << "#Point: "<<sil_indices[idx]<<std::endl;
    ppm_file << SUB_DIVISIONS*10<<" " <<SUB_DIVISIONS*10<<std::endl;
    ppm_file << MAX_VALUE_PPM << std::endl;

    for(int row = 0; row < SUB_DIVISIONS; row ++)
    {
      for(int j = 0; j < 10; j++)
      {
        for(int col = 0; col < SUB_DIVISIONS; col ++)
        {
          int i = row*SUB_DIVISIONS + col;
          if( sil.points[idx].histogram[i] > MAX_VALUE_PPM)
          {
            ROS_WARN("Spin Image feature value larger than user defined max value");
            sil.points[idx].histogram[i] = MAX_VALUE_PPM;
          }
          for(int k =0; k < 10; k++)
          {
              ppm_file<<sil.points[idx].histogram[i] <<" ";
              ppm_file<<sil.points[idx].histogram[i] <<" ";
              ppm_file<<sil.points[idx].histogram[i] <<" ";
              ppm_file<<"    ";
          }
        }

        ppm_file << std::endl;
      }
    }
    ppm_file.close();
  }
  return 0;
}

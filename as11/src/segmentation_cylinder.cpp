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
 * $Id: segmentation_cylinder.cpp 35522 2011-01-26 08:17:01Z rusu $
 *
 */
#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZ PointT;

/* ---[ */
int
  main (int argc, char** argv)
{
  // All the objects needed
  /*****************************************************
   * TODO 1: What do these classes do - document next lines
   * Documentation http://www.ros.org/doc/api/pcl/html/annotated.html
   */
  pcl::PCDReader reader;
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients ()), coefficients_cylinder (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices ()), inliers_cylinder (new pcl::PointIndices ());

  /*****************************************************
   * TODO 2: Read in the input pointcloud: data/table_scene_mug_stereo_textured.pcd
   * Use "reader" object
   */


  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.5);
  pass.filter (*cloud_filtered);
  ROS_INFO ("PointCloud after filtering has: %zu data points.", cloud_filtered->points.size ());


  /*****************************************************
   * TODO 3: Estimate the normals
   * Use "ne" object, methods to be called in order: setSearchMethod, setInputCloud, setKSearch, compute
   */


    /*****************************************************
   * TODO 4: Create the segmentation object for the planar model and set all the parameters
   * Use "seg" object, methods to be called in order: setOptimizeCoefficients, setNormalDistanceWeight, 
   * setMethodType (use SAC_RANSAC), setMaxIterations, setDistanceThreshold, setInputCloud, segment 
   */


  /*****************************************************
   * TODO 5: print out plane coefficients
   */
  std::cerr << "Plane coefficients: " << /*todo*/ << std::endl;


  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);


  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);
  ROS_INFO ("PointCloud representing the planar component: %zu data points.", cloud_plane->points.size ());
  /*****************************************************
   * TODO 6:  Write the planar inliers to disk
   * Use "writer" object
   */


    /*****************************************************
   * TODO 7:  Remove the planar inliers, extract the cluster lying on a table
   * Use "extract" object and use the following functions in order setNegative, filter
   */


  /*****************************************************
   * TODO 8:  Write the cluster to disk
   */

  return (0);
}
/* ]--- */

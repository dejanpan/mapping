/*
 * main.cpp
 *
 *  Created on: 11.07.2012
 *      Author: ross Kidson
 */

#include <stdio.h>
#include <iostream>
#include <ros/ros.h>

//local files
#include "rgbd_registration/typedefs.h"
#include "rgbd_registration/rgb_feature_matcher.h"
#include "rgbd_registration/pcl_utils.h"
#include "rgbd_registration/joint_optimize_wrapper.h"
#include "rgbd_registration/parameter_server.h"

int main (int argc, char** argv)
{
  ros::init (argc, argv, "rgbd_registration");
  if(!ros::master::check())
  {
    ROS_ERROR("roscore not running. stop.");
    exit(0);
  }

  std::string source_filename, target_filename;
  source_filename = ParameterServer::instance()->get<std::string>("source_cloud_filename");
  target_filename = ParameterServer::instance()->get<std::string>("target_cloud_filename");
  ROS_INFO_STREAM("[main] Source pointcloud file: " << source_filename);
  ROS_INFO_STREAM("[main] Target pointclout file: " << target_filename);

  PointCloudPtr source_cloud_ptr (new PointCloud);
  PointCloudPtr target_cloud_ptr (new PointCloud);
  pcl::PCDReader reader;
  reader.read (source_filename, *source_cloud_ptr);
  reader.read (target_filename, *target_cloud_ptr);

  // Extract 2d RGB features and project them into 3d.  Use Ransac to filter out outliers and
  // obtain a transformation between the 2 point clouds
  std::vector<Eigen::Vector4f> source_feature_3d_locations, target_feature_3d_locations;
  Eigen::Matrix4f ransac_trafo, joint_opt_trafo;
  RGBFeatureMatcher point_cloud_matcher (source_cloud_ptr, target_cloud_ptr);
  if (!point_cloud_matcher.getMatches (source_feature_3d_locations, target_feature_3d_locations,
      ransac_trafo))
    exit (0);

  // use the feature points as distinct correspondences in a joint optimization over dense
  // clouds and sparse feature points
  joint_opt_trafo = performJointOptimization (source_cloud_ptr, target_cloud_ptr,
      source_feature_3d_locations, target_feature_3d_locations, ransac_trafo);

  // write the resulting transformed pointcloud to disk
  transformAndWriteToFile (source_cloud_ptr, ransac_trafo);

  return 0;
}

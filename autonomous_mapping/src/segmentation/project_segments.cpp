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





#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointXYZ PointT;

int
  main (int argc, char** argv)
{
  // All the objects needed
  pcl::PCDReader reader;

  pcl::PCDWriter writer;

  // Datasets
  pcl::PointCloud<PointT>::Ptr input_cloud (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr output_cloud (new pcl::PointCloud<PointT> ());

  // Read in the cloud data
  reader.read (std::string(argv[1]), *input_cloud);
  ROS_INFO ("PointCloud has: %zu data points.", input_cloud->points.size ());
  //compute the transform usig btTransform
  std::vector<btTransform> poses_first (100);
  std::vector<btTransform> poses_last (100);
  btTransform bt1,bt2,bt1_inverse,bt2_inverse,bt_relative;
  Eigen::Matrix4f transform_matrix;
  poses_first[0]=btTransform(btQuaternion(0.998846824056,0.0321420649378,-0.0347925244179,-0.00783517578594) ,btVector3(0.794980231638,2.61825838228,0.793744988276));
  poses_last[0]=btTransform(btQuaternion(0.998752133384,0.0207771171007,-0.0400542187211,-0.0214043693847) ,btVector3(0.196217134071,2.58788842192,0.802082517919));
  poses_first[11]=btTransform(btQuaternion(1,0,0,0) ,btVector3(0.854595096684,2.1117762422,0.83581194444));
  poses_last[11]=btTransform(btQuaternion(1,0,0,0) ,btVector3(0.304168821189,2.0881359323,0.835223245908));
  poses_first[12]=btTransform(btQuaternion(1,0,0,0) ,btVector3(0.87515422062,2.1264438802,0.688548777959));
  poses_last[12]=btTransform(btQuaternion(1,0,0,0) ,btVector3(0.226732063269,2.0704584748,0.677043344612));
  poses_first[20]=btTransform(btQuaternion(1,0,0,0) ,btVector3(0.794980231638,1.7,0.783785592596));
  poses_last[20]=btTransform(btQuaternion(1,0,0,0) ,btVector3(0.196217134071,1.67243435547,0.794664770984));
  poses_first[30]=btTransform(btQuaternion(1,0,0,0) ,btVector3(0.708097227301,1.12431635089,0.783785592596));
  poses_last[30]=btTransform(btQuaternion(1,0,0,0) ,btVector3(0.15762123548,1.09807397523,0.783569963996));
  poses_first[31]=btTransform(btQuaternion(1,0,0,0) ,btVector3(0.707196010134,1.15281266082,0.64182772675));
  poses_last[31]=btTransform(btQuaternion(1,0,0,0) ,btVector3(0.108505198317,1.11737966783,0.631973685219));
  poses_first[32]=btTransform(btQuaternion(1,0,0,0) ,btVector3(0.720480791782,1.14081120967,0.349637179347));
  poses_last[32]=btTransform(btQuaternion(1,0,0,0) ,btVector3(0.169155951244,1.09290312037,0.352896361082));
  poses_first[40]=btTransform(btQuaternion(0.69988310609,0.0125240341376,0.0516863610901,0.712274740852) ,btVector3(0.759088815798,0.447555838799,0.788543972139));
  poses_last[40]=btTransform(btQuaternion(0.679154223135,-0.182616350164,0.209965139294,0.679202068704) ,btVector3(0.328294951431,0.455333579402,0.536777352992));
  poses_first[50]=btTransform(btQuaternion(1,0,0,0) ,btVector3(0.811475455459,-0.159275054725,0.64285697828));
  poses_last[50]=btTransform(btQuaternion(1,0,0,0) ,btVector3(0.262167086028,-0.189935857791,0.635715051788));
  poses_first[60]=btTransform(btQuaternion(-0.0231417985797,-0.004722980116,0.0150256782545,0.999608113017) ,btVector3(0.970785923351,-0.51177511333,0.987374139302));
  poses_last[60]=btTransform(btQuaternion(-0.0757567991896,-0.0338534608379,0.714338624073,0.694863425951) ,btVector3(0.481417937933,-1.13998831455,0.975107082045));
  poses_first[61]=btTransform(btQuaternion(1,0,0,0) ,btVector3(0.985176471098,-0.75775319426,0.467585908667));
  poses_last[61]=btTransform(btQuaternion(1,0,0,0) ,btVector3(0.486508365334,-0.797871464651,0.460385745908));
  bt1=poses_first[std::atoi(argv[1])];
  bt2=poses_last[std::atoi(argv[1])];
  bt1_inverse=bt1.inverse();
  bt2_inverse=bt2.inverse();
  bt_relative=bt1_inverse * bt2;
  bt_relative=bt1 * bt2_inverse;
  ROS_INFO("The transform bt1 is %f, %f, %f ",bt1.getOrigin().x(),bt1.getOrigin().y(),bt1.getOrigin().z());
  ROS_INFO("The transform bt2 is%f, %f, %f ",bt2.getOrigin().x(),bt2.getOrigin().y(),bt2.getOrigin().z());
  ROS_INFO("The transform is %f, %f, %f ",bt_relative.getOrigin().x(),bt_relative.getOrigin().y(),bt_relative.getOrigin().z());
  ROS_INFO("The orientation is %f, %f, %f, %f",bt_relative.getRotation().x(),bt_relative.getRotation().y(),bt_relative.getRotation().z(),bt_relative.getRotation().w());
  //tf::StampedTransform c2h_transform (bt_relative);
  pcl_ros::transformAsMatrix (bt_relative, transform_matrix);
  pcl::transformPointCloud(*input_cloud, *output_cloud,transform_matrix);
  // Write the planar inliers to disk
  std::string s="data/transformed_" + std::string(argv[1]);
  ROS_INFO("String is: %s ",s.c_str());
  ROS_INFO ("PointCloud representing the planar component: %zu data points.", output_cloud->points.size ());
  writer.write (s.c_str(), *output_cloud, false);
  return (0);
}
/* ]--- */

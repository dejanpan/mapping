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

class ConcatenatePointCloudNode
{
protected:
  ros::NodeHandle nh_;
public:
  string input_cloud_topic_;
  pcl::PointCloud<pcl::PointXYZ> concatenated_pc_, concatenated_pc_filtered_;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr concatenated_pc_ptr_;
  ros::Subscriber sub_;
  pcl::PCDWriter pcd_writer_;
  double voxel_size_;
  pcl::VoxelGrid<pcl::PointXYZ> vgrid_;

  ////////////////////////////////////////////////////////////////////////////////
  ConcatenatePointCloudNode  (ros::NodeHandle &n) : nh_(n)
  {
    // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
    nh_.param("input_cloud_topic", input_cloud_topic_, std::string("cloud_pcd"));
    nh_.param("voxel_size", voxel_size_, 0.02);
    sub_ = nh_.subscribe (input_cloud_topic_, 1,  &ConcatenatePointCloudNode::cloud_cb, this);
    ROS_INFO ("[ConcatenatePointCloudNode:] Listening for incoming data on topic %s", nh_.resolveName (input_cloud_topic_).c_str ());
    vgrid_.setLeafSize (voxel_size_, voxel_size_, voxel_size_);
    concatenated_pc_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ> ());
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  ~ConcatenatePointCloudNode ()
    {
      concatenated_pc_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ> (concatenated_pc_));
      vgrid_.setInputCloud (concatenated_pc_ptr_);
//      vgrid_.setFilterFieldName("x");
      //    vgrid_.setFilterLimits(0.0, 3.0);
      vgrid_.setFilterFieldName("y");
      vgrid_.setFilterLimits(-3.0, 1.5);
      vgrid_.filter(concatenated_pc_filtered_);
      ROS_INFO("[ConcatenatePointCloudNode:] Writting cloud with %ld points", concatenated_pc_filtered_.points.size());
      pcd_writer_.write ("concatenated_cloud.pcd", concatenated_pc_filtered_, true);
    }
  
  ////////////////////////////////////////////////////////////////////////////////
  // cloud_cb (!)
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pc)
  {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*pc, pcl_cloud);
    concatenated_pc_.header = pcl_cloud.header;
    concatenated_pc_ += pcl_cloud;
    ROS_INFO("[ConcatenatePointCloudNode:] concatenated cloud with %ld points", concatenated_pc_.points.size());
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "concatenate_pointcloud_node");
  ros::NodeHandle n("~");
  ConcatenatePointCloudNode cp(n);
  ros::spin ();
  return (0);
}

/* ]--- */

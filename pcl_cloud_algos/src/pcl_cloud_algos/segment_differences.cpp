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
 * $Id: segment_difference.cpp 30719 2010-07-09 20:28:41Z rusu $
 *
 */

/**

\author Dejan Pangercic

@b segmentat_difference exemplifies how to find a difference between 2 point clouds.

**/

// ROS core
#include <ros/ros.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/segmentation/segment_differences.h>

#include "pcl_ros/publisher.h"

using namespace std;

class SegmentDifferencesNode
{
protected:
  ros::NodeHandle nh_;
  
public:
  string output_cloud_topic_;
  string output_cloud1_topic_, output_cloud2_topic_;
  
  pcl_ros::Publisher<pcl::PointXYZ> pub_diff;
  pcl_ros::Publisher<pcl::PointXYZ> pub_cloud1;
  pcl_ros::Publisher<pcl::PointXYZ> pub_cloud2;
  
  double rate_;
  ////////////////////////////////////////////////////////////////////////////////
  SegmentDifferencesNode  (ros::NodeHandle &n) : nh_(n)
  {
    // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
    output_cloud_topic_ = "difference";
    output_cloud1_topic_ = "cloud1";
    output_cloud2_topic_ = "cloud2";
    pub_diff.advertise (nh_, output_cloud_topic_.c_str (), 1);
    pub_cloud1.advertise (nh_, output_cloud1_topic_.c_str (), 1);
    pub_cloud2.advertise (nh_, output_cloud2_topic_.c_str (), 1);
    ROS_INFO ("Publishing data on topic %s.", nh_.resolveName (output_cloud_topic_).c_str ());
    rate_ = 1;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Spin (!)
  bool spin ()
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud1;
    pcl::PointCloud<pcl::PointXYZ> output;
    
    // Fill in the cloud data
    cloud.width = 15;
    cloud1.width  = 15;
    cloud.height = 1;
    cloud1.height  = 1;
    cloud.points.resize (cloud.width * cloud.height);
    cloud1.points.resize (cloud1.width * cloud1.height);
    cloud.header.stamp = cloud1.header.stamp = ros::Time::now();
    cloud.header.frame_id = cloud1.header.frame_id = "base_link";
    
    // Generate the data
    for (size_t i = 0; i < cloud.points.size (); i++)
    {
      cloud.points[i].x = cloud1.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      cloud.points[i].y = cloud1.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      cloud.points[i].z = 1.0;
      cloud1.points[i].z = 1.0;
    }
    // Set a few outliers
    cloud.points[0].z = 2.0;
    cloud.points[3].z = -2.0;
    cloud.points[6].z = 4.0;
    
    
    // Create the segmentation object
    pcl::SegmentDifferences <pcl::PointXYZ> seg;
    seg.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
    seg.setDistanceThreshold (0.00005);
    seg.setTargetCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud1));
    seg.segment (output);
    double interval = rate_ * 1e+6;
     
    while (nh_.ok ())
    {
      pub_diff.publish (output);
      pub_cloud1.publish (cloud);
      pub_cloud2.publish (cloud1);
      usleep (interval);
      ros::spinOnce ();
    }
    return (true);
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "segment_difference_node");
  ros::NodeHandle n("~");
  SegmentDifferencesNode sd(n);
  sd.spin ();
  return (0);
}

/* ]--- */

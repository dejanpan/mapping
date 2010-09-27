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
#include "pcl_ros/subscriber.h"

using namespace std;

class SegmentDifferencesNode
{
protected:
  ros::NodeHandle nh_;
  
public:
  string output_cloud_topic_, input_cloud_topic_;
  string output_cloud1_topic_, output_cloud2_topic_;
  
  pcl_ros::Publisher<pcl::PointXYZ> pub_diff;
  pcl_ros::Subscriber<sensor_msgs::PointCloud2> sub_;
  // Create the segmentation object
  pcl::SegmentDifferences <pcl::PointXYZ> seg_;
  double rate_;
  int counter_;
  double distance_threshold_;
  ////////////////////////////////////////////////////////////////////////////////
  SegmentDifferencesNode  (ros::NodeHandle &n) : nh_(n)
  {
    // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
    nh_.param("input_cloud_topic", input_cloud_topic_, std::string("/cloud_pcd"));
    nh_.param("output_cloud_topic", output_cloud_topic_, std::string("difference"));
    nh_.param("distance_threshold", distance_threshold_, 0.0005);
    ROS_INFO ("Distance threshold set to %lf.", distance_threshold_);
    pub_diff.advertise (nh_, output_cloud_topic_.c_str (), 1);
    ROS_INFO ("Publishing data on topic %s.", nh_.resolveName (output_cloud_topic_).c_str ());
    sub_.subscribe (nh_, input_cloud_topic_, 1,  boost::bind (&SegmentDifferencesNode::cloud_cb, this, _1));
    ROS_INFO ("Listening for incoming data on topic %s", nh_.resolveName (input_cloud_topic_).c_str ());
    seg_.setDistanceThreshold (distance_threshold_);
    rate_ = 1;
    counter_ = 0;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // cloud_cb (!)
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pc)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::PointCloud<pcl::PointXYZ> output;
    pcl::fromROSMsg(*pc, cloud_in);
    
    if (counter_ == 0)
    {
      ROS_INFO("Setting input cloud with %ld points", cloud_in.points.size());
      seg_.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_in));
      counter_++;
    }
    else
    {
      ROS_INFO("Setting target cloud with %ld points", cloud_in.points.size());
      seg_.setTargetCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_in));
      seg_.segment (output);
      counter_ = 0;
      ROS_INFO("Publishing difference cloud with %ld points", output.points.size());
      pub_diff.publish (output);
    }
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "segment_difference_node");
  ros::NodeHandle n("~");
  SegmentDifferencesNode sd(n);
  ros::spin ();
  return (0);
}

/* ]--- */

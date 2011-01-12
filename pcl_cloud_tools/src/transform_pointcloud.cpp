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
 * $Id: transform_pointcloud.cpp 30719 2010-07-09 20:28:41Z rusu $
 *
 */

/**

\author Dejan Pangercic

@b transform_pointcloud transforms cloud from frame1 to frame2.

**/

// ROS core
#include <ros/ros.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl_ros/transforms.h"

//#include "pcl_ros/publisher.h"
//#include "pcl_ros/subscriber.h"

#include <tf/transform_listener.h>

using namespace std;

class TransformPointcloudNode
{
protected:
  ros::NodeHandle nh_;

public:
  string output_cloud_topic_, input_cloud_topic_, to_frame_;

  ros::Subscriber sub_;
  ros::Publisher pub_;
  tf::TransformListener tf_;

  sensor_msgs::PointCloud2 output_cloud_;
  ////////////////////////////////////////////////////////////////////////////////
  TransformPointcloudNode  (ros::NodeHandle &n) : nh_(n)
  {
    // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
    nh_.param("input_cloud_topic", input_cloud_topic_, std::string("cloud_pcd"));
    output_cloud_topic_ = input_cloud_topic_ + "_transformed";
    nh_.param("to_frame", to_frame_, std::string("base_link"));

    sub_ = nh_.subscribe (input_cloud_topic_, 1,  &TransformPointcloudNode::cloud_cb, this);
    ROS_INFO ("[TransformPointcloudNode:] Listening for incoming data on topic %s", nh_.resolveName (input_cloud_topic_).c_str ());
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 1);
    ROS_INFO ("[TransformPointcloudNode:] Will be publishing data on topic %s.", nh_.resolveName (output_cloud_topic_).c_str ());
  }

  ////////////////////////////////////////////////////////////////////////////////
  // cloud_cb (!)
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pc)
  {
    bool found_transform = tf_.waitForTransform(pc->header.frame_id, to_frame_,
                                                pc->header.stamp, ros::Duration(10.0));
    if (found_transform)
    {
      //ROS_ASSERT_MSG(found_transform, "Could not transform to camera frame");
      tf::StampedTransform transform;
      tf_.lookupTransform(to_frame_,pc->header.frame_id, pc->header.stamp, transform);
      pcl_ros::transformPointCloud(to_frame_, transform, *pc, output_cloud_);
      ROS_DEBUG("[TransformPointcloudNode:] Point cloud published in frame %s", output_cloud_.header.frame_id.c_str());
      pub_.publish (output_cloud_);
    }
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "transform_pointcloud_node");
  ros::NodeHandle n("~");
  TransformPointcloudNode tp(n);
  ros::spin ();
  return (0);
}

/* ]--- */

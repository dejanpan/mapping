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
#include "pcl/common/common.h"

#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

using namespace std;

class HullContractNode
{
protected:
  ros::NodeHandle nh_;

public:
  string output_cloud_topic_, input_cloud_topic_;

  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher vis_pub_;

  sensor_msgs::PointCloud2 output_cloud_;
  pcl::PointCloud<pcl::PointXYZ> cloud_in_;
  pcl::PointXYZ point_min_;
  pcl::PointXYZ point_max_;
  pcl::PointXYZ point_center_;
  visualization_msgs::Marker marker_;

  double padding_, offset_x_;
  ////////////////////////////////////////////////////////////////////////////////
  HullContractNode  (ros::NodeHandle &n) : nh_(n)
  {
    // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
    nh_.param("input_cloud_topic", input_cloud_topic_, std::string("cloud_pcd"));
    nh_.param("padding", padding_, 0.8);
    //how much to offset the contracted hull into x direction
    //this hack is needed to get the objects on the front edge of the shelf
    nh_.param("offset_x", offset_x_, 0.02);
    output_cloud_topic_ = input_cloud_topic_ + "_padded";
    sub_ = nh_.subscribe (input_cloud_topic_, 1,  &HullContractNode::cloud_cb, this);
    ROS_INFO ("[HullContractNode:] Listening for incoming data on topic %s", nh_.resolveName (input_cloud_topic_).c_str ());
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 1);
    ROS_INFO ("[HullContractNode:] Will be publishing data on topic %s.", nh_.resolveName (output_cloud_topic_).c_str ());
    vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  }

  ////////////////////////////////////////////////////////////////////////////////
  // cloud_cb (!)
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pc)
  {
    pcl::fromROSMsg(*pc, cloud_in_);
    //    cloud_in_.width = cloud_in_.width + 2;
    getMinMax3D (cloud_in_, point_min_, point_max_);
    //Calculate the centroid of the hull
    point_center_.x = (point_max_.x + point_min_.x)/2;
    point_center_.y = (point_max_.y + point_min_.y)/2;
    point_center_.z = (point_max_.z + point_min_.z)/2;
    // float min_x, max_y, min_y, z;
    // min_x = min_y = FLT_MAX;
    // max_y = FLT_MIN;

    for (unsigned long i = 0; i < cloud_in_.points.size(); i++)
    {
      //hack to preserve the closest edge of the plane
      // if (cloud_in_.points[i].y < min_y && cloud_in_.points[i].x < min_x)
      // 	{
      // 	  min_y = cloud_in_.points[i].y;
      // 	  min_x = cloud_in_.points[i].x;
      // 	  z = cloud_in_.points[i].z;
      // 	}
      // if (cloud_in_.points[i].y > max_y &&  cloud_in_.points[i].x < min_x)
      // 	{
      // 	  max_y = cloud_in_.points[i].y;
      // 	  min_x = cloud_in_.points[i].x;
      // 	  z = cloud_in_.points[i].z;
      // 	}
      double dist_to_center = sqrt((point_center_.x - cloud_in_.points[i].x) * (point_center_.x - cloud_in_.points[i].x) +
                                   (point_center_.y - cloud_in_.points[i].y) * (point_center_.y - cloud_in_.points[i].y));
      ROS_DEBUG("[HullContractNode:] Dist to center: %lf", dist_to_center);
      double angle;
      angle= atan2((cloud_in_.points[i].y - point_center_.y), (cloud_in_.points[i].x - point_center_.x));
      double new_dist_to_center = padding_ * dist_to_center;
      cloud_in_.points[i].y = point_center_.y + sin(angle) * new_dist_to_center;
      cloud_in_.points[i].x = point_center_.x + cos(angle) * new_dist_to_center;
      cloud_in_.points[i].x = cloud_in_.points[i].x - offset_x_;
    }
    //  //hack to preserve the closest edge of the plane
    // pcl::PointXYZ minx_miny, minx_maxy;
    // minx_miny.x = min_x;
    // minx_miny.y = min_y;
    // minx_miny.z = z;
    // minx_maxy.x = min_x;
    // minx_maxy.y = max_y;
    // minx_maxy.z = z;
    // cloud_in_.points.push_back(minx_maxy);
    // cloud_in_.points.push_back(minx_miny);
    //end of hack
    pcl::toROSMsg (cloud_in_, output_cloud_);
    ROS_INFO("[HullContractNode:] Published contracted hull to topic %s", output_cloud_topic_.c_str());
    pub_.publish (output_cloud_);

    marker_.header.frame_id = output_cloud_.header.frame_id;
    marker_.header.stamp = ros::Time();
    marker_.ns = "my_namespace";
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::SPHERE;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.position.x = point_center_.x;
    marker_.pose.position.y = point_center_.y;
    marker_.pose.position.z = point_center_.z;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = 0.1;
    marker_.scale.y = 0.1;
    marker_.scale.z = 0.1;
    marker_.color.a = 1.0;
    marker_.color.r = 0.0;
    marker_.color.g = 1.0;
    marker_.color.b = 0.0;
    vis_pub_.publish( marker_ );
  }

  void publish_center_radius()
  {

  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "transform_pointcloud_node");
  ros::NodeHandle n("~");
  HullContractNode tp(n);
  ros::spin ();
  return (0);
}

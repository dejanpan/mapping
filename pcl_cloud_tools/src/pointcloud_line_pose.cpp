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

@b pointcloud_minmax_3d computes the MinMax3D point of the pointcloud
and publishes them along with the cloud's centroid.

**/

// ROS core
#include <ros/ros.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/common/common.h"
#include "pcl/ModelCoefficients.h"

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

class PointcloudLinePose
{
protected:
  ros::NodeHandle nh_;

public:
  string output_pose_topic_, input_model_topic_;

  ros::Subscriber sub_;
  ros::Publisher pub_;

  pcl::ModelCoefficients model_in_;
  //handle center pose
  geometry_msgs::PoseStamped handle_pose_;

  //sleep one second between publishing of handle poses to
  bool sleep_;
  ////////////////////////////////////////////////////////////////////////////////
  PointcloudLinePose  (ros::NodeHandle &n) : nh_(n)
  {
    // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
    nh_.param("input_model_topic", input_model_topic_, std::string("model"));
    nh_.param("output_pose_topic", output_pose_topic_, std::string("handle_pose"));
    nh_.param("sleep", sleep_, false);
    //5 cm between cluster
    sub_ = nh_.subscribe (input_model_topic_, 10,  &PointcloudLinePose::model_cb, this);
    ROS_INFO ("[PointcloudLinePose:] Listening for incoming data on topic %s.", nh_.resolveName (input_model_topic_).c_str ());
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>(output_pose_topic_, 10);
    ROS_INFO ("[PointcloudLinePose:] Will be publishing data on topic %s.", nh_.resolveName (output_pose_topic_).c_str ());
  }

  ////////////////////////////////////////////////////////////////////////////////
  // cloud_cb (!)
  void model_cb (const pcl::ModelCoefficientsConstPtr& model)
  {
    handle_pose_.header = model->header;
    handle_pose_.pose.position.x = model->values[0];
    handle_pose_.pose.position.y = model->values[1];
    handle_pose_.pose.position.z = model->values[2];

    //axis-angle rotation
    btVector3 axis(model->values[3], model->values[4], model->values[5]);
    btVector3 marker_axis(1, 0, 0);
    btQuaternion qt(marker_axis.cross(axis.normalize()), marker_axis.angle(axis.normalize()));
    geometry_msgs::Quaternion quat_msg;
    tf::quaternionTFToMsg(qt, quat_msg);
    handle_pose_.pose.orientation = quat_msg;
    ROS_INFO("[PointcloudLinePose:] Published cluster to topic %s", output_pose_topic_.c_str());
    if (sleep_)
      sleep(1.0);
    pub_.publish (handle_pose_);
  }

//   void compute_marker(visualization_msgs::Marker &marker,  geometry_msgs::Point32 &point)
//   {
//     marker.header.frame_id = output_cluster_.header.frame_id;
//     marker.header.stamp =  output_cluster_.header.stamp;
//     marker.ns = "object_cluster";
//     marker.id = 0;
//     marker.type = visualization_msgs::Marker::SPHERE;
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.pose.position.x = point.x;
//     marker.pose.position.y = point.y;
//     marker.pose.position.z = point.z;
//     marker.pose.orientation.x = 0.0;
//     marker.pose.orientation.y = 0.0;
//     marker.pose.orientation.z = 0.0;
//     marker.pose.orientation.w = 1.0;
//     marker.scale.x = 0.05;
//     marker.scale.y = 0.05;
//     marker.scale.z = 0.05;
//     marker.color.a = 1.0;
//     marker.color.r = 0.0;
//     marker.color.g = 1.0;
//     marker.color.b = 0.0;

//   }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "pointcloud_line_pose");
  ros::NodeHandle n("~");
  PointcloudLinePose lp(n);
  ros::spin ();
  return (0);
}

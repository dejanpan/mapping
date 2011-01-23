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
 * $Id: object_releaser.cpp 30719 2010-07-09 20:28:41Z pangercic $
 *
 */

/**
object_releaser:
a)waits for the stop (release object) gesture
b)calculates world to object_frame transform
c)publishes object continously in world frame
\author Dejan Pangercic

@b 

**/

// ROS core
#include <ros/ros.h>
#include <pcl/common/angles.h>

#include <tf/transform_listener.h>
#include <kinect_cleanup/GetReleasedObject.h>
#include <kinect_cleanup/ReleaseObject.h>

//for transformPointCloud
#include <pcl_ros/transforms.h>

using namespace std;

class ObjectReleaser
{
protected:
  ros::NodeHandle nh_;
  
public:
  tf::TransformListener tf_;
  std::string world_, output_cloud_topic_, object_frame_;
  double tf_buffer_time_;
  bool got_object_;
  sensor_msgs::PointCloud2 output_cloud_;
  ros::Publisher pub_;
  ros::ServiceClient client_get_released_object;
  ros::ServiceServer service;
  ////////////////////////////////////////////////////////////////////////////////
  ObjectReleaser  (ros::NodeHandle &n) : nh_(n)
  {
    nh_.param("world", world_, std::string("openni_depth"));
    nh_.param("object_frame", object_frame_, std::string("object_frame"));
    nh_.param("output_cloud_topic_", output_cloud_topic_, std::string("/moved_object"));
    nh_.param("tf_buffer_time", tf_buffer_time_, 10.0);
    service = nh_.advertiseService("/release_object", &ObjectReleaser::calculate_transform, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 1);
    client_get_released_object = nh_.serviceClient<kinect_cleanup::GetReleasedObject>("/get_released_object");
    got_object_ = false;
  }


  ////////////////////////////////////////////////////////////////////////////////
  // get_object() - get object from object_grabber.cpp
  void get_object(sensor_msgs::PointCloud2 &cloud)
    {
      kinect_cleanup::GetReleasedObject srv_rel_obj;
      if (client_get_released_object.call (srv_rel_obj))
      {
        cloud = srv_rel_obj.response.object;
        got_object_ = true;
        ROS_INFO ("Service call /get_released_object successful");
      }
      else
      {
        ROS_ERROR ("Failed to call service /get_released_object");
      }
    }
  

  ////////////////////////////////////////////////////////////////////////////////
  // calculate_transform ()
  bool calculate_transform(kinect_cleanup::ReleaseObject::Request &req, 
                           kinect_cleanup::ReleaseObject::Response &res)
    {
      ros::Time time = ros::Time::now();
      bool found_transform = tf_.waitForTransform(world_, object_frame_,
                                                  time, ros::Duration(tf_buffer_time_));
      if (found_transform)
      {
        tf::StampedTransform transform;
        //TODO: check if this indeed makes sense
        tf_.lookupTransform(world_, object_frame_, time, transform);
        sensor_msgs::PointCloud2 input_cloud;
        get_object(input_cloud);
        pcl_ros::transformPointCloud(world_, transform, input_cloud, output_cloud_);
        //pub_.publish (output_cloud_);
      }
      res.error = "OK";
      return true; 
    }

  // ////////////////////////////////////////////////////////////////////////////////
  // // spin (!)
   void spin ()
    {
      ros::Rate loop_rate(20);
      while (ros::ok())
      {
        if (got_object_)
        {
          output_cloud_.header.stamp = ros::Time::now();
          pub_.publish (output_cloud_);
          ROS_INFO("[ObjectReleaser:] Point cloud published in frame %s", output_cloud_.header.frame_id.c_str());
        }
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "object_releaser_node");
  ros::NodeHandle n("~");
  ObjectReleaser object_releaser(n);
  //ros::spin ();
  object_releaser.spin();
  return (0);
}

/* ]--- */

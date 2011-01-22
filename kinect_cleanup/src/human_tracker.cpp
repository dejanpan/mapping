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
 * $Id: human_tracker.cpp 30719 2010-07-09 20:28:41Z rusu $
 *
 */

/**
human tracker tracks human using openni_tracker and detects
stop (right hand streched-out) and start (right hand in L) gestures.
\author Dejan Pangercic

@b 

**/

// ROS core
#include <ros/ros.h>
#include <pcl/common/angles.h>

#include <tf/transform_listener.h>
#include <kinect_cleanup/GrabObject.h>
#include <kinect_cleanup/ReleaseObject.h>

using namespace std;

class HumanTracker
{
protected:
  ros::NodeHandle nh_;
  
public:
  tf::TransformListener tf_;
  std::string neck_frame_, right_elbow_frame_, right_hand_frame_, left_elbow_frame_, 
    left_hand_frame_, world_;
  double tf_buffer_time_;
  bool start_, stop_;
  ros::ServiceClient client_grab, client_release;
  ////////////////////////////////////////////////////////////////////////////////
  HumanTracker  (ros::NodeHandle &n) : nh_(n)
  {
    nh_.param("world", world_, std::string("openni_depth"));
    nh_.param("neck_frame", neck_frame_, std::string("neck"));
    nh_.param("right_elbow_frame", right_elbow_frame_, std::string("right_elbow"));
    nh_.param("right_hand_frame", right_hand_frame_, std::string("right_hand"));
    nh_.param("left_elbow_frame", left_elbow_frame_, std::string("left_elbow"));
    nh_.param("left_hand_frame", left_hand_frame_, std::string("left_hand"));

    nh_.param("tf_buffer_time", tf_buffer_time_, 10.0);
    start_ = stop_ = false;
    client_grab = nh_.serviceClient<kinect_cleanup::GrabObject>("/grab_object");
    client_release = nh_.serviceClient<kinect_cleanup::ReleaseObject>("/release_object");
  }

  ////////////////////////////////////////////////////////////////////////////////
  // spin (!)
  void spin ()
  {
    ros::Rate loop_rate(20);
    while (ros::ok())
    {
      //find if we have start or stop gesture
      ros::Time time = ros::Time::now();
      bool found_transform1 = tf_.waitForTransform(neck_frame_, right_elbow_frame_,
                                                   time, ros::Duration(tf_buffer_time_));
      bool found_transform2 = tf_.waitForTransform(neck_frame_, right_hand_frame_,
                                                   time, ros::Duration(tf_buffer_time_));
      if (found_transform1 && found_transform2)
      {
        tf::StampedTransform transform1, transform2;
        tf_.lookupTransform(neck_frame_, right_elbow_frame_, time, transform1);
        tf_.lookupTransform(neck_frame_, right_hand_frame_, time, transform2);                
        
        //stop gesture
        if (transform2.getOrigin().x() > 1.5 * transform1.getOrigin().x())
        {
          ROS_INFO("Stop");
          stop_ = true;
          kinect_cleanup::ReleaseObject srv_release;
          srv_release.request.release = stop_;
          if (client_release.call (srv_release))
          {
            ROS_INFO ("Service call /release_object successful");
          }
          else
          {
            ROS_ERROR ("Failed to call service /release_object");
          }
        }

        //start gesture (triggers computation of hand end-effector point and pointing direction)
        else if (0.9 * transform1.getOrigin().x() < transform2.getOrigin().x() && 
                 transform2.getOrigin().x() < 1.1 * transform1.getOrigin().x() &&
                 transform1.getOrigin().y() < transform2.getOrigin().y())
        {
          ROS_INFO("Start");
          start_ = true;
        }
        else
        {
          //          ROS_INFO("No gesture");
        }
      }
      
      //computation of hand end-effector point and pointing direction
      if (start_)
      {
        //sleep(17);
        time = ros::Time::now();
        bool found_transform3 = tf_.waitForTransform(world_, left_elbow_frame_,
                                                     time, ros::Duration(tf_buffer_time_));

        bool found_transform4 = tf_.waitForTransform(world_, left_hand_frame_,
                                                     time, ros::Duration(tf_buffer_time_));

        if (found_transform3 && found_transform4)
        {
          tf::StampedTransform transform3, transform4;
          tf_.lookupTransform(world_, left_elbow_frame_, time, transform3);
          tf_.lookupTransform(world_, left_hand_frame_, time, transform4);
          btVector3 direction = transform4.getOrigin() - transform3.getOrigin();
          std::vector <float> point_axis (6, 0.0);
          point_axis[0] = transform4.getOrigin().x();
          point_axis[1] = transform4.getOrigin().y();
          point_axis[2] = transform4.getOrigin().z();
          point_axis[3] = direction.normalize().x();
          point_axis[4] = direction.normalize().y();
          point_axis[5] = direction.normalize().z();
          
          //service call
          kinect_cleanup::GrabObject srv_grab;
          ROS_INFO("point and direction:");
          for (unsigned long i=0; i < point_axis.size(); i++)
          {
            srv_grab.request.point_line[i] = point_axis[i];
            ROS_INFO_STREAM(" " << point_axis[i]);
          }
          
          if (client_grab.call (srv_grab))
          {
            ROS_INFO ("Service call /grab_object successful");
          }
          else
          {
            ROS_ERROR ("Failed to call service /grab_object");
          }
          start_ = false;
        } 
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "human_tracker_node");
  ros::NodeHandle n("~");
  HumanTracker ht(n);
  //ros::spin ();
  ht.spin();
  return (0);
}

/* ]--- */

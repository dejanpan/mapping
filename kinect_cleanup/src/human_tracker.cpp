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

using namespace std;

class HumanTracker
{
protected:
  ros::NodeHandle nh_;
  
public:
  tf::TransformListener tf_;
  std::string neck_frame_, elbow_frame_, hand_frame_;
  double tf_buffer_time_;
  ////////////////////////////////////////////////////////////////////////////////
  HumanTracker  (ros::NodeHandle &n) : nh_(n)
  {
    nh_.param("neck_frame", neck_frame_, std::string("neck"));
    nh_.param("elbow_frame", elbow_frame_, std::string("right_elbow"));
    nh_.param("hand_frame", hand_frame_, std::string("right_hand"));
    nh_.param("tf_buffer_time", tf_buffer_time_, 10.0);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // spin (!)
  void spin ()
  {
    ros::Rate loop_rate(20);
    while (ros::ok())
    {
      ros::Time time = ros::Time::now();
      bool found_transform1 = tf_.waitForTransform(neck_frame_, elbow_frame_,
                                                   time, ros::Duration(tf_buffer_time_));
      bool found_transform2 = tf_.waitForTransform(neck_frame_, hand_frame_,
                                                   time, ros::Duration(tf_buffer_time_));

      if (found_transform1 && found_transform2)
      {
        tf::StampedTransform transform1, transform2;
        tf_.lookupTransform(neck_frame_, elbow_frame_, time, transform1);
        tf_.lookupTransform(neck_frame_, hand_frame_, time, transform2);                
        if (transform2.getOrigin().x() > 1.5 * transform1.getOrigin().x())
        {
          ROS_INFO("Start");
        }
        else if (0.9 * transform1.getOrigin().x() < transform2.getOrigin().x() && 
                 transform2.getOrigin().x() < 1.1 * transform1.getOrigin().x() &&
                 transform1.getOrigin().y() < transform2.getOrigin().y())
        {
          ROS_INFO("Stop");
        }
        else
        {
          ROS_INFO("No gesture");
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

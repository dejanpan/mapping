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
  std::string source_frame_, target_frame_;
  double tf_buffer_time_;
  ////////////////////////////////////////////////////////////////////////////////
  HumanTracker  (ros::NodeHandle &n) : nh_(n)
  {
    nh_.param("source_frame", source_frame_, std::string("neck"));
    nh_.param("target_frame", target_frame_, std::string("right_elbow"));
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
      bool found_transform = tf_.waitForTransform(source_frame_, target_frame_,
                                                   time, ros::Duration(tf_buffer_time_));
      if (found_transform)
      {
        //ROS_ASSERT_MSG(found_transform, "Could not transform to camera frame");
        tf::StampedTransform transform;
        tf_.lookupTransform(source_frame_, target_frame_, time, transform);
        double r, p, y;
        btMatrix3x3 m = transform.getBasis();
        m.getRPY(r, p, y);
        // ROS_INFO("[HumanTracker:] Transform between %s and %s: %f, %f, %f, %f, %f, %f", source_frame_.c_str(), 
        //       target_frame_.c_str(), transform.getOrigin().x(), transform.getOrigin().y(), 
        //       transform.getOrigin().z(), r, p, y);
        if (pcl::deg2rad(80.0) < r && r <  pcl::deg2rad(100.0)
            && pcl::deg2rad(-10.0) < p && p <  pcl::deg2rad(10.0)
            pcl::deg2rad(80.0) < y && y <  pcl::deg2rad(100.0))
        {
          ROS_INFO("Stop");
        }
        else if (pcl::deg2rad(80.0) < r && r <  pcl::deg2rad(100.0)
                 && pcl::deg2rad(-10.0) < p && p <  pcl::deg2rad(10.0)
                 pcl::deg2rad(80.0) < y && y <  pcl::deg2rad(100.0))
        {
          ROS_INFO("Start");
        }
        else
        {
          //nothing
        }
      }
      ros::spinOnce();
      loop_rate.sleep();
      //      ROS_INFO("Sleeping!");
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

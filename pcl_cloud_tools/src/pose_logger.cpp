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

#include <fstream>
#include <iostream>

using namespace std;

class PoseLogger
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
  PoseLogger  (ros::NodeHandle &n) : nh_(n)
  {

  }

  ////////////////////////////////////////////////////////////////////////////////
  // cloud_cb (!)
  void spin (double rate)
  {
    ros::Rate loop_rate(rate);
    std::ofstream a_file ( "poses.txt" );
    a_file<< "time" << " " << "x" << " " << "y" << std::endl;
    // Outputs to example.txt through a_file

    // Close the file stream explicitly

    while (ros::ok())
      {
	ros::Time time;
	time = ros::Time::now();
	bool found_transform = tf_.waitForTransform("map", "/base_link",
						    time, ros::Duration(1.0));
	if (found_transform)
	  {
	    //ROS_ASSERT_MSG(found_transform, "Could not transform to camera frame");
	    tf::StampedTransform transform;
	    tf_.lookupTransform("map", "base_link", time, transform);
	    std::cerr << time << " " << transform.getOrigin().x() << " " << transform.getOrigin().y() << std::endl;
	    a_file<< time << " " << transform.getOrigin().x() << " " << transform.getOrigin().y() << std::endl;
	  }
	ros::spinOnce();
	loop_rate.sleep();
      }
    a_file.close();
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "pose_logger_node");
  if (argc != 2)
    {
      ROS_ERROR("Usage %s <logging rate>", argv[0]);
      exit (0);
    }
  ros::NodeHandle n("~");
  PoseLogger tp(n);
  tp.spin(atof(argv[1]));
  //  ros::spin ();
  return (0);
}

/* ]--- */

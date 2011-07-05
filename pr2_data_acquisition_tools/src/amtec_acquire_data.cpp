/*
 * Copyright (c) 2010, Dejan Pangercic <pangercic@cs.tum.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

//for GetClustersAction
#include <pcl_cloud_tools/GetClustersAction.h>

//for rotating amtec PTU
#include <amtec/SetPosition.h>

boost::shared_ptr<const pcl_cloud_tools::GetClustersResult> getClusters(int ec_goal)
{
  //get the object from Kinect
  typedef actionlib::SimpleActionClient<pcl_cloud_tools::GetClustersAction> GetClustersClient;
  GetClustersClient gcc("/extract_clusters/get_clusters", true);
  while(!gcc.waitForServer())
    {
      ROS_INFO("Waiting for /extract_clusters/get_clusters server to come up");
    }
  
  pcl_cloud_tools::GetClustersGoal goal;
  goal.ec_goal = ec_goal;
  // Fill in goal here
  gcc.sendGoal(goal);
  gcc.waitForResult(ros::Duration(5.0));
  if (gcc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("/extract_clusters/get_clusters SUCCEEDED");
  boost::shared_ptr<const pcl_cloud_tools::GetClustersResult> result = gcc.getResult();
  return result;
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "amtec_acquire_data");
    ros::NodeHandle nh;
    //get the cluster on the table
    boost::shared_ptr<const pcl_cloud_tools::GetClustersResult> result = getClusters(0);
    if (result->clusters.size () == 0)
    {
      ROS_ERROR("No clusters found, returning");
      return -1;
    }
    else
    {
      ROS_INFO("%ld clusters found", result->clusters.size ());
    }

    //rotate amtec unit
    ros::ServiceClient client = nh.serviceClient<amtec::SetPosition>("/amtec/set_position");
    amtec::SetPosition srv;
    srv.request.position_pan = 0.0;
    srv.request.position_pan = atof(argv[1]);
    if (client.call(srv))
    {
      ROS_INFO("Successful call to service /amtec/set_position");
    }
    else
    {
      ROS_ERROR("Failed to call service /amtec/set_position");
      return -1;
    }
    return 0;
}

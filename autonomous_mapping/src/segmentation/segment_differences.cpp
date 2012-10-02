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

   @b segment_difference exemplifies how to find a difference between 2 point clouds.

**/

// ROS core
#include <ros/ros.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/segmentation/segment_differences.h>

#include "pcl_ros/publisher.h"
#include <pcl/filters/radius_outlier_removal.h>
#include "pcl/segmentation/extract_clusters.h"
#include <tf/transform_listener.h>
#include "pcl/common/common.h"
#include "pcl_ros/transforms.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;

using namespace std;

class SegmentDifferencesNode
{
protected:
  ros::NodeHandle nh_;
  
public:
  string output_cloud_topic_, input_cloud_topic_;
  string output_filtered_cloud_topic_;
  
  pcl_ros::Publisher<pcl::PointXYZ> pub_diff_;
  pcl_ros::Publisher<pcl::PointXYZ> pub_filtered_;
  ros::Subscriber sub_;
  // Create the segmentation object
  pcl::SegmentDifferences <pcl::PointXYZ> seg_;
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem_;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_;
  KdTreePtr clusters_tree_;
  double rate_;
  int counter_;
  double distance_threshold_;
  bool segment_, take_first_cloud_, save_segmented_cloud_;
  double object_cluster_tolerance_;
  int object_cluster_min_size_, object_cluster_max_size_;
  tf::TransformListener listener_; 
  ////////////////////////////////////////////////////////////////////////////////
  SegmentDifferencesNode  (ros::NodeHandle &n) : nh_(n)
  {
    // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
    nh_.param("input_cloud_topic", input_cloud_topic_, std::string("/cloud_pcd"));
    nh_.param("output_cloud_topic", output_cloud_topic_, std::string("difference"));
    nh_.param("output_filtered_cloud_topic", output_filtered_cloud_topic_, std::string("difference_filtered"));
    nh_.param("distance_threshold", distance_threshold_, 0.01);
    nh_.param("save_segmented_cloud_", save_segmented_cloud_, true);
    ROS_INFO ("Distance threshold set to %lf.", distance_threshold_);
    pub_diff_.advertise (nh_, output_cloud_topic_.c_str (), 1);
    ROS_INFO ("Publishing data on topic %s.", nh_.resolveName (output_cloud_topic_).c_str ());
    pub_filtered_.advertise (nh_, output_filtered_cloud_topic_.c_str (), 1);
    ROS_INFO ("Publishing data on topic %s.", nh_.resolveName (output_filtered_cloud_topic_).c_str ());
    sub_ = nh_.subscribe (input_cloud_topic_, 1,  &SegmentDifferencesNode::cloud_cb, this);
    ROS_INFO ("Listening for incoming data on topic %s", nh_.resolveName (input_cloud_topic_).c_str ());
    nh_.param("object_cluster_tolerance", object_cluster_tolerance_, 0.1);
    //min 100 points
    nh_.param("object_cluster_min_size", object_cluster_min_size_, 500);
    //set PCL classes
    seg_.setDistanceThreshold (distance_threshold_);
    clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
    clusters_tree_->setEpsilon (1);
    rate_ = 1;
    counter_ = 0;
    segment_ = take_first_cloud_ = false;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // cloud_cb (!)
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pc)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::PointCloud<pcl::PointXYZ> output;
    pcl::PointCloud<pcl::PointXYZ> output_filtered;
    pcl::fromROSMsg(*pc, cloud_in);
    nh_.getParam("/segment_difference_interactive/segment", segment_);
    nh_.getParam("/segment_difference_interactive/take_first_cloud", take_first_cloud_);
    
    if (take_first_cloud_)
    {
      ROS_INFO("Setting target cloud with %ld points", cloud_in.points.size());
      //seg_.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_in));
      seg_.setTargetCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_in));
      counter_++;
      nh_.setParam("/segment_difference_interactive/take_first_cloud", false);
      pub_diff_.publish (cloud_in);
    }
    else if (segment_)
    {
      ROS_INFO("Setting input cloud with %ld points", cloud_in.points.size());
      //seg_.setTargetCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_in));
      seg_.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_in));
      seg_.segment (output);
      //counter_ = 0;
      outrem_.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(output));
      outrem_.setRadiusSearch (0.02);
      outrem_.setMinNeighborsInRadius (10);
      outrem_.filter (output_filtered);
      
      //cluster
      std::vector<pcl::PointIndices> clusters;
      cluster_.setInputCloud (boost::make_shared<PointCloud>(output_filtered));
      cluster_.setClusterTolerance (object_cluster_tolerance_);
      cluster_.setMinClusterSize (object_cluster_min_size_);
      //    cluster_.setMaxClusterSize (object_cluster_max_size_);
      cluster_.setSearchMethod (clusters_tree_);
      cluster_.extract (clusters);
      ROS_INFO("[SegmentDifferencesNode:] Found %ld cluster", clusters.size());

      //get robot's pose
      tf::StampedTransform transform;
      listener_.waitForTransform("map", "base_laser_link", ros::Time(), ros::Duration(10.0));
      try
	{
	  listener_.lookupTransform("map", "base_laser_link", ros::Time(), transform);
	}
      catch (tf::TransformException ex)
	{
	  ROS_ERROR("getTransformIn %s",ex.what());
	}
      
      //get closest cluster
      pcl::PointCloud<pcl::PointXYZ> cloud_object_clustered;
      btVector3 robot_position = transform.getOrigin();
      btVector3 cluster_center;
      pcl::PointXYZ point_min;
      pcl::PointXYZ point_max;
      btScalar dist = 100.00;
      int closest_cluster = 0;
      if (clusters.size() > 0)
	{
	  for (unsigned int i = 0; i < clusters.size(); i++)
	    {
	      pcl::copyPointCloud (output_filtered, clusters[i], cloud_object_clustered);
	      getMinMax3D (cloud_object_clustered, point_min, point_max);
	      //Calculate the centroid of the hull
	      cluster_center.setX((point_max.x + point_min.x)/2);
	      cluster_center.setY((point_max.y + point_min.y)/2);
	      cluster_center.setZ((point_max.z + point_min.z)/2);
	      if (dist > fabs(robot_position.distance(cluster_center)))
		{
		  closest_cluster = i;
		  dist = fabs(robot_position.distance(cluster_center));
		}
	      ROS_INFO("Robot's distance to cluster %d is %fm", i, fabs(robot_position.distance(cluster_center)));
	      cloud_object_clustered.points.clear();
	    }
	  ROS_INFO("Closest cluster: %d", closest_cluster);
	  pcl::copyPointCloud (output_filtered, clusters[closest_cluster], cloud_object_clustered);
	  ROS_INFO("Publishing difference cloud with %ld points to topic %s", cloud_object_clustered.points.size(), output_filtered_cloud_topic_.c_str());
	  pub_filtered_.publish (cloud_object_clustered);
	  if (save_segmented_cloud_)
	    {
	      pcl::PCDWriter writer;
	      std::stringstream furniture_face;
	      furniture_face << "furniture_face_" << ros::Time::now() << ".pcd";
	      ROS_INFO("Writing to file: %s", furniture_face.str().c_str());

	      bool found_transform = listener_.waitForTransform(cloud_object_clustered.header.frame_id, "map",
							  cloud_object_clustered.header.stamp, ros::Duration(10.0));
	      if (found_transform)
		{
		  pcl::PointCloud<pcl::PointXYZ> output_cloud;
		  //ROS_ASSERT_MSG(found_transform, "Could not transform to camera frame");
		  tf::StampedTransform transform;
		  listener_.lookupTransform("map", cloud_object_clustered.header.frame_id, cloud_object_clustered.header.stamp, transform);
		  Eigen::Matrix4f transform_matrix;                                                                                                      
		  pcl_ros::transformAsMatrix (transform, transform_matrix);    
		  pcl::transformPointCloud(cloud_object_clustered, output_cloud, transform_matrix);
		  writer.write (furniture_face.str(), output_cloud, false);
		}
	    }
	}
      //ROS_INFO("Publishing difference cloud with %ld points to topic %s", output.points.size(), output_filtered_cloud_topic_.c_str());
      //pub_filtered_.publish (output);
      segment_ = false;
      nh_.setParam("/segment_difference_interactive/segment", false);
    }
    else 
      return;
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

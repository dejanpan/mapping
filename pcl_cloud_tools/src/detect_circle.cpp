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
 *
 */

/**

\author Dejan Pangercic

@b detect_circle detects circles in 3D point clouds

**/

// ROS core
#include <ros/ros.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl_ros/transforms.h"

#include <pcl/ModelCoefficients.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef pcl::KdTree<Point>::Ptr KdTreePtr;

class DetectCircle
{
protected:
  ros::NodeHandle nh_;

public:
  string output_cloud_topic_, input_cloud_topic_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  pcl::NormalEstimation<Point, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg;
  KdTreePtr tree_;
  pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_;
  //  pcl::PointCloud<pcl::Normal> cloud_normals_;
  pcl::PointIndices circle_inliers_;
  pcl::ModelCoefficients circle_coeff;
  ////////////////////////////////////////////////////////////////////////////////
  DetectCircle  (ros::NodeHandle &n) : nh_(n)
  {
    // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
    nh_.param("input_cloud_topic", input_cloud_topic_, std::string("/camera/depth/points2_throttle"));
    nh_.param("output_cloud_topic", output_cloud_topic_, std::string("circle"));
   
    sub_ = nh_.subscribe (input_cloud_topic_, 1,  &DetectCircle::cloud_cb, this);
    ROS_INFO ("[DetectCircle:] Listening for incoming data on topic %s", nh_.resolveName (input_cloud_topic_).c_str ());
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 1);
    ROS_INFO ("[DetectCircle:] Will be publishing data on topic %s.", nh_.resolveName (output_cloud_topic_).c_str ());
    tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
    tree_->setEpsilon (1);

    // Estimate point normals                                                                                                               
    ne.setSearchMethod (tree_);
    ne.setKSearch (50);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // cloud_cb (!)
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pc)
  {
    PointCloud cloud_raw;
    pcl::fromROSMsg (*pc, cloud_raw);
    ne.setInputCloud (boost::make_shared<PointCloud> (cloud_raw));
    pcl::PointCloud<pcl::Normal> cloud_normals;
    ne.compute (cloud_normals);
    cloud_normals_.reset (new pcl::PointCloud<pcl::Normal> (cloud_normals));
    // Create the segmentation object for circle segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    //seg.setModelType (pcl::SACMODEL_CIRCLE3D);
    seg.setModelType (pcl::SACMODEL_CIRCLE2D);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0.05, 0.2);
    seg.setInputCloud (boost::make_shared<PointCloud> (cloud_raw));
    seg.setInputNormals (cloud_normals_);
    // Obtain the circle inliers and coefficients
    seg.segment (circle_inliers_, circle_coeff);
    std::cerr << "Circle inliers " << circle_inliers_.indices.size() << std::endl;
    std::cerr << "Circle coefficients: " << circle_coeff << std::endl;
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "detect_circle_node");
  ros::NodeHandle n("~");
  DetectCircle dc(n);
  ros::spin ();
  return (0);
}

/* ]--- */

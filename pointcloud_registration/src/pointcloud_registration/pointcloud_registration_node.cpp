/*
 * Copyright (c) 2010, Hozefa Indorewala <indorewala@ias.in.tum.de>
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

/*
 * Author: Hozefa Indorewala
 * This node subscribes to a PointCloud2 topic, receives point clouds and then registers them and publishes
 * the result on /merged_pointcloud (PointCloud2)
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pointcloud_registration/pointcloud_registration_point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/SVD>

#include "pcl/filters/statistical_outlier_removal.h" // to filter outliers

#include "pcl/filters/voxel_grid.h" //for downsampling the point cloud

#include "pcl/kdtree/kdtree_flann.h" //for the kdtree

#include "pcl/registration/transforms.h" //for the transformation function

#include <pcl/features/normal_3d_omp.h>

#include <pointcloud_registration/icp/icp_correspondences_check.h> //for icp
#include <algorithm> //for the sort and unique functions

#include <ctime>

const float PI = 3.14159265;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool pclSort (pcl::PointNormal i, pcl::PointNormal j)
{
  return (i.x < j.x);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool pclUnique (pcl::PointNormal i, pcl::PointNormal j)
{
  double x_diff = fabs(i.x - j.x);
  double y_diff = fabs(i.y - j.y);
  double z_diff = fabs(i.z - j.z);
  if(x_diff < 0.0001 && y_diff < 0.0001 && z_diff < 0.0001 )
    return 1;
  else
    return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class PointCloudRegistration
{
  public:
    PointCloudRegistration();
    ~PointCloudRegistration();
    void pointcloudRegistrationCallBack(const sensor_msgs::PointCloud2& msg);
    Eigen::Matrix4f getOverlapTransformation();
    void publishPointCloud(pcl::PointCloud<pcl::PointNormal> &pointcloud);
    pcl::PointCloud<pcl::PointNormal> convertFromMsgToPointCloud(const sensor_msgs::PointCloud2& pointcloud_msg);

    void run();

  private:
    ros::NodeHandle nh_;
    std::string merged_pointcloud_topic_, subscribe_pointcloud_topic_, frame_id_;
    int max_number_of_iterations_icp_, max_nn_icp_, max_nn_overlap_;
    double downsample_leafsize_, epsilon_z_, epsilon_curvature_, epsilon_transformation_, radius_icp_, radius_overlap_;
    bool downsample_pointcloud_before_, downsample_pointcloud_after_, filter_outliers_, curvature_check_;
    int scan_index_, counter_;
    time_t start, end;
    Eigen::Matrix4f final_transformation_;
    ros::Subscriber pointcloud_subscriber_;
    ros::Publisher pointcloud_merged_publisher_;

    pcl::IterativeClosestPointCorrespondencesCheck<pcl::PointNormal, pcl::PointNormal> icp_; // for icp
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree_;  // for kdtree
    bool firstCloudReceived_, secondCloudReceived_;
    pcl::PointCloud<pcl::PointNormal> pointcloud2_current_, pointcloud2_merged_, pointcloud2_transformed_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::Matrix4f PointCloudRegistration::getOverlapTransformation()
{
  // In this function we extract the overlapped region of the two points cloud and compute
  // the transformation and return it.

  if(firstCloudReceived_ == false && secondCloudReceived_ == false )
  {
    ROS_ERROR("getOverlapTransformation called before receiving atleast 2 point clouds");
    exit(1);
  }
  else
  {
    //searching for overlapped points in the point cloud
    // Allocate enough space to hold the results
    std::vector<int> nn_indices (max_nn_overlap_);
    std::vector<float> nn_dists (max_nn_overlap_);

    pcl::PointCloud<pcl::PointNormal> overlap_model, overlap_current;
    Eigen::Matrix4f transformation;
    ROS_INFO("[PointCloudRegistration:] finding overlapping points");
    start = time(NULL);
    std::vector<pcl:: PointNormal, Eigen::aligned_allocator<pcl:: PointNormal> >::iterator it;
    for(size_t idx = 0 ; idx < pointcloud2_current_.points.size(); idx++ )
    {
      kdtree_.radiusSearch(pointcloud2_current_, idx, radius_overlap_, nn_indices, nn_dists, max_nn_overlap_);

      if(nn_indices.size() > 0 )
      {
        overlap_current.points.push_back(pointcloud2_current_.points[idx]);
	      for(size_t i = 0 ; i < nn_indices.size(); i++)
        {
          overlap_model.points.push_back (kdtree_.getInputCloud()->points[nn_indices[i]]);
        }
      }
    }
    end = time(NULL);
    ROS_INFO("[PointCloudRegistration:] found overlapping points in %d seconds", (int)(end - start));

    //Getting rid of duplicate points in model
    ROS_INFO("[PointCloudRegistration:] removing duplicate points");
    start = time(NULL);
    std::sort(overlap_model.points.begin(), overlap_model.points.end(), pclSort);
    it = std::unique(overlap_model.points.begin(), overlap_model.points.end(), pclUnique);
    overlap_model.points.resize(it - overlap_model.points.begin());
    end = time(NULL);
    ROS_INFO("[PointCloudRegistration:] removed duplicate points %d seconds", (int)(end - start));

    icp_.setInputTarget(boost::make_shared< pcl::PointCloud < pcl::PointNormal> > (overlap_model));
    icp_.setInputCloud(boost::make_shared< pcl::PointCloud < pcl::PointNormal> > (overlap_current));

    ROS_INFO("[PointCloudRegistration: ] aligning");
    icp_.align(pointcloud2_transformed_);

    ROS_INFO("[PointCloudRegistration:] getting final transformation");
    transformation = icp_.getFinalTransformation();
    return (transformation);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudRegistration::publishPointCloud(pcl::PointCloud<pcl::PointNormal> &pointcloud)
{
  sensor_msgs::PointCloud2 mycloud;
  pcl::toROSMsg(pointcloud, mycloud);

  if( downsample_pointcloud_after_ == true)
  {
    // for downsampling of pointclouds
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor_;
    sensor_msgs::PointCloud2 mycloud_downsampled;

    //Now we will downsample the point cloud
    sor_.setInputCloud (boost::make_shared<sensor_msgs::PointCloud2> (mycloud));
    sor_.setLeafSize (downsample_leafsize_, downsample_leafsize_, downsample_leafsize_);
    sor_.filter (mycloud_downsampled);
    mycloud_downsampled.header.frame_id = frame_id_;
    mycloud_downsampled.header.stamp = ros::Time::now();

    pointcloud_merged_publisher_.publish(mycloud_downsampled);
  }
  else
  {
    mycloud.header.frame_id = frame_id_;
    mycloud.header.stamp = ros::Time::now();

    pointcloud_merged_publisher_.publish(mycloud);
  }
  ROS_INFO("Merged Point cloud published");
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::PointCloud<pcl::PointNormal> PointCloudRegistration::convertFromMsgToPointCloud(const sensor_msgs::PointCloud2& pointcloud_msg)
{
  //incrementing the scan index
  scan_index_++;
  // Declaring some variables required in this function
  sensor_msgs::PointCloud2 pointcloud2_msg = pointcloud_msg;
  pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl_step01, pointcloud_pcl_step02;
  pcl::PointCloud<pcl::PointNormal> pointcloud_pcl_normals;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree_ptr_;
  tree_ptr_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
  std::vector<int> indices;

  // Converting from PointCloud msg format to PointCloud2 msg format

  //sensor_msgs::convertPointCloudToPointCloud2(pointcloud_msg, pointcloud2_msg);

  //This is done because there is a bug in PCL.
  // for(u_int i = 0 ; i < pointcloud2_msg.fields.size(); i++)
  // {
  //   if(pointcloud2_msg.fields[i].name == "intensities")
  //   {
  //     pointcloud2_msg.fields[i].name = "intensity";
  //   }
  // }

  // STEP 01: Check if we should downsample the input cloud or not
  if(downsample_pointcloud_before_ == true)
  {
    ROS_INFO ("PointCloud before downsampling: %d .", pointcloud2_msg.width * pointcloud2_msg.height);
    sensor_msgs::PointCloud2 pointcloud_downsampled_msg;
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor_; // for downsampling of pointclouds

    //Now we will downsample the point cloud
    sor_.setInputCloud (boost::make_shared<sensor_msgs::PointCloud2> (pointcloud2_msg));
    sor_.setLeafSize (downsample_leafsize_, downsample_leafsize_, downsample_leafsize_);
    sor_.filter (pointcloud_downsampled_msg);
    ROS_INFO ("PointCloud after downsampling: %d .", pointcloud_downsampled_msg.width * pointcloud_downsampled_msg.height);

    // Converting from PointCloud2 msg format to pcl pointcloud format
    pcl::fromROSMsg(pointcloud_downsampled_msg, pointcloud_pcl_step01);
  }
  else
  {
    // Converting from PointCloud2 msg format to pcl pointcloud format
    pcl::fromROSMsg(pointcloud2_msg, pointcloud_pcl_step01);
  }

  // STEP 02: Check if we should filter the outliers or not

  if(filter_outliers_)
  {
    // Removing outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(pointcloud_pcl_step01));
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (pointcloud_pcl_step02);
  }
  else
  {
    pointcloud_pcl_step02 = pointcloud_pcl_step01;
  }
  tree_ptr_->setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (pointcloud_pcl_step02));
  indices.resize (pointcloud_pcl_step02.points.size ());
  for (size_t i = 0; i < indices.size (); ++i)
  {
    indices[i] = i;
  }

  // STEP 03: Here we perform Normal Estimation on the input cloud

  // Object
  pcl::PointCloud<pcl::Normal> normals;
  // set parameters

  n.setInputCloud (boost::make_shared <const pcl::PointCloud<pcl::PointXYZ> > (pointcloud_pcl_step02));
  n.setIndices (boost::make_shared <std::vector<int> > (indices));
  n.setSearchMethod (tree_ptr_);
  n.setKSearch (10);

  // estimate
  n.compute (normals);

  // STEP 04: Here we copy data from the normals and the input cloud into the pcl::PointNormal cloud

  pointcloud_pcl_normals.points.resize(pointcloud_pcl_step02.points.size());

  for(u_int i = 0 ; i < pointcloud_pcl_step02.points.size(); i++)
  {
    pointcloud_pcl_normals.points[i].x = pointcloud_pcl_step02.points[i].x;
    pointcloud_pcl_normals.points[i].y = pointcloud_pcl_step02.points[i].y;
    pointcloud_pcl_normals.points[i].z = pointcloud_pcl_step02.points[i].z;
    //pointcloud_pcl_normals.points[i].rgb = pointcloud_pcl_step02.points[i].rgb;
    //    pointcloud_pcl_normals.points[i].intensity = pointcloud_pcl_step02.points[i].intensity;
    pointcloud_pcl_normals.points[i].normal[0] = normals.points[i].normal[0];
    pointcloud_pcl_normals.points[i].normal[1] = normals.points[i].normal[1];
    pointcloud_pcl_normals.points[i].normal[2] = normals.points[i].normal[2];
    pointcloud_pcl_normals.points[i].curvature = normals.points[i].curvature;
    //pointcloud_pcl_normals.points[i].scan_index = scan_index_;
  }

  pointcloud_pcl_normals.header.frame_id = pointcloud_pcl_normals.header.frame_id;
  pointcloud_pcl_normals.header.stamp = pointcloud_pcl_normals.header.stamp;
  pointcloud_pcl_normals.width    = pointcloud_pcl_normals.points.size ();
  pointcloud_pcl_normals.height   = 1;
  pointcloud_pcl_normals.is_dense = false;

  return (pointcloud_pcl_normals);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PointCloudRegistration::PointCloudRegistration(): nh_("~")
{
  nh_.param("publish_merged_pointcloud_topic", merged_pointcloud_topic_, std::string("/merged_pointcloud"));
  nh_.param("subscribe_pointcloud_topic", subscribe_pointcloud_topic_, std::string("/shoulder_cloud"));
  nh_.param("max_number_of_iterations_icp", max_number_of_iterations_icp_, 50);
  nh_.param("max_nn_icp", max_nn_icp_, 100);
  nh_.param("max_nn_overlap", max_nn_overlap_, 10);
  nh_.param("radius_icp", radius_icp_, 0.05);
  nh_.param("radius_overlap", radius_overlap_, 0.05);
  nh_.param("curvature_check", curvature_check_, true);
  nh_.param("downsample_pointcloud_before", downsample_pointcloud_before_, false);
  nh_.param("downsample_pointcloud_after", downsample_pointcloud_after_, false);
  nh_.param("filter_outliers", filter_outliers_, true);
  nh_.param("downsample_leafsize", downsample_leafsize_, 0.05);
  nh_.param("epsilon_z", epsilon_z_, 0.001);
  nh_.param("epsilon_curvature", epsilon_curvature_, 0.001);
  nh_.param("epsilon_transformation", epsilon_transformation_, 1e-6);
  firstCloudReceived_ = false;
  secondCloudReceived_ = false;
  scan_index_ = 0;
  counter_ = 0;
  icp_.setMaximumIterations(max_number_of_iterations_icp_);
  icp_.setTransformationEpsilon(epsilon_transformation_);
  icp_.setParameters(radius_icp_, max_nn_icp_, epsilon_z_, epsilon_curvature_, curvature_check_ );
  ROS_INFO("pointcloud_registration node is up and running.");

  run();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PointCloudRegistration::~PointCloudRegistration()
{
  ROS_INFO("Shutting down pointcloud_registration node!.");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudRegistration::run()
{
  pointcloud_subscriber_ = nh_.subscribe(subscribe_pointcloud_topic_, 100, &PointCloudRegistration::pointcloudRegistrationCallBack, this);
  pointcloud_merged_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(merged_pointcloud_topic_, 100);

  ros::spin();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudRegistration::pointcloudRegistrationCallBack(const sensor_msgs::PointCloud2& pointcloud_msg)
{
  counter_++;
  frame_id_ = pointcloud_msg.header.frame_id;
  start = time(NULL);
  if( firstCloudReceived_ == false)
  {
    pointcloud2_current_ = convertFromMsgToPointCloud(pointcloud_msg);
    ROS_INFO("Size of point cloud received = %d", (int) pointcloud2_current_.points.size());
    firstCloudReceived_ = true;
    ROS_INFO("Received first point cloud.");
    kdtree_.setInputCloud(boost::make_shared< pcl::PointCloud < pcl::PointNormal> > (pointcloud2_current_));

    pointcloud2_merged_ = pointcloud2_current_;
  }
  else if( secondCloudReceived_ == false)
  {
    ROS_INFO("Received second point cloud.");
    secondCloudReceived_ = true;
    pointcloud2_current_ = convertFromMsgToPointCloud(pointcloud_msg);

    //Now we get the transformation from the overlapped regions of the 2 point clouds
    final_transformation_= getOverlapTransformation();
    pcl::transformPointCloud(pointcloud2_current_, pointcloud2_transformed_, final_transformation_);
    pointcloud2_merged_ += pointcloud2_transformed_;

  }
  else
  {
    ROS_INFO("Received point cloud number: %d", counter_);
    pointcloud2_current_ = convertFromMsgToPointCloud(pointcloud_msg);
    kdtree_.setInputCloud(boost::make_shared< pcl::PointCloud < pcl::PointNormal> > (pointcloud2_merged_));

    //Now we get the transformation from the overlapped regions of the 2 point clouds
    final_transformation_= getOverlapTransformation();
    pcl::transformPointCloud(pointcloud2_current_, pointcloud2_transformed_, final_transformation_);

    pointcloud2_merged_ += pointcloud2_transformed_;
  }

  publishPointCloud(pointcloud2_merged_);
  end = time(NULL);
  ROS_INFO("Time taken: %d seconds", (int)(end - start));

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_registration");
    PointCloudRegistration pointcloud_registration;
    return(0);
}

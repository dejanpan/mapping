#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

//#include <roboterzelle/CountBox.h>

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
//#include <std_srvs/Empty.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/impl/ransac.hpp>

//#include <roboterzelle/pcl_cloud_algos/box_fit2_algo.cpp>
#include <pcl_cloud_algos/box_fit2_algo.h>
#include "visualization_msgs/Marker.h"
#include <pcl/filters/radius_outlier_removal.h>

using namespace std;
using namespace pcl;



bool boxFitting(boost::shared_ptr<const pcl::PointCloud <pcl::PointXYZINormal> > cloud, std::vector<double> &coeff)
{
  pcl_cloud_algos::RobustBoxEstimation b;
  b.setInputCloud(cloud);
  b.find_model(cloud, coeff);
  return true;
}



bool process(int argc, char** argv)
{
  if (argc != 2)
    {
      ROS_ERROR("Usage: %s <input_cloud.pcd>", argv[0]);
      exit(2);
    }
  
  sensor_msgs::PointCloud2 pc2;
  pcl::io::loadPCDFile(argv[1],pc2);
  ROS_INFO ("PointCloud has: %zu data points.", pc2.width * pc2.height);



  // ----- PCL objects -----
  pcl::PassThrough<pcl::PointXYZRGB> filter;
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr pTree (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());
  // -----------------------

  // ----- Cloud declaration -----
  sensor_msgs::PointCloud2 pc2Cloud;
  pcl::PointCloud<pcl::Normal>::Ptr pNormals (new pcl::PointCloud<pcl::Normal> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud(new pcl::PointCloud< pcl::PointXYZRGB >());
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr pCloudN(new pcl::PointCloud< pcl::PointXYZINormal >());
  pcl::PointCloud<pcl::PointXYZINormal> pCloudNFiltered;
  // -----------------------------

  pcl::fromROSMsg(pc2,*pCloud);

  // ----- Normal estimation -----
  ne.setSearchMethod (pTree);
  ne.setInputCloud (pCloud);
  ne.setKSearch (10);
  ne.compute (*pNormals);
  // -----------------------------

  pCloudN->points.resize(std::min(pNormals->points.size(),pCloud->points.size()));
  float sum = 0;
  for (size_t i = 0; i< pCloudN->points.size() ; i++)
    {
      pCloudN->points[i].x = pCloud->points[i].x;
      pCloudN->points[i].y = pCloud->points[i].y;
      pCloudN->points[i].z = pCloud->points[i].z;
      pCloudN->points[i].normal_x = pNormals->points[i].normal_x;
      pCloudN->points[i].normal_y = pNormals->points[i].normal_y;
      pCloudN->points[i].normal_z = pNormals->points[i].normal_z;
      pCloudN->points[i].intensity = 0;
      sum = pNormals->points[i].normal_x;
    }
  std::cerr<<sum<<std::endl;
  std::vector<double> coeff(15, 0.0);

  //remove noise
  pcl::RadiusOutlierRemoval<pcl::PointXYZINormal> outrem_;
  outrem_.setInputCloud (pCloudN);
  outrem_.setRadiusSearch (0.02);
  outrem_.setMinNeighborsInRadius (10);
  outrem_.filter (pCloudNFiltered);

  boxFitting(pCloudNFiltered.makeShared(), coeff);

  //publish_marker
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_, cloud_pub_;
  //advertise as latched
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("box_marker", 1, true);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("box_cloud", 1, true);
  visualization_msgs::Marker marker_;
  btMatrix3x3 box_rot (coeff[6], coeff[7], coeff[8],
		       coeff[9], coeff[10], coeff[11],
		       coeff[12], coeff[13], coeff[14]);
  btMatrix3x3 box_rot_trans (box_rot.transpose());
  btQuaternion qt;
  box_rot_trans.getRotation(qt);
  marker_.header.frame_id = "base_link";
  marker_.header.stamp = ros::Time::now();
  marker_.ns = "BoxEstimation";
  marker_.id = 0;
  marker_.type = visualization_msgs::Marker::CUBE;
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.pose.position.x = coeff[0];
  marker_.pose.position.y = coeff[1];
  marker_.pose.position.z = coeff[2];
  marker_.pose.orientation.x = qt.x();
  marker_.pose.orientation.y = qt.y();
  marker_.pose.orientation.z = qt.z();
  marker_.pose.orientation.w = qt.w();
  marker_.scale.x = coeff[3];
  marker_.scale.y = coeff[4];
  marker_.scale.z = coeff[5];
  marker_.color.a = 0.75;
  marker_.color.r = 0.0;
  marker_.color.g = 1.0;
  marker_.color.b = 0.0;


  ROS_INFO("Publishing box marker and box cloud");
  marker_pub_.publish(marker_);
  pCloudNFiltered.header.frame_id = "base_link";
  pCloudNFiltered.header.stamp = ros::Time::now();
  cloud_pub_.publish(pCloudNFiltered);

  while (nh_.ok())
    {
      sleep(1);
    }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sigIllDemo");
  ros::NodeHandle node;

  process(argc, argv);
  return 0;
}

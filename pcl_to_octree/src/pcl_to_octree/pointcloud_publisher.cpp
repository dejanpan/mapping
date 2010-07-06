#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <sensor_msgs/point_cloud_conversion.h>

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    ROS_INFO("Usage <%s> <sleep rate[s]>", argv[0]);
    exit(0);
  }
  ros::init(argc, argv, "pointcloud_publisher");
  ros::NodeHandle n;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 300;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
  }

  sensor_msgs::PointCloud2 mycloud2;
  sensor_msgs::PointCloud mycloud1;
  point_cloud::toMsg(cloud, mycloud2);
  sensor_msgs::convertPointCloud2ToPointCloud(mycloud2, mycloud1);
  
  ros::Publisher pointcloud_pub = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1);
  ros::Rate loop_rate(atof(argv[1]));
  while (ros::ok())
  {
    mycloud1.header.frame_id = "/map";
    mycloud1.header.stamp = ros::Time::now();
    pointcloud_pub.publish(mycloud1);
    ROS_INFO("PointCloud published");
    ros::spinOnce();
    loop_rate.sleep();
  }
}


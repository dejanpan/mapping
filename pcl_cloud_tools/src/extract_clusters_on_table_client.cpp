#include "ros/ros.h"
#include "pcl_cloud_tools/GetClusters.h"
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "extract_clusters_client");


  ros::NodeHandle n;
  ros::ServiceClient client =
    n.serviceClient<pcl_cloud_tools::GetClusters>("/extract_clusters_server/cluster_tracking");
  pcl_cloud_tools::GetClusters srv;
  std::vector<sensor_msgs::PointCloud2> clusters;
  if (client.call(srv))
    {
      ROS_INFO("Cluster(s) extraction suceeded");
      clusters = srv.response.clusters;
    }
  else
    {
      ROS_ERROR("Cluster extraction failed");
      return 1;
    }
  
  return 0;
}

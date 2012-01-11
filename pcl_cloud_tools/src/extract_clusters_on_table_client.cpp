#include "ros/ros.h"
#include "pcl_cloud_tools/GetClusters.h"
#include <sensor_msgs/PointCloud2.h>
#include "pcl/common/common.h"
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include "pcl/filters/extract_indices.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "extract_clusters_client");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient < pcl_cloud_tools::GetClusters
			> ("/extract_clusters_server/cluster_tracking");
	pcl_cloud_tools::GetClusters srv;
	pcl::PointCloud < pcl::PointXYZ > cloud;

	// Creating the artificial cloud
	cloud.width = 5;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	sensor_msgs::PointCloud2 cloud_ros;
	pcl::toROSMsg(cloud, cloud_ros);
	cloud_ros.header.stamp = ros::Time::now();
	cloud_ros.header.frame_id = "camera_rgb_optical_frame";
	cloud_ros.header.seq = 0;

	// Calling the service
	srv.request.input_cloud = cloud_ros;
	std::vector<pcl::PointIndices> clusters_indices;
	if (client.call(srv))
	{
		ROS_INFO("Cluster(s) extraction suceeded");
		clusters_indices = srv.response.clusters_indices;
	}
	else
	{
		ROS_ERROR("Cluster extraction failed");
		return 1;
	}

	//Extracting the point clouds from indices
	pcl::PointCloud < pcl::PointXYZ > cloud_object_clustered;
	if (int(clusters_indices.size()) >= 0)
	{
		for (int i = 0; i < clusters_indices.size(); i++)
		{
			pcl::copyPointCloud(cloud, clusters_indices[i], cloud_object_clustered);
		}
	}
	else
	{
		ROS_ERROR("Only %ld clusters found",
				clusters_indices.size());
	}

	return 0;
}

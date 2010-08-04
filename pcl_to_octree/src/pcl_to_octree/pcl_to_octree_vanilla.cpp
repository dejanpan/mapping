/*
 * Based on the original code by Kai M. Wurm and Armin Hornung
 * (http://octomap.sourceforge.net)
 * Author: Hozefa Indorewala
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "octomap/octomap.h"
//#include "octree/OcTreeNode.h"
//#include "octree/OcTree.h"
//#include "octree/OcTreeServerPCL.h"
#include "octomap_server/octomap_server.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <iostream>

class PclToOctree
{
public:
  PclToOctree();
  ~PclToOctree();
	void pclToOctreeCallback(const sensor_msgs::PointCloud& msg);
	void run();
    
private:
  ros::NodeHandle nh_;
  double laser_offset_, octree_res_; 
  int octree_maxrange_, level_;
  std::string point_cloud_topic_, frame_id_;	
  bool visualize_octree_;
	// Publishes the octree in MarkerArray format so that it can be visualized in rviz
  ros::Publisher octree_marker_array_publisher_;
	
	/*
	 * The following publisher, even though not required, is used because otherwise rviz 
	 * cannot visualize the MarkerArray format without advertising the Marker format
	 */
  ros::Publisher octree_marker_publisher_;
	
	ros::Publisher octree_binary_publisher_;
	
	// Subscribes to the PointCloud format to acquire point cloud data
	ros::Subscriber pointcloud_subscriber_;
	
	// Marker array to visualize the octree. It displays the occuplied cells of the octree
	visualization_msgs::MarkerArray octree_marker_array_msg_;
};




PclToOctree::PclToOctree() : nh_("~")
{
  nh_.param("laser_offset", laser_offset_, 1.5);
  nh_.param("octree_resolution", octree_res_, 0.5);
  nh_.param("octree_maxrange", octree_maxrange_, -1);
  nh_.param("point_cloud_topic", point_cloud_topic_, std::string("/point_cloud")); 
  nh_.param("frame_id", frame_id_, std::string("/map"));
  nh_.param("level", level_, 0);
  nh_.param("visualize_octree", visualize_octree_, false); 
  ROS_INFO("pcl_to_octree node is up and running.");
  run();   
}


void PclToOctree::run()
{
  octree_binary_publisher_ = nh_.advertise<octomap_server::OctomapBinary>("octree_binary", 100);
    
  pointcloud_subscriber_ = nh_.subscribe(point_cloud_topic_, 100, &PclToOctree::pclToOctreeCallback, this);
    
  ros::spin();
}

PclToOctree::~PclToOctree()
{
  ROS_INFO("Shutting down pcl_to_octree node!");
}

void PclToOctree::pclToOctreeCallback(const sensor_msgs::PointCloud& pointcloud_msg)
{
  ROS_INFO("Received a point cloud.");
  sensor_msgs::PointCloud2 pointcloud2_msg;
  octomap_server::OctomapBinary octree_msg;
      
  // Converting from PointCloud msg format to PointCloud2 msg format
  sensor_msgs::convertPointCloudToPointCloud2(pointcloud_msg, pointcloud2_msg);
  pcl::PointCloud<pcl::PointXYZ> pointcloud2_pcl;
    
  octomap::point3d octomap_3d_point;
  octomap::Pointcloud octomap_pointcloud;
      
  //Converting PointCloud2 msg format to pcl pointcloud format in order to read the 3d data
  pcl::fromROSMsg(pointcloud2_msg, pointcloud2_pcl);
      
  //Reading from pcl point cloud and saving it into octomap point cloud
  for(unsigned int i =0; i < pointcloud2_pcl.points.size(); i++)
  {
	  octomap_3d_point(0) = pointcloud2_pcl.points[i].x;
	  octomap_3d_point(1) = pointcloud2_pcl.points[i].y;
	  octomap_3d_point(2) = pointcloud2_pcl.points[i].z;
	  octomap_pointcloud.push_back(octomap_3d_point);
  }
    
  // Converting from octomap point cloud to octomap graph
  octomap::pose6d offset_trans(0,0,-laser_offset_,0,0,0);
  octomap::pose6d laser_pose(0,0,laser_offset_,0,0,0);
  octomap_pointcloud.transform(offset_trans);
    
    
  octomap::ScanGraph* octomap_graph = new octomap::ScanGraph();
  octomap_graph->addNode(&octomap_pointcloud, laser_pose);
    
  ROS_INFO("Number of points in graph: %d", octomap_graph->getNumPoints());
    
  // Converting from octomap graph to octomap tree (octree)
  octomap::OcTree* octree = new octomap::OcTree(octree_res_);
  for (octomap::ScanGraph::iterator scan_it = octomap_graph->begin(); scan_it != octomap_graph->end(); scan_it++)
  {
    octree->insertScan(**scan_it, octree_maxrange_, false);
  }

  
  //convert octree to OctreeBinary (serialization)
  octomap_server::octomapMapToMsg(*octree, octree_msg);
  octree_binary_publisher_.publish(octree_msg);
  ROS_INFO("OctreeBinary built and published");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_to_octree_vanilla");
  PclToOctree pcl_to_octree;
  return (0);
}

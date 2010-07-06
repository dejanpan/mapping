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
#include "pcl_to_octree/octree/OcTreeNodePCL.h"
#include "pcl_to_octree/octree/OcTreePCL.h"
#include "pcl_to_octree/octree/OcTreeServerPCL.h"
//#include "octomap_server/octomap_server.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
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
  nh_.param("octree_resolution", octree_res_, 0.05);
  nh_.param("octree_maxrange", octree_maxrange_, -1);
  nh_.param("point_cloud_topic", point_cloud_topic_, std::string("/shoulder_cloud")); 
  nh_.param("frame_id", frame_id_, std::string("/map"));
  nh_.param("level", level_, 0);
  ROS_INFO("pcl_to_octree node is up and running.");
  run();   
}


void PclToOctree::run()
{
  octree_marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
    
  octree_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    
  octree_binary_publisher_ = nh_.advertise<octomap_server::OctomapBinary>("octree_binary", 100);
    
  pointcloud_subscriber_ = nh_.subscribe(point_cloud_topic_, 100, &PclToOctree::pclToOctreeCallback, this);
    
  ros::spin();
}

PclToOctree::~PclToOctree()
{
  ROS_INFO("Shutting down pcl_to_octree node!");
  octree_marker_array_publisher_.shutdown();
  octree_marker_publisher_.shutdown();
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
  point_cloud::fromMsg(pointcloud2_msg, pointcloud2_pcl);
      
  //Reading from pcl point cloud and saving it into octomap point cloud
  for(unsigned int i =0; i < pointcloud2_pcl.points.size(); i++)
  {
	  octomap_3d_point(0) = pointcloud2_pcl.points[i].x;
	  octomap_3d_point(1) = pointcloud2_pcl.points[i].y;
	  octomap_3d_point(2) = pointcloud2_pcl.points[i].z;
	  octomap_pointcloud.push_back(octomap_3d_point);
  }
    
  /* Converting from octomap point cloud to octomap graph */
  octomap::pose6d offset_trans(0,0,-laser_offset_,0,0,0);
  octomap::pose6d laser_pose(0,0,laser_offset_,0,0,0);
  octomap_pointcloud.transform(offset_trans);
    
    
  octomap::ScanGraph* octomap_graph = new octomap::ScanGraph();
  octomap_graph->addNode(&octomap_pointcloud, laser_pose);
    
  ROS_INFO("Number of points in graph: %d", octomap_graph->getNumPoints());
    
  /*Converting from octomap graph to octomap tree (octree) */
  octomap::OcTreePCL* octree = new octomap::OcTreePCL(octree_res_);
  for (octomap::ScanGraph::iterator scan_it = octomap_graph->begin(); scan_it != octomap_graph->end(); scan_it++)
  {
    octree->insertScan(**scan_it, octree_maxrange_, false);
  }
  octomap_server::octomapMapToMsg(*octree, octree_msg);
  octree_binary_publisher_.publish(octree_msg);
  ROS_INFO("Octree built and published");

  std::list<octomap::OcTreeVolume> voxels, leaves;
  //octree->getLeafNodes(leaves, level_);
  octree->getLeafNodes(leaves);
  std::list<octomap::OcTreeVolume>::iterator it1, it2;
  int cnt = 0;
  
  std::vector < octomap::OcTreeNodePCL *> octree_node_list;
  //debug outputs
  for( it1 = leaves.begin(); it1 != leaves.end(); ++it1)
  {
    //    ROS_INFO("Leaf Node %d : x = %f y = %f z = %f side length = %f ", cnt, it1->first.x(), it1->first.y(), it1->first.z(), it1->second);
    //cnt++;
    octomap::point3d centroid;
    centroid(0) = it1->first.x(),  centroid(1) = it1->first.y(),  centroid(2) = it1->first.z();
    octomap::OcTreeNodePCL *octree_node = new octomap::OcTreeNodePCL();
    octree_node->setCentroid(centroid);
    octree_node_list.push_back(octree_node);
  }
  
  for (unsigned int ii = 0; ii < octree_node_list.size(); ii++)
  {
    cnt++;
    ROS_INFO("Leaf Node %d : x = %f y = %f z = %f", cnt, octree_node_list[ii]->getCentroid().x(), 
             octree_node_list[ii]->getCentroid().y(), octree_node_list[ii]->getCentroid().z());
  }
  octree_node_list.clear();

  ROS_INFO("Octree published %d nodes at resolution %f m.",octree->size(), octree->getResolution());
  double sx, sy, sz;
  octree->getMetricSize(sx, sy, sz);
  ROS_INFO("Octree metric size x: %f, y: %f, z: %f", sx, sy, sz);
  
  
  //OcTreeNode * on = octree->search ();
  // ---------------------- TEST CODE (TO BE REMOVED)------------------------
  /*

    
    octomap::point3d p1(pointcloud2_pcl.points[34].x, pointcloud2_pcl.points[34].y, pointcloud2_pcl.points[34].z);
    octomap::point3d p2 (pointcloud2_pcl.points[52].x, pointcloud2_pcl.points[52].y, pointcloud2_pcl.points[52].z);
    std::vector < octomap::point3d> ray;
    octree->computeRay(p1, p2, ray);
   
    std::vector<octomap::point3d>::iterator iter;
    for(iter = ray.begin(); iter != ray.end(); iter++)
    {
    std::cout<< iter->x() <<" " << iter->y()<<" " <<iter->z()<<std::endl;
    }
    
    octomap::OcTreeNode* normalNode=NULL;
    octomap::OcTreeNodeLabeled* labeledNode=NULL;
    std::vector< octomap::OcTreeNode*> normalNodeVector;
    for(unsigned int i =0; i < pointcloud2_pcl.points.size(); i++)
    {
    normalNode = octree->search(pointcloud2_pcl.points[i].x, pointcloud2_pcl.points[i].y, pointcloud2_pcl.points[i].z);
    if(normalNode != NULL)
	  normalNodeVector.push_back(normalNode);
    }
    
    ROS_INFO("Number of Nodes: %d", normalNodeVector.size());
  */
  /*
    ROS_INFO("Level = %d", level_);
    sleep(1);
    octree->getVoxels(voxels, level_);

  

    for( it1 = voxels.begin(); it1 != voxels.end(); ++it1)
    {
      
    someNode = octree->search(it1->first.x(), it1->first.y(), it1->first.z());
    if(someNode != NULL)
    ROS_INFO("Node value %f", someNode->getValue());
    //ROS_INFO("Voxel %d : x = %f y = %f z = %f side length = %f ", cnt, it1->first.x(), it1->first.y(), it1->first.z(), it1->second);
    cnt++;
    }
    cnt = 0;
    for(it2 = leaves.begin(); it2 != leaves.end(); ++it2)
    {
    ROS_INFO("Leaf %d : x = %f y = %f z = %f ", cnt, it2->first.x(), it2->first.y(), it2->first.z());
    cnt++; 
    }
  */
  //-----------------------TEST CODE ENDS--------------------------
   

  // each array stores all cubes of a different size, one for each depth level:
  octree_marker_array_msg_.markers.resize(16);
  double lowestRes = octree->getResolution();
  std::list<octomap::OcTreeVolume>::iterator it;

  for(unsigned int i = 0; i < 16; i++)
  {
    std::list<octomap::OcTreeVolume> all_cells;
    //getting the occupied cells at different depths of the octree
    //    octree->getOccupied(all_cells, i);
    octree->getLeafNodes(all_cells, i);
    for (it = all_cells.begin(); it != all_cells.end(); ++it)
    {
      // which array to store cubes in?
      int idx = int(log2(it->second / lowestRes) +0.5);  
      assert (idx >= 0 && unsigned(idx) < octree_marker_array_msg_.markers.size());
      geometry_msgs::Point cube_center;
      cube_center.x = it->first.x();
      cube_center.y = it->first.y();
      cube_center.z = it->first.z();
      octree_marker_array_msg_.markers[idx].points.push_back(cube_center);
    }
  }

  for (unsigned i = 0; i < octree_marker_array_msg_.markers.size(); ++i)
  {
    octree_marker_array_msg_.markers[i].header.frame_id = frame_id_;
    octree_marker_array_msg_.markers[i].header.stamp = ros::Time::now();
	
    double size = lowestRes * pow(2,i);

    std::stringstream ss;
    ss <<"Level "<<i;
    octree_marker_array_msg_.markers[i].ns = ss.str();
    octree_marker_array_msg_.markers[i].id = i;
    octree_marker_array_msg_.markers[i].lifetime = ros::Duration::Duration();
    octree_marker_array_msg_.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    octree_marker_array_msg_.markers[i].scale.x = size;
    octree_marker_array_msg_.markers[i].scale.y = size;
    octree_marker_array_msg_.markers[i].scale.z = size;
    octree_marker_array_msg_.markers[i].color.r = 1.0f;
    octree_marker_array_msg_.markers[i].color.g = 0.0f;
    octree_marker_array_msg_.markers[i].color.b = 0.0f;
    octree_marker_array_msg_.markers[i].color.a = 0.5f;
	
	
    if (octree_marker_array_msg_.markers[i].points.size() > 0)
	    octree_marker_array_msg_.markers[i].action = visualization_msgs::Marker::ADD;
    else
	    octree_marker_array_msg_.markers[i].action = visualization_msgs::Marker::DELETE;
		
  }

  octree_marker_array_publisher_.publish(octree_marker_array_msg_);
    
     
  for (unsigned int i = 0; i < octree_marker_array_msg_.markers.size(); i++)
  {
    if (!octree_marker_array_msg_.markers[i].points.empty())
    {
	    octree_marker_array_msg_.markers[i].points.clear();
    }
  }
  octree_marker_array_msg_.markers.clear();
    
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_to_octree");
  PclToOctree pcl_to_octree;
  return (0);
}

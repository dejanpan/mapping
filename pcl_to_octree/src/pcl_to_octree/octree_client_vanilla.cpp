/*
 * Based on the original code by Kai M. Wurm and Armin Hornung
 * (http://octomap.sourceforge.net)
 * Author: Hozefa Indorewala
 */

#include <ros/ros.h>
#include "octomap/octomap.h"
//#include "pcl_to_octree/octree/OcTreeNodePCL.h"
//#include "pcl_to_octree/octree/OcTreePCL.h"
//#include "pcl_to_octree/octree/OcTreeServerPCL.h"
#include "octomap_server/octomap_server.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <visualization_msgs/MarkerArray.h>


#include <vector>
#include <iostream>

class OctreeClient
{
public:
  OctreeClient();
  ~OctreeClient();
	void OctreeCallback(const octomap_server::OctomapBinary& mapMsg);
	void run();
    
private:
  ros::NodeHandle nh_;
  std::string octree_topic_;	
  bool visualize_octree_;
	// Publishes the octree in MarkerArray format so that it can be visualized in rviz
  ros::Publisher octree_marker_array_publisher_;
	
	/*
	 * The following publisher, even though not required, is used because otherwise rviz 
	 * cannot visualize the MarkerArray format without advertising the Marker format
	 */
  ros::Publisher octree_marker_publisher_;
	
	// Subscribes to the Octree
	ros::Subscriber octree_subscriber_;
	
	// Marker array to visualize the octree. It displays the occuplied cells of the octree
	visualization_msgs::MarkerArray octree_marker_array_msg_;
};


OctreeClient::OctreeClient() : nh_("~")
{
  nh_.param("octree_topic", octree_topic_, std::string("/pcl_to_octree_vanilla/octree_binary")); 
  nh_.param("visualize_octree", visualize_octree_, true); 
  ROS_INFO("octree_client node is up and running.");
  run();   
}


void OctreeClient::run()
{
  octree_marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
    
  octree_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    
  octree_subscriber_ = nh_.subscribe(octree_topic_, 100, &OctreeClient::OctreeCallback, this);
    
  ros::spin();
}

OctreeClient::~OctreeClient()
{
  ROS_INFO("Shutting octree_client node!");
  octree_marker_array_publisher_.shutdown();
  octree_marker_publisher_.shutdown();
}

void OctreeClient::OctreeCallback(const octomap_server::OctomapBinary& mapMsg)
{
  ROS_INFO("Received an octree.");
  octomap::OcTree* octree = new octomap::OcTree(0.5);
  octomap_server::octomapMsgToMap(mapMsg, *octree);
  ROS_INFO("OctomapBinary converted to Octree");

  //ROS_INFO("Octree Node List size: %ld",octree->octree_node_list.size());

  //**********************************************************************************
  //Visualization of Octree
  //**********************************************************************************
  if (visualize_octree_)
  {
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
      octree_marker_array_msg_.markers[i].header.frame_id = "/map";
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
    
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "octree_client");
  OctreeClient octree_client;
  return (0);
}

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "octomap/octomap.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <octomap_server/OctomapBinary.h>
#include <octomap_server/octomap_server.h>
#include <visualization_msgs/MarkerArray.h>

#define laser_offset 1.5
#define octree_res 0.5
#define octree_maxrange -1

class plocto
{
    public:
        plocto();
        //virtual ~plocto();
        void ploctoCallback(const sensor_msgs::PointCloud& msg);
    private:
        ros::NodeHandle n;
//    	ros::NodeHandle p;
    	ros::Publisher plocto_pub;
        ros::Publisher plocto_marker_pub;
    	ros::Subscriber plocto_sub;
        visualization_msgs::MarkerArray m_occupiedCellsVis;
};




plocto::plocto() : n("~")
{
    plocto_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
    plocto_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    plocto_sub = n.subscribe("/cloud_pcd", 100, &plocto::ploctoCallback, this);
}

void plocto::ploctoCallback(const sensor_msgs::PointCloud& msg)
{
    ROS_INFO("Received a point cloud.");
    sensor_msgs::PointCloud2 pc2;
    sensor_msgs::convertPointCloudToPointCloud2(msg, pc2);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    ROS_INFO("PointCloud converted to PointCloud2");
   
    octomap::point3d p;
    octomap::Pointcloud octopc;
    point_cloud::fromMsg(pc2, cloud);
    
    for(unsigned int i =0; i < cloud.points.size(); i++)
    {
        p(0) = cloud.points[i].x;
        p(1) = cloud.points[i].y;
        p(2) = cloud.points[i].z;
        octopc.push_back(p);
    }
   
   /*From PointCloud to graph */
   octomap::pose6d offset_trans(0,0,-laser_offset,0,0,0);
   octomap::pose6d laser_pose(0,0,laser_offset,0,0,0);
   octopc.transform(offset_trans);
   
   
   octomap::ScanGraph* graph = new octomap::ScanGraph();
   graph->addNode(&octopc, laser_pose);
   ROS_INFO("Number of points in graph: %d", graph->getNumPoints());
  
   /*From Graph to Tree */
   octomap::OcTree* tree = new octomap::OcTree(octree_res);
   for (octomap::ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++)
   {
       tree->insertScan(**scan_it, octree_maxrange, false);
   }

   ROS_INFO("Octree built");
   
   octomap_server::OctomapBinary octobin_msg;
   octobin_msg.header.frame_id = "/base_link";
   octobin_msg.header.stamp = ros::Time::now();
   octomap_server::octomapMapToMsg(*tree, octobin_msg);

   // each array stores all cubes of a different size, one for each depth level:
	m_occupiedCellsVis.markers.resize(16);
	double lowestRes = tree->getResolution();

	std::list<octomap::OcTreeVolume> occupiedCells;
	tree->getOccupied(occupiedCells);

	// rough heuristics for expected size of cells at lowest level
	m_occupiedCellsVis.markers[0].points.reserve(occupiedCells.size());
	m_occupiedCellsVis.markers[1].points.reserve(occupiedCells.size()/2);
	m_occupiedCellsVis.markers[2].points.reserve(occupiedCells.size()/4);
	m_occupiedCellsVis.markers[3].points.reserve(occupiedCells.size()/4);


	std::list<octomap::OcTreeVolume>::iterator it;

	for (it = occupiedCells.begin(); it != occupiedCells.end(); ++it){
		// which array to store cubes in?
		int idx = int(log2(it->second / lowestRes) +0.5);
		assert (idx >= 0 && unsigned(idx) < m_occupiedCellsVis.markers.size());
		geometry_msgs::Point cubeCenter;
		cubeCenter.x = it->first.x();
		cubeCenter.y = it->first.y();
		cubeCenter.z = it->first.z();
		m_occupiedCellsVis.markers[idx].points.push_back(cubeCenter);
	}

	for (unsigned i= 0; i < m_occupiedCellsVis.markers.size(); ++i){
		double size = lowestRes * pow(2,i);

		m_occupiedCellsVis.markers[i].header.frame_id = "/base_link";
		m_occupiedCellsVis.markers[i].header.stamp = ros::Time::now();
    std::stringstream ss;
    ss << i;
		m_occupiedCellsVis.markers[i].ns = ss.str();
		m_occupiedCellsVis.markers[i].id = i;
		m_occupiedCellsVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
		m_occupiedCellsVis.markers[i].scale.x = size;
		m_occupiedCellsVis.markers[i].scale.y = size;
		m_occupiedCellsVis.markers[i].scale.z = size;
		m_occupiedCellsVis.markers[i].color.r = 1.0f;
		m_occupiedCellsVis.markers[i].color.g = 0.0f;
		m_occupiedCellsVis.markers[i].color.b = 0.0f;
		m_occupiedCellsVis.markers[i].color.a = 0.5f;

		if (m_occupiedCellsVis.markers[i].points.size() > 0)
			m_occupiedCellsVis.markers[i].action = visualization_msgs::Marker::ADD;
		else
			m_occupiedCellsVis.markers[i].action = visualization_msgs::Marker::DELETE;
		
		//plocto_marker_pub.publish(m_occupiedCellsVis.markers[i]);
	}

	plocto_pub.publish(m_occupiedCellsVis);
  for (int i = 0; i < m_occupiedCellsVis.markers.size(); i++)
  {
    if (!m_occupiedCellsVis.markers[i].points.empty())
    {
      m_occupiedCellsVis.markers[i].points.clear();
      ROS_INFO("m_occupiedCellsVis array %d", i);
    }
  }

  ROS_INFO("Octomap Published");
	ROS_INFO("Octomap file published (%d nodes).",tree->size());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_to_octree");
    
    plocto pr;
    ros::spin();
}

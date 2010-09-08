#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "octomap/octomap.h"
#include <octomap_server/OctomapBinary.h>
#include "pcl_to_octree/octree/OcTreePCL.h"
#include "pcl_to_octree/octree/OcTreeNodePCL.h"
#include "pcl_to_octree/octree/OcTreeServerPCL.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/MarkerArray.h>

#include <vector>

/**
@file

@brief  next best view.

@par Advertises
 - \b /nbv_octree topic

@par Subscribes
 - \b /cloud_pcd topic

@par Parameters
- \b input_cloud_topic
- \b output_octree_topic
*/


class Nbv
{
protected:
	ros::NodeHandle nh_;
	std::string input_cloud_topic_;
	std::string output_octree_topic_;

	ros::Subscriber cloud_sub_;
	ros::Publisher octree_pub_;

	sensor_msgs::PointCloud2 cloud_in_;

	octomap::OcTreePCL* octree_;
	octomap::ScanGraph* octomap_graph_;
	double laser_offset_, octree_res_, octree_maxrange_;
	int level_, free_label_, occupied_label_, unknown_label_;
	bool check_centroids_;
	bool visualize_octree_;

	tf::TransformListener tf_listener_;


	void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg);
	void createOctree (pcl::PointCloud<pcl::PointXYZ>& pointcloud2_pcl, octomap::pose6d laser_pose);


	// Publishes the octree in MarkerArray format so that it can be visualized in rviz
	ros::Publisher octree_marker_array_publisher_;

	/*
	 * The following publisher, even though not required, is used because otherwise rviz
	 * cannot visualize the MarkerArray format without advertising the Marker format
	 */
	ros::Publisher octree_marker_publisher_;

	// Marker array to visualize the octree. It displays the occuplied cells of the octree
	visualization_msgs::MarkerArray octree_marker_array_msg_;

public:
	Nbv(ros::NodeHandle &anode);
	~Nbv();
};


Nbv::Nbv (ros::NodeHandle &anode) : nh_(anode) {
	nh_.param ("input_cloud_topic", input_cloud_topic_, std::string("/cloud_pcd"));
	nh_.param ("output_octree_topic", output_octree_topic_, std::string("/nbv_octree"));

	nh_.param("laser_offset", laser_offset_, 1.5);
	nh_.param("octree_resolution", octree_res_, 0.10);
	nh_.param("octree_maxrange", octree_maxrange_, -1.0);
	nh_.param("level", level_, 0);
	nh_.param("check_centroids", check_centroids_, false);
	nh_.param("free_label", free_label_, 0);
	nh_.param("occupied_label", occupied_label_, 1);
	nh_.param("unknown_label", unknown_label_, -1);

	nh_.param("visualize_octree", visualize_octree_, true);

	cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &Nbv::cloud_cb, this);
	octree_pub_ = nh_.advertise<octomap_server::OctomapBinary> (output_octree_topic_, 1);

	octree_marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
	octree_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 100);

	octree_ = NULL;
}

Nbv::~Nbv()
{
  ROS_INFO("Shutting down next_best_view node!");

  octree_marker_array_publisher_.shutdown();
  octree_marker_publisher_.shutdown();
}

/**
* \brief cloud callback and the core filtering function at the same time
* \param pointcloud2_msg input point cloud to be processed
*/
void Nbv::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg) {

	pcl::PointCloud<pcl::PointXYZ> pointcloud2_pcl;
	octomap::point3d octomap_point3d;

	ROS_INFO("Received point cloud");

	tf::StampedTransform transform;
	try {
		ros::Time acquisition_time = pointcloud2_msg->header.stamp;
		ros::Duration timeout(1.0 / 30);
		tf_listener_.waitForTransform(pointcloud2_msg->header.frame_id, "/laser_tilt_link", acquisition_time, timeout);
		tf_listener_.lookupTransform(pointcloud2_msg->header.frame_id, "/laser_tilt_link", acquisition_time, transform);
	}
	catch (tf::TransformException& ex) {
		ROS_WARN("[next_best_view] TF exception:\n%s", ex.what());
	}
	tf::Point pt = transform.getOrigin();
	//octomap::pose6d laser_pose (-0.526,0.921,1.317,0,0,0);
	octomap::pose6d laser_pose (pt.x(), pt.y(), pt.z(),0,0,0);
	octomap::point3d laser_origin (laser_pose.trans());

	ROS_INFO("viewpoint [%f %f %f]", pt.x(), pt.y(), pt.z());


	//Converting PointCloud2 msg format to pcl pointcloud format in order to read the 3d data
	pcl::fromROSMsg(*pointcloud2_msg, pointcloud2_pcl);

	if (octree_ == NULL) {
		octomap_graph_ = new octomap::ScanGraph();
		octree_ = new octomap::OcTreePCL(octree_res_);
	}

	createOctree(pointcloud2_pcl, laser_pose);

	//assign points to Leaf Nodes  and cast rays from laser pos to point
	BOOST_FOREACH (const pcl::PointXYZ& pcl_pt, pointcloud2_pcl.points) {
		octomap_point3d(0) = pcl_pt.x;
		octomap_point3d(1) = pcl_pt.y;
		octomap_point3d(2) = pcl_pt.z;
		octomap::OcTreeNodePCL * octree_node1 = octree_->search(octomap_point3d);
		if (octree_node1 != NULL) {
			 // Get the nodes along the ray and label them as free
			std::vector<octomap::point3d> ray;
			if (octree_->computeRay(laser_origin, octomap_point3d, ray)) {
				BOOST_FOREACH (const octomap::point3d& pt, ray) {
					octomap::OcTreeNodePCL * free_node = octree_->search(pt);
					if (free_node != NULL) {
						if (free_node->getLabel() != occupied_label_)
							free_node->setLabel(free_label_);
					}
					else
						ROS_DEBUG("free node at x=%f y=%f z=%f not found", pt(0), pt(1), pt(2));
				}
			} else {
				ROS_DEBUG("could not compute ray from [%f %f %f] to [%f %f %f]",
							laser_pose.x(), laser_pose.y(), laser_pose.z(), pcl_pt.x, pcl_pt.y, pcl_pt.z);
			}

			octree_node1->set3DPointInliers(0);
			octree_node1->setLabel(occupied_label_);
		} else {
			ROS_DEBUG("ERROR: node at [%f %f %f] not found", pcl_pt.x, pcl_pt.y, pcl_pt.z);
		}
	}

	// publish binary octree
	octomap_server::OctomapBinary octree_msg;
	octomap_server::octomapMapToMsg(*octree_, octree_msg);
	octree_pub_.publish(octree_msg);
	ROS_INFO("Octree built and published");


  //**********************************************************************************
  //Visualization of Octree
  //**********************************************************************************
  if (visualize_octree_)
  {
    // each array stores all cubes of a different size, one for each depth level:
    octree_marker_array_msg_.markers.resize(16);
    double lowestRes = octree_->getResolution();
    //ROS_INFO_STREAM("lowest resolution: " << lowestRes);
    std::list<octomap::OcTreeVolume>::iterator it;

      std::list<octomap::OcTreeVolume> all_cells;
      //getting the cells at level 0
      octree_->getLeafNodes(all_cells, 0);
      for (it = all_cells.begin(); it != all_cells.end(); ++it)
      {
	geometry_msgs::Point cube_center;
	cube_center.x = it->first.x();
	cube_center.y = it->first.y();
	cube_center.z = it->first.z();
	octomap::point3d octo_point (cube_center.x, cube_center.y, cube_center.z);
	octomap::OcTreeNodePCL * node = octree_->search(octo_point);
	if (node != NULL) {
		if (occupied_label_ == node->getLabel())
			octree_marker_array_msg_.markers[0].points.push_back(cube_center);
		else if (free_label_ == node->getLabel())
			octree_marker_array_msg_.markers[1].points.push_back(cube_center);
		else if (unknown_label_ == node->getLabel())
			octree_marker_array_msg_.markers[2].points.push_back(cube_center);
	}
      }
      geometry_msgs::Point viewpoint;
      viewpoint.x = pt.x();
      viewpoint.y = pt.y();
      viewpoint.z = pt.z();
      octree_marker_array_msg_.markers[3].points.push_back(viewpoint);

      // occupied cells
      octree_marker_array_msg_.markers[0].ns = "Occupied cells";
      octree_marker_array_msg_.markers[0].color.r = 1.0f;
      octree_marker_array_msg_.markers[0].color.g = 0.0f;
      octree_marker_array_msg_.markers[0].color.b = 0.0f;
      octree_marker_array_msg_.markers[0].color.a = 0.5f;

      // free cells
      octree_marker_array_msg_.markers[1].ns ="Free cells";
      octree_marker_array_msg_.markers[1].color.r = 0.0f;
      octree_marker_array_msg_.markers[1].color.g = 1.0f;
      octree_marker_array_msg_.markers[1].color.b = 0.0f;
      octree_marker_array_msg_.markers[1].color.a = 0.5f;

      // unknown cells
      octree_marker_array_msg_.markers[2].ns = "Unknown cells";
      octree_marker_array_msg_.markers[2].color.r = 0.0f;
      octree_marker_array_msg_.markers[2].color.g = 0.0f;
      octree_marker_array_msg_.markers[2].color.b = 1.0f;
      octree_marker_array_msg_.markers[2].color.a = 0.05f;

      // viewpoint
      octree_marker_array_msg_.markers[3].ns = "viewpoint";
      octree_marker_array_msg_.markers[3].color.r = 1.0f;
      octree_marker_array_msg_.markers[3].color.g = 1.0f;
      octree_marker_array_msg_.markers[3].color.b = 0.0f;
      octree_marker_array_msg_.markers[3].color.a = 0.8f;

    for (unsigned i = 0; i < octree_marker_array_msg_.markers.size(); ++i)
    {
      octree_marker_array_msg_.markers[i].header.frame_id = pointcloud2_msg->header.frame_id;
      octree_marker_array_msg_.markers[i].header.stamp = ros::Time::now();

      octree_marker_array_msg_.markers[i].id = i;
      octree_marker_array_msg_.markers[i].lifetime = ros::Duration::Duration();
      octree_marker_array_msg_.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      octree_marker_array_msg_.markers[i].scale.x = lowestRes;
      octree_marker_array_msg_.markers[i].scale.y = lowestRes;
      octree_marker_array_msg_.markers[i].scale.z = lowestRes;


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


/**
* creating an octree from pcl data
*/
void Nbv::createOctree (pcl::PointCloud<pcl::PointXYZ>& pointcloud2_pcl, octomap::pose6d laser_pose) {

	octomap::point3d octomap_3d_point;
	octomap::Pointcloud octomap_pointcloud;

	//Reading from pcl point cloud and saving it into octomap point cloud
	BOOST_FOREACH (const pcl::PointXYZ& pt, pointcloud2_pcl.points) {
		octomap_3d_point(0) = pt.x;
		octomap_3d_point(1) = pt.y;
		octomap_3d_point(2) = pt.z;
		octomap_pointcloud.push_back(octomap_3d_point);
	}

	// Converting from octomap point cloud to octomap graph
	octomap_pointcloud.transform(laser_pose.inv());
	octomap::ScanNode* scan_node = octomap_graph_->addNode(&octomap_pointcloud, laser_pose);

	ROS_INFO("Number of points in scene graph: %d", octomap_graph_->getNumPoints());

	// Converting from octomap graph to octomap tree (octree)
	octree_->insertScan(*scan_node, octree_maxrange_, false);

	octree_->expand();

	//
	// create nodes that are unknown
	//
	octomap::point3d min, max;
	octree_->getMetricMin(min(0), min(1), min(2));
	octree_->getMetricMax(max(0), max(1), max(2));
	//ROS_INFO("octree min bounds [%f %f %f]", min(0), min(1), min(2));
	//ROS_INFO("octree max bounds [%f %f %f]", max(0), max(1), max(2));

	double x,y,z;
	for (x = min(0)+octree_res_/2; x < max(0)-octree_res_/2; x+=octree_res_) {
		for (y = min(1)+octree_res_/2; y < max(1)-octree_res_/2; y+=octree_res_) {
			for (z = min(2)+octree_res_/2; z < max(2)-octree_res_/2; z+=octree_res_) {
				octomap::point3d centroid (x, y, z);
				if (z > max(2))
					ROS_INFO("ahrg node at [%f %f %f]", centroid(0), centroid(1), centroid(2));
				octomap::OcTreeNodePCL *octree_node = octree_->search(centroid);
				if (octree_node != NULL) {
					octree_node->setCentroid(centroid);
				} else {
					//ROS_INFO("creating node at [%f %f %f]", centroid(0), centroid(1), centroid(2));
					//corresponding node doesn't exist yet -> create it
					octomap::OcTreeNodePCL *new_node = octree_->updateNode(centroid, false);
					new_node->setCentroid(centroid);
					new_node->setLabel(unknown_label_);
				}
			}
		}
	}


	if (check_centroids_) {
		//int cnt = 0;
		// get all existing leaves
		std::list<octomap::OcTreeVolume> leaves;
		octree_->getLeafNodes(leaves);
		std::list<octomap::OcTreeVolume>::iterator it1;

		//find Leaf Nodes' centroids, assign controid coordinates to Leaf Node
		for( it1 = leaves.begin(); it1 != leaves.end(); ++it1) {
			//ROS_DEBUG("Leaf Node %d : x = %f y = %f z = %f side length = %f ", cnt++, it1->first.x(), it1->first.y(), it1->first.z(), it1->second);
			octomap::point3d centroid;
			centroid(0) = it1->first.x(),  centroid(1) = it1->first.y(),  centroid(2) = it1->first.z();
			octomap::OcTreeNodePCL *octree_node = octree_->search(centroid);
			if (octree_node != NULL) {
				octomap::point3d test_centroid;
				test_centroid = octree_node->getCentroid();
				if (centroid.dist(test_centroid) > octree_res_/4)
					ROS_INFO("node at [%f %f %f] has a wrong centroid: [%f %f %f]", centroid(0), centroid(1), centroid(2), test_centroid(0), test_centroid(1), test_centroid(2));
			}
			else {
				ROS_INFO("node at [%f %f %f] not found", centroid(0), centroid(1), centroid(2));
			}
		}
	}
}


int main (int argc, char* argv[])
{
	ros::init (argc, argv, "next_best_view");
	ros::NodeHandle nh("~");
	Nbv n (nh);
	ROS_INFO("Node up and running...");
	ros::spin ();

	return (0);
}

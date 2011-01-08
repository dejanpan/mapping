#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PoseArray.h>

#include "octomap/octomap.h"
#include <octomap_server/OctomapBinary.h>
#include "pcl_to_octree/octree/OcTreePCL.h"
#include "pcl_to_octree/octree/OcTreeNodePCL.h"
#include "pcl_to_octree/octree/OcTreeServerPCL.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl_ros/publisher.h>

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

  //parameters
  std::string input_cloud_topic_;
  std::string output_octree_topic_;
  std::string output_pose_topic_;
  std::string laser_frame_;

  double octree_res_, octree_maxrange_;
  int level_, free_label_, occupied_label_, unknown_label_;
  bool check_centroids_;
  bool visualize_octree_;

  double normal_search_radius_;
  int min_pts_per_cluster_;
  double eps_angle_;
  double tolerance_;
  double boundary_angle_threshold_;

  //objects needed
  tf::TransformListener tf_listener_;

  //datasets
  octomap::OcTreePCL* octree_;
  octomap::ScanGraph* octomap_graph_;

  sensor_msgs::PointCloud2 cloud_in_;
  geometry_msgs::PoseArray nbv_pose_array_;

  ros::Subscriber cloud_sub_;
  ros::Publisher octree_pub_;
  pcl_ros::Publisher<pcl::PointNormal> border_cloud_pub_;
  pcl_ros::Publisher<pcl::PointXYZ> cluster_cloud_pub_;
  pcl_ros::Publisher<pcl::PointXYZ> cluster_cloud2_pub_;
  pcl_ros::Publisher<pcl::PointXYZ> cluster_cloud3_pub_;
  ros::Publisher pose_pub_;

  // Publishes the octree in MarkerArray format so that it can be visualized in rviz
  ros::Publisher octree_marker_array_publisher_;
  /* The following publisher, even though not required, is used because otherwise rviz
   * cannot visualize the MarkerArray format without advertising the Marker format*/
  ros::Publisher octree_marker_publisher_;
  // Marker array to visualize the octree. It displays the occuplied cells of the octree
  visualization_msgs::MarkerArray octree_marker_array_msg_;

  static bool compareClusters(pcl::PointIndices c1, pcl::PointIndices c2) { return (c1.indices.size() < c2.indices.size()); }

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg);
  void createOctree (pcl::PointCloud<pcl::PointXYZ>& pointcloud2_pcl, octomap::pose6d laser_pose);
  void visualizeOctree(const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg, geometry_msgs::Point viewpoint);
  void castRayAndLabel(pcl::PointCloud<pcl::PointXYZ>& cloud, octomap::pose6d origin);
  void findBorderPoints(pcl::PointCloud<pcl::PointXYZ>& border_cloud, std::string frame_id);
  void computeBoundaryPoints(pcl::PointCloud<pcl::PointXYZ>& border_cloud, pcl::PointCloud<pcl::Normal>& border_normals, std::vector<pcl::PointIndices>& clusters, pcl::PointCloud<pcl::PointXYZ>* cluster_clouds);

  void extractClusters (const pcl::PointCloud<pcl::PointXYZ> &cloud,
				 const pcl::PointCloud<pcl::Normal> &normals,
				 float tolerance,
				 const boost::shared_ptr<pcl::KdTree<pcl::PointXYZ> > &tree,
				 std::vector<pcl::PointIndices> &clusters, double eps_angle,
				 unsigned int min_pts_per_cluster = 1,
				 unsigned int max_pts_per_cluster = std::numeric_limits<int>::max ());


public:
  Nbv(ros::NodeHandle &anode);
  ~Nbv();
};


Nbv::Nbv (ros::NodeHandle &anode) : nh_(anode) {
  nh_.param("input_cloud_topic", input_cloud_topic_, std::string("/points2_out"));
  nh_.param("output_octree_topic", output_octree_topic_, std::string("/nbv_octree"));
  nh_.param("output_pose_topic", output_pose_topic_, std::string("/nbv_pose"));
  nh_.param("laser_frame", laser_frame_, std::string("/laser_tilt_link"));
  nh_.param("octree_resolution", octree_res_, 0.1);

  nh_.param("octree_maxrange", octree_maxrange_, -1.0);
  nh_.param("level", level_, 0);
  nh_.param("check_centroids", check_centroids_, false);
  nh_.param("free_label", free_label_, 0);
  nh_.param("occupied_label", occupied_label_, 1);
  nh_.param("unknown_label", unknown_label_, -1);
  nh_.param("visualize_octree", visualize_octree_, true);

  nh_.param("normal_search_radius", normal_search_radius_, 0.6);
  nh_.param("min_pts_per_cluster", min_pts_per_cluster_, 10);
  nh_.param("eps_angle", eps_angle_, 0.25);
  nh_.param("tolerance", tolerance_, 0.3);
  nh_.param("boundary_angle_threshold", boundary_angle_threshold_, 2.5);

  cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &Nbv::cloud_cb, this);
  octree_pub_ = nh_.advertise<octomap_server::OctomapBinary> (output_octree_topic_, 1);
  border_cloud_pub_ = pcl_ros::Publisher<pcl::PointNormal> (nh_, "border_cloud", 1);
  cluster_cloud_pub_ = pcl_ros::Publisher<pcl::PointXYZ> (nh_, "cluster_cloud", 1);
  cluster_cloud2_pub_ = pcl_ros::Publisher<pcl::PointXYZ> (nh_, "cluster_cloud2", 1);
  cluster_cloud3_pub_ = pcl_ros::Publisher<pcl::PointXYZ> (nh_, "cluster_cloud3", 1);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseArray> (output_pose_topic_, 1);

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

  //get the latest parameters
  nh_.getParam("normal_search_radius", normal_search_radius_);
  nh_.getParam("min_pts_per_cluster", min_pts_per_cluster_);
  nh_.getParam("eps_angle", eps_angle_);
  nh_.getParam("tolerance", tolerance_);
  nh_.getParam("boundary_angle_threshold", boundary_angle_threshold_);

  //get the viewpoint (position of laser) from tf
  tf::StampedTransform transform;
  try {
    ros::Time acquisition_time = pointcloud2_msg->header.stamp;
    ros::Duration timeout(1.0 / 30);
    tf_listener_.waitForTransform(pointcloud2_msg->header.frame_id, laser_frame_, acquisition_time, timeout);
    tf_listener_.lookupTransform(pointcloud2_msg->header.frame_id, laser_frame_, acquisition_time, transform);
  }
  catch (tf::TransformException& ex) {
    ROS_WARN("[next_best_view] TF exception:\n%s", ex.what());
  }
  tf::Point pt = transform.getOrigin();
  octomap::pose6d laser_pose (pt.x(), pt.y(), pt.z(),0,0,0);
  ROS_INFO("viewpoint [%f %f %f]", pt.x(), pt.y(), pt.z());


  //Converting PointCloud2 msg format to pcl pointcloud format in order to read the 3d data
  pcl::fromROSMsg(*pointcloud2_msg, pointcloud2_pcl);

  // create or update the octree
  if (octree_ == NULL) {
    octomap_graph_ = new octomap::ScanGraph();
    octree_ = new octomap::OcTreePCL(octree_res_);
  }
  createOctree(pointcloud2_pcl, laser_pose);

  /*
   * assign new points to Leaf Nodes  and cast rays from laser pos to point
   */
  castRayAndLabel(pointcloud2_pcl, laser_pose);

  /*
   * find unknown voxels with free neighbors and add them to a pointcloud
   */
  pcl::PointCloud<pcl::PointXYZ> border_cloud;
  findBorderPoints(border_cloud, pointcloud2_msg->header.frame_id);

  // Create the filtering objects
  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ExtractIndices<pcl::Normal> nextract;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

  //creating datasets
  pcl::PointCloud<pcl::Normal> border_normals;

  //filter out ceiling
  pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (border_cloud));
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 2.2);
  pass.filter(border_cloud);

  // tree object used for search
  pcl::KdTreeANN<pcl::PointXYZ>::Ptr tree = boost::make_shared<pcl::KdTreeANN<pcl::PointXYZ> > ();

  // Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (border_cloud));
  ne.setRadiusSearch(normal_search_radius_);
  //ne.setKSearch(0);
  ne.compute(border_normals);

  //filter again to remove spurious NaNs
  pcl::PointIndices nan_indices;
  for (unsigned int i = 0; i < border_normals.points.size(); i++) {
    if (isnan(border_normals.points[i].normal[0]))
      nan_indices.indices.push_back(i);
  }
  ROS_INFO("%d NaNs found", (int)nan_indices.indices.size());
  //in pointcloud
  extract.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (border_cloud));
  extract.setIndices(boost::make_shared<pcl::PointIndices> (nan_indices));
  extract.setNegative(true);
  extract.filter(border_cloud);
  ROS_INFO("%d points in border cloud after filtering and NaN removal", (int)border_cloud.points.size());
  //and in the normals
  nextract.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::Normal> > (border_normals));
  nextract.setIndices(boost::make_shared<pcl::PointIndices> (nan_indices));
  nextract.setNegative(true);
  nextract.filter(border_normals);
  ROS_INFO("%d points in normals cloud after NaN removal", (int)border_normals.points.size());

  // tree object used for search
  pcl::KdTreeANN<pcl::PointXYZ>::Ptr tree2 = boost::make_shared<pcl::KdTreeANN<pcl::PointXYZ> > ();

  tree2->setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (border_cloud));
  // Decompose a region of space into clusters based on the euclidean distance between points, and the normal
  std::vector<pcl::PointIndices> clusters;
  pcl::extractEuclideanClusters <pcl::PointXYZ, pcl::Normal> (border_cloud, border_normals, tolerance_, tree2, clusters, eps_angle_, min_pts_per_cluster_);

  pcl::PointCloud<pcl::PointXYZ> cluster_clouds[3];

  /*
   * compute boundary points from clusters and put them into pose array
   */
  computeBoundaryPoints(border_cloud, border_normals, clusters, cluster_clouds);

  //publish poses
  pose_pub_.publish(nbv_pose_array_);

  //publish border cloud for visualization
  pcl::PointCloud<pcl::PointNormal> border_pn_cloud;
  if (border_cloud.points.size() > 0) {
    pcl::concatenateFields(border_cloud, border_normals, border_pn_cloud);
  } else {
    border_pn_cloud.header.frame_id = border_cloud.header.frame_id;
    border_pn_cloud.header.stamp = ros::Time::now();
  }
  border_cloud_pub_.publish(border_pn_cloud);

  //publish the clusters for visualization
  for (unsigned int i = 0; i < 3; i++) {
    if (cluster_clouds[i].points.size() == 0) {
      cluster_clouds[i].header.frame_id = border_cloud.header.frame_id;
      cluster_clouds[i].header.stamp = ros::Time::now();
    }
  }
  cluster_cloud_pub_.publish(cluster_clouds[0]);
  cluster_cloud2_pub_.publish(cluster_clouds[1]);
  cluster_cloud3_pub_.publish(cluster_clouds[2]);

  // publish binary octree
  if (0) {
    octomap_server::OctomapBinary octree_msg;
    octomap_server::octomapMapToMsg(*octree_, octree_msg);
    octree_pub_.publish(octree_msg);
  }

  ROS_INFO("All computed and published");

  //**********************************************************************************
  //Visualization
  //**********************************************************************************
  if (visualize_octree_)
  {
    geometry_msgs::Point viewpoint;
    viewpoint.x = pt.x();
    viewpoint.y = pt.y();
    viewpoint.z = pt.z();
    visualizeOctree(pointcloud2_msg, viewpoint);
  }
}

void Nbv::computeBoundaryPoints(pcl::PointCloud<pcl::PointXYZ>& border_cloud, pcl::PointCloud<pcl::Normal>& border_normals, std::vector<pcl::PointIndices>& clusters, pcl::PointCloud<pcl::PointXYZ>* cluster_clouds) {
  //clear old poses
  nbv_pose_array_.poses.clear();

  if (clusters.size() > 0) {
    ROS_INFO ("%d clusters found.", (int)clusters.size());
    // sort the clusters according to number of points they contain
    std::sort(clusters.begin(), clusters.end(), compareClusters);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::Normal> nextract;

    for (unsigned int nc = 0; nc < clusters.size(); nc++) {
      //extract maximum of 3 biggest clusters
      if (nc == 3)
	break;

      //extract a cluster
      pcl::PointCloud<pcl::PointXYZ> cluster_cloud;
      extract.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (border_cloud));
      extract.setIndices(boost::make_shared<pcl::PointIndices> (clusters.back()));
      extract.setNegative(false);
      extract.filter(cluster_cloud);
      ROS_INFO ("PointCloud representing the cluster %d: %d data points.", nc, cluster_cloud.width * cluster_cloud.height);

      //extract normals of cluster
      pcl::PointCloud<pcl::Normal> cluster_normals;
      nextract.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::Normal> > (border_normals));
      nextract.setIndices(boost::make_shared<pcl::PointIndices> (clusters.back()));
      nextract.setNegative(false);
      nextract.filter(cluster_normals);

      // find boundary points of cluster
      pcl::KdTreeANN<pcl::PointXYZ>::Ptr tree3 = boost::make_shared<pcl::KdTreeANN<pcl::PointXYZ> > ();
      pcl::PointCloud<pcl::Boundary> boundary_cloud;
      pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> be;
      be.setSearchMethod(tree3);
      be.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cluster_cloud));
      be.setInputNormals(boost::make_shared<pcl::PointCloud<pcl::Normal> > (cluster_normals));
      be.setRadiusSearch(.5);
      be.angle_threshold_ = boundary_angle_threshold_;
      be.compute(boundary_cloud);

      geometry_msgs::Pose nbv_pose;
      unsigned int nbp = 0;
      for (unsigned int i = 0; i < boundary_cloud.points.size(); ++i) {
	if (boundary_cloud.points[i].boundary_point) {
	  nbv_pose.position.x = cluster_cloud.points[i].x;
	  nbv_pose.position.y = cluster_cloud.points[i].y;
	  nbv_pose.position.z = cluster_cloud.points[i].z;
	  btVector3 axis(0, -cluster_normals.points[i].normal[2], cluster_normals.points[i].normal[1]);
	  btQuaternion quat(axis, axis.length());
	  geometry_msgs::Quaternion quat_msg;
	  tf::quaternionTFToMsg(quat, quat_msg);
	  nbv_pose.orientation = quat_msg;
	  nbv_pose_array_.poses.push_back(nbv_pose);
	  nbp++;
	}
      }
      ROS_INFO ("%d boundary points in cluster %d.", nbp, nc);

      //save this cluster pointcloud for visualization
      cluster_clouds[nc] = cluster_cloud;

      //pop the just used cluster from indices
      clusters.pop_back();
    }
    nbv_pose_array_.header.frame_id = border_cloud.header.frame_id;
    nbv_pose_array_.header.stamp = ros::Time::now();
  }
  else {
    ROS_INFO ("No clusters found!");
  }
}

void Nbv::castRayAndLabel(pcl::PointCloud<pcl::PointXYZ>& cloud, octomap::pose6d origin) {
  octomap::point3d octomap_point3d;

  BOOST_FOREACH (const pcl::PointXYZ& pcl_pt, cloud.points) {
    octomap_point3d(0) = pcl_pt.x;
    octomap_point3d(1) = pcl_pt.y;
    octomap_point3d(2) = pcl_pt.z;
    octomap::OcTreeNodePCL * octree_node1 = octree_->search(octomap_point3d);
    if (octree_node1 != NULL) {
      // Get the nodes along the ray and label them as free
      std::vector<octomap::point3d> ray;
      if (octree_->computeRay(origin.trans(), octomap_point3d, ray)) {
	BOOST_FOREACH (const octomap::point3d& pt, ray) {
	  octomap::OcTreeNodePCL * free_node = octree_->search(pt);
	  if (free_node != NULL) {
	    if (free_node->getLabel() != occupied_label_)
	      free_node->setLabel(free_label_);
	  }
	  else
	    ROS_DEBUG("free node at x=%f y=%f z=%f not found", pt(0), pt(1), pt(2));
	}
      }
      else {
	ROS_DEBUG("could not compute ray from [%f %f %f] to [%f %f %f]",
		  origin.x(), origin.y(), origin.z(), pcl_pt.x, pcl_pt.y, pcl_pt.z);
      }
      octree_node1->set3DPointInliers(0);
      octree_node1->setLabel(occupied_label_);
    }
    else {
      ROS_DEBUG("ERROR: node at [%f %f %f] not found", pcl_pt.x, pcl_pt.y, pcl_pt.z);
    }
  }
}

void Nbv::findBorderPoints(pcl::PointCloud<pcl::PointXYZ>& border_cloud, std::string frame_id) {
  border_cloud.header.frame_id = frame_id;
  border_cloud.header.stamp = ros::Time::now();
  std::list<octomap::OcTreeVolume> leaves;
  octree_->getLeafNodes(leaves);
  BOOST_FOREACH(octomap::OcTreeVolume vol, leaves) {
    octomap::point3d centroid;
    centroid(0) = vol.first.x(),  centroid(1) = vol.first.y(),  centroid(2) = vol.first.z();
    octomap::OcTreeNodePCL *octree_node = octree_->search(centroid);

    // if free voxel -> check for unknown neighbors
    if (octree_node != NULL && octree_node->getLabel() == free_label_) {
      for (int i=0; i<3; i++) {
	octomap::point3d neighbor_centroid = centroid;
	for (int j=-1; j<2; j+=2) {
	  neighbor_centroid(i) += j * octree_res_;
	  octomap::OcTreeNodePCL *neighbor = octree_->search(neighbor_centroid);
	  if (neighbor != NULL && neighbor->getLabel() == unknown_label_) {
	    // add to list of border voxels
	    pcl::PointXYZ border_pt (centroid.x(), centroid.y(), centroid.z());
	    border_cloud.points.push_back(border_pt);
	    break;
	  }
	}
      }
    }
  }
  ROS_INFO("%d points in border cloud", (int)border_cloud.points.size());
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
  //ROS_DEBUG("octree min bounds [%f %f %f]", min(0), min(1), min(2));
  //ROS_DEBUG("octree max bounds [%f %f %f]", max(0), max(1), max(2));

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

    //find Leaf Nodes' centroids, assign controid coordinates to Leaf Node
    BOOST_FOREACH(octomap::OcTreeVolume vol, leaves) {
      //ROS_DEBUG("Leaf Node %d : x = %f y = %f z = %f side length = %f ", cnt++, it1->first.x(), it1->first.y(), it1->first.z(), it1->second);
      octomap::point3d centroid;
      centroid(0) = vol.first.x(), centroid(1) = vol.first.y(), centroid(2) = vol.first.z();
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


void Nbv::visualizeOctree(const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg, geometry_msgs::Point viewpoint) {
  // each array stores all cubes of a different size, one for each depth level:
  octree_marker_array_msg_.markers.resize(4);
  double lowestRes = octree_->getResolution();
  //ROS_INFO_STREAM("lowest resolution: " << lowestRes);

  std::list<octomap::OcTreeVolume> all_cells;
  //getting the cells at level 0
  octree_->getLeafNodes(all_cells, 0);
  BOOST_FOREACH(octomap::OcTreeVolume vol, all_cells)
  {
    geometry_msgs::Point cube_center;
    cube_center.x = vol.first.x();
    cube_center.y = vol.first.y();
    cube_center.z = vol.first.z();
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

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "next_best_view");
  ros::NodeHandle nh("~");
  Nbv n (nh);
  ROS_INFO("Node up and running...");
  ros::spin ();

  return (0);
}

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * \author Dejan Pangercic
 *
 * @b extract_cluster_on_table extracts euclidean clusters from pointclouds
 * of tabletop scenes
 */
// ROS core
#include <ros/ros.h>
// Messages
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
// PCL stuff
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/extract_polygonal_prism_data.h"
#include "pcl/common/common.h"
#include <pcl/io/pcd_io.h>
#include "pcl/segmentation/extract_clusters.h"
#include <pcl/features/normal_3d.h>
#include <pcl/common/angles.h>

#include <pcl_ros/publisher.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include "pcl/common/common.h"

#include "pcl_cloud_tools/GetClusters.h"
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef pcl::KdTree<Point>::Ptr KdTreePtr;

const tf::Vector3 wp_normal(1, 0, 0);
const double wp_offset = -1.45;

class ExtractClustersServer
{
public:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ExtractClustersServer (const ros::NodeHandle &nh) : nh_ (nh)
    {
      nh_.param("sac_distance", sac_distance_, 0.03);
      nh_.param("z_min_limit", z_min_limit_, 0.3);
      nh_.param("z_max_limit", z_max_limit_, 1.5);
      nh_.param("max_iter", max_iter_, 500);
      nh_.param("normal_distance_weight", normal_distance_weight_, 0.1);
      nh_.param("eps_angle", eps_angle_, 20.0);
      nh_.param("seg_prob", seg_prob_, 0.99);
      nh_.param("normal_search_radius", normal_search_radius_, 0.05);
      //what area size of the table are we looking for?
      nh_.param("rot_table_frame", rot_table_frame_, std::string("rotating_table"));
      nh_.param("object_cluster_tolerance", object_cluster_tolerance_, 0.03);
      //min 100 points
      nh_.param("object_cluster_min_size", object_cluster_min_size_, 100);
      nh_.param("k", k_, 10);
//      nh_.param("base_link_head_tilt_link_angle", base_link_head_tilt_link_angle_, 0.8);
      nh_.param("min_table_inliers", min_table_inliers_, 100);
      nh_.param("cluster_min_height", cluster_min_height_, 0.02);
      nh_.param("cluster_max_height", cluster_max_height_, 0.4);
      nh_.param("nr_cluster", nr_cluster_, 1);
      nh_.param("downsample", downsample_, true);
      nh_.param("voxel_size", voxel_size_, 0.01);
      nh_.param("save_to_files", save_to_files_, false);
      nh_.param("point_cloud_topic", point_cloud_topic, std::string("/narrow_stereo_textured/points2"));
      nh_.param("publish_token", publish_token_, false);
      nh_.param("padding", padding_, 0.85);

      service_ = nh_.advertiseService("cluster_tracking", &ExtractClustersServer::clustersCallback, this);

      cloud_pub_.advertise (nh_, "table_inliers", 1);
      //      cloud_extracted_pub_.advertise (nh_, "cloud_extracted", 1);
      cloud_objects_pub_.advertise (nh_, "cloud_objects", 10);

      vgrid_.setFilterFieldName ("z");
      vgrid_.setFilterLimits (z_min_limit_, z_max_limit_);
      if (downsample_)
        vgrid_.setLeafSize (voxel_size_, voxel_size_, voxel_size_);

      seg_.setDistanceThreshold (sac_distance_);
      seg_.setMaxIterations (max_iter_);
      seg_.setNormalDistanceWeight (normal_distance_weight_);
      seg_.setOptimizeCoefficients (true);
      seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
      seg_.setEpsAngle(pcl::deg2rad(eps_angle_));
      seg_.setMethodType (pcl::SAC_RANSAC);
      seg_.setProbability (seg_prob_);

      proj_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
      clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
      clusters_tree_->setEpsilon (1);
      normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();

      n3d_.setKSearch (k_);
      n3d_.setSearchMethod (normals_tree_);
      //plane prior (for seg_.setAxis)
      base_link_head_tilt_link_angle_ = 0.9;
    }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual ~ExtractClustersServer () 
  {
    for (size_t i = 0; i < table_coeffs_.size (); ++i) 
      delete table_coeffs_[i];
  }    

private:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool 
  clustersCallback (pcl_cloud_tools::GetClusters::Request & req,
                    pcl_cloud_tools::GetClusters::Response & res)
    {
      //sensor_msgs::PointCloud2ConstPtr cloud_in;
	  sensor_msgs::PointCloud2 cloud_in;
      cloud_in = req.input_cloud;
      //ros::topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_topic,
                                                     //  ros::Duration(5.0));
      if (cloud_in.width == 0)
        {
          res.result = false;
          return false;
        }

      //get the transform between base_link and cloud frame
      bool found_transform = tf_listener_.waitForTransform("head_tilt_link", "base_link",
                                                           cloud_in.header.stamp, ros::Duration(1.0));
      tf::StampedTransform transform;
      if (found_transform)
      {
        tf_listener_.lookupTransform("head_tilt_link", "base_link", cloud_in.header.stamp, transform);
        double yaw, pitch, roll;
        transform.getBasis().getEulerZYX(yaw, pitch, roll);
        ROS_INFO("[ExtractCluster:] Transform X: %f Y: %f Z: %f R: %f P: %f Y: %f", transform.getOrigin().getX(), 
                 transform.getOrigin().getY(), transform.getOrigin().getZ(), roll, pitch, yaw);
        base_link_head_tilt_link_angle_ = pitch;
      }
      else
      {
        ROS_INFO("[ExtractCluster:] No transform found between %s and base_link", cloud_in.header.frame_id.c_str());
        res.result = false;
        return false;
      }
      
        ROS_INFO_STREAM ("[" << getName ().c_str () << "] Received cloud: in frame " << cloud_in.header.frame_id);
      
        // Downsample + filter the input dataser
        PointCloud cloud_raw, cloud;
        pcl::fromROSMsg (cloud_in, cloud_raw);
        vgrid_.setInputCloud (boost::make_shared<PointCloud> (cloud_raw));
        vgrid_.filter (cloud);
        //cloud_pub_.publish(cloud);
        //return;
      
        // Fit a plane (the table)
        pcl::ModelCoefficients table_coeff;
        pcl::PointIndices table_inliers;
        PointCloud cloud_projected;
        pcl::PointCloud<Point> cloud_hull;
        // ---[ Estimate the point normals
        pcl::PointCloud<pcl::Normal> cloud_normals;
        n3d_.setInputCloud (boost::make_shared<PointCloud> (cloud));
        n3d_.compute (cloud_normals);
        //cloud_pub_.publish(cloud_normals);
        //return;
        cloud_normals_.reset (new pcl::PointCloud<pcl::Normal> (cloud_normals));
      
        seg_.setInputCloud (boost::make_shared<PointCloud> (cloud));
        seg_.setInputNormals (cloud_normals_);
        //z axis in Kinect frame
        btVector3 axis(0.0, 0.0, 1.0);
        //rotate axis around x in Kinect frame for an angle between base_link and head_tilt_link + 90deg
        //todo: get angle automatically
        btVector3 axis2 = axis.rotate(btVector3(1.0, 0.0, 0.0), btScalar(base_link_head_tilt_link_angle_ + pcl::deg2rad(90.0)));
        //std::cerr << "axis: " << fabs(axis2.getX()) << " " << fabs(axis2.getY()) << " " << fabs(axis2.getZ()) << std::endl;
        seg_.setAxis (Eigen::Vector3f(fabs(axis2.getX()), fabs(axis2.getY()), fabs(axis2.getZ())));
        // seg_.setIndices (boost::make_shared<pcl::PointIndices> (selection));
        seg_.segment (table_inliers, table_coeff);
        ROS_INFO ("[%s] Table model: [%f, %f, %f, %f] with %d inliers.", getName ().c_str (), 
                  table_coeff.values[0], table_coeff.values[1], table_coeff.values[2], table_coeff.values[3], (int)table_inliers.indices.size ());
        if ((int)table_inliers.indices.size () <= min_table_inliers_)
        {
          ROS_ERROR ("table has to few inliers");
          res.result = false;
          return false;
        }
        // Project the table inliers using the planar model coefficients    
        proj_.setInputCloud (boost::make_shared<PointCloud> (cloud));
        proj_.setIndices (boost::make_shared<pcl::PointIndices> (table_inliers));
        proj_.setModelCoefficients (boost::make_shared<pcl::ModelCoefficients> (table_coeff));
        proj_.filter (cloud_projected);
        //cloud_pub_.publish (cloud_projected);
      
        // Create a Convex Hull representation of the projected inliers
        chull_.setInputCloud (boost::make_shared<PointCloud> (cloud_projected));
        chull_.reconstruct (cloud_hull);
        ROS_INFO ("Convex hull has: %d data points.", (int)cloud_hull.points.size ());
        cloud_pub_.publish (cloud_hull);
       
        //pcl::PointCloud<Point> cloud_hull_padded;
        //add_remove_padding_hull(cloud_hull, cloud_hull_padded, padding_);
	//ROS_INFO ("New Convex hull has: %d data points.", (int)cloud_hull_padded.points.size ());
	//sleep(2);
        //cloud_pub_.publish (cloud_hull_padded);

        // ---[ Get the objects on top of the table
        pcl::PointIndices cloud_object_indices;
        prism_.setHeightLimits (cluster_min_height_, cluster_max_height_);
        prism_.setInputCloud (boost::make_shared<PointCloud> (cloud_raw));
        prism_.setInputPlanarHull (boost::make_shared<PointCloud>(cloud_hull));
        prism_.segment (cloud_object_indices);
        //ROS_INFO ("[%s] Number of object point indices: %d.", getName ().c_str (), (int)cloud_object_indices.indices.size ());
      
        //pcl::PointCloud<Point> cloud_object;
        //pcl::ExtractIndices<Point> extract_object_indices;
        //extract_object_indices.setInputCloud (boost::make_shared<PointCloud> (cloud));
        //extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
        //extract_object_indices.filter (cloud_object);
        //ROS_INFO ("[%s ] Publishing number of object point candidates: %d.", getName ().c_str (), 
        //        (int)cloud_objects.points.size ());
      
      
        std::vector<pcl::PointIndices> clusters;
        cluster_.setInputCloud (boost::make_shared<PointCloud>(cloud_raw));
        cluster_.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
        cluster_.setClusterTolerance (object_cluster_tolerance_);
        cluster_.setMinClusterSize (object_cluster_min_size_);
        cluster_.setSearchMethod (clusters_tree_);
        cluster_.extract (clusters);

        res.clusters_indices.clear();

        if (clusters.size() > 0)
        {
			res.clusters_indices = clusters;
			res.result = true;
        }
        else
        {
        	res.result = false;
        }

//        pcl::PointCloud<Point> cloud_object_clustered;
//        if (int(clusters.size()) >= 0)
//        {
//          for (int i = 0; i < clusters.size(); i++)
//          {
//            pcl::copyPointCloud (cloud_object, clusters[i], cloud_object_clustered);
//            cloud_objects_pub_.publish (cloud_object_clustered);
//            sensor_msgs::PointCloud2 cluster;
//            pcl::toROSMsg (cloud_object_clustered, cluster);
//            res.clusters.push_back(cluster);
//          }
//          res.result = true;
//        }
//        else
//        {
//          ROS_ERROR("Only %ld clusters found with size > %d points",
//                    clusters.size(), object_cluster_min_size_);
//          res.result = false;
//        }
        return true;
    }

  ros::NodeHandle nh_;  // Do we need to keep it?
  tf::TransformBroadcaster transform_broadcaster_;
  tf::TransformListener tf_listener_;
  bool save_to_files_, downsample_, publish_token_, got_cluster_, action_called_;

  double normal_search_radius_;
  double voxel_size_;
  double padding_;

  std::string rot_table_frame_, object_name_, point_cloud_topic;
  double object_cluster_tolerance_,  cluster_min_height_, cluster_max_height_;
  int object_cluster_min_size_, object_cluster_max_size_;

  pcl::PCDWriter pcd_writer_;
  double sac_distance_, normal_distance_weight_, z_min_limit_, z_max_limit_;
  double eps_angle_, seg_prob_, base_link_head_tilt_link_angle_;
  int k_, max_iter_, min_table_inliers_, nr_cluster_;
  
  ros::Subscriber point_cloud_sub_;

  std::vector<Eigen::Vector4d *> table_coeffs_;

  pcl_ros::Publisher<Point> cloud_pub_;
  //  pcl_ros::Publisher<Point> cloud_extracted_pub_;
  pcl_ros::Publisher<Point> cloud_objects_pub_;
  pcl_ros::Publisher<Point> token_pub_;

  // PCL objects
  //pcl::PassThrough<Point> vgrid_;                   // Filtering + downsampling object
  pcl::VoxelGrid<Point> vgrid_;                   // Filtering + downsampling object
  pcl::NormalEstimation<Point, pcl::Normal> n3d_;   //Normal estimation
  // The resultant estimated point cloud normals for \a cloud_filtered_
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;               // Planar segmentation object
  pcl::ProjectInliers<Point> proj_;               // Inlier projection object
  pcl::ExtractIndices<Point> extract_;            // Extract (too) big tables
  pcl::ConvexHull<Point> chull_;  
  pcl::ExtractPolygonalPrismData<Point> prism_;
  pcl::PointCloud<Point> cloud_objects_;
  pcl::EuclideanClusterExtraction<Point> cluster_;
  KdTreePtr clusters_tree_, normals_tree_;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get a string representation of the name of this class. */
  std::string getName () const { return ("ExtractClustersServer"); }

  ros::ServiceServer service_;
};

/* ---[ */
int
main (int argc, char** argv)
{
  ros::init (argc, argv, "extract_clusters_server");
  ros::NodeHandle nh("~");
  ExtractClustersServer clusters (nh);
  ros::spin ();
}
/* ]--- */

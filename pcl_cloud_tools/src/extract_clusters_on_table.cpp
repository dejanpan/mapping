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

//for actionlib
#include <actionlib/server/simple_action_server.h>
#include <pcl_cloud_tools/GetClustersAction.h>

#include "pcl/common/common.h"
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef pcl::KdTree<Point>::Ptr KdTreePtr;

const tf::Vector3 wp_normal(1, 0, 0);
const double wp_offset = -1.45;

class ExtractClusters 
{
public:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ExtractClusters (const ros::NodeHandle &nh) : nh_ (nh), 
                                                as_(nh_, "get_clusters", boost::bind(&ExtractClusters::executeCB, this, _1), false)
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
      //action server
      as_.start();
      //needed for the synchronization of callbacks
      got_cluster_ = action_called_ = false;
      //plane prior (for seg_.setAxis)
      base_link_head_tilt_link_angle_ = 0.9;
    }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual ~ExtractClusters () 
  {
    for (size_t i = 0; i < table_coeffs_.size (); ++i) 
      delete table_coeffs_[i];
  }    

  void 
  init (double tolerance, std::string object_name)  // tolerance: how close to (0,0) is good enough?
    {
      ROS_INFO ("[ExtractCluster:] Listening for incoming data on topic %s", nh_.resolveName (point_cloud_topic).c_str ());
      point_cloud_sub_ = nh_.subscribe (point_cloud_topic, 1,  &ExtractClusters::clustersCallback, this);
      object_name_ = object_name;
    }
  
  void add_remove_padding_hull (pcl::PointCloud<Point> &hull_in, pcl::PointCloud<Point> &hull_out, double padding)
    {
      //      hull_out.points.resize(hull_in.points.size());
      hull_out = hull_in;
      Point point_max, point_min, point_center;
      getMinMax3D (hull_in, point_min, point_max);
      //Calculate the centroid of the hull
      point_center.x = (point_max.x + point_min.x)/2;
      point_center.y = (point_max.y + point_min.y)/2;
      point_center.z = (point_max.z + point_min.z)/2;

      for (unsigned long i = 0; i < hull_in.points.size(); i++)
      {
        double dist_to_center = sqrt((point_center.x - hull_in.points[i].x) * (point_center.x - hull_in.points[i].x) +
                                     (point_center.y - hull_in.points[i].y) * (point_center.y - hull_in.points[i].y));
        ROS_INFO("[%s] Dist to center: %lf", getName().c_str (), dist_to_center);
        double angle;
        angle= atan2((hull_in.points[i].y - point_center.y), (hull_in.points[i].x - point_center.x));
        double new_dist_to_center = padding * dist_to_center;
        hull_out.points[i].y = point_center.y + sin(angle) * new_dist_to_center;
        hull_out.points[i].x = point_center.x + cos(angle) * new_dist_to_center;
      }
    }
    
  void executeCB(const pcl_cloud_tools::GetClustersGoalConstPtr &goal)
    {
      action_called_ = true;
      got_cluster_ = false;
      while (!got_cluster_)
      {
        ROS_INFO("[%s: ] Waiting to get cluster", getName().c_str ());
        sleep(1);    
      }
      bool success = true;
      // publish info to the console for the user
      ROS_INFO("[%s: ] Getting the cluster through action", getName().c_str ());

      // start executing the action
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", getName().c_str ());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
      }
      
      pcl::PointXYZ point_min;
      pcl::PointXYZ point_max;
      ias_table_msgs::TableCluster table_cluster;
      result_.clusters.clear();
      for (uint i = 0; i < cloud_object_clustered_array_.size(); i++)
      {
        pcl::getMinMax3D (cloud_object_clustered_array_[i], point_min, point_max);
        
        //get the cluster
        table_cluster.header.stamp = ros::Time::now();
        table_cluster.header.frame_id = cloud_object_clustered_array_[i].header.frame_id;
        table_cluster.header.seq = 0;

        table_cluster.center.x = (point_max.x + point_min.x)/2;
        table_cluster.center.y = (point_max.y + point_min.y)/2;
        table_cluster.center.z = (point_max.z + point_min.z)/2;
        
        table_cluster.min_bound.x = point_min.x;
        table_cluster.min_bound.y = point_min.y;
        table_cluster.min_bound.z = point_min.z;
        
        table_cluster.max_bound.x = point_max.x;
        table_cluster.max_bound.y = point_max.y;
        table_cluster.max_bound.z = point_max.z;
        result_.clusters.push_back(table_cluster);
      // publish the feedback
      //as_.publishFeedback(feedback_);
      }
      if(success)
      {

        ROS_INFO("%s: Action Succeeded", getName().c_str ());
      // set the action state to succeeded
        as_.setSucceeded(result_);
      }
      got_cluster_ = false;
      action_called_ = false;
      cloud_object_clustered_array_.clear();
    }

private:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void 
  clustersCallback (const sensor_msgs::PointCloud2ConstPtr &cloud_in)
    {
      //get the transform between base_link and cloud frame
      bool found_transform = tf_listener_.waitForTransform("head_tilt_link", "base_link",
                                                           cloud_in->header.stamp, ros::Duration(1.0));
      tf::StampedTransform transform;
      if (found_transform)
      {
        tf_listener_.lookupTransform("head_tilt_link", "base_link", cloud_in->header.stamp, transform);
        double yaw, pitch, roll;
        transform.getBasis().getEulerZYX(yaw, pitch, roll);
        ROS_INFO("[ExtractCluster:] Transform X: %f Y: %f Z: %f R: %f P: %f Y: %f", transform.getOrigin().getX(), 
                 transform.getOrigin().getY(), transform.getOrigin().getZ(), roll, pitch, yaw);
        base_link_head_tilt_link_angle_ = pitch;
      }
      else
      {
        ROS_INFO("[ExtractCluster:] No transform found between %s and base_link", cloud_in->header.frame_id.c_str());
        return;
      }
      
      if (!got_cluster_ || !action_called_)
      {
        ROS_INFO_STREAM ("[" << getName ().c_str () << "] Received cloud: in frame " << cloud_in->header.frame_id);
      
        // Downsample + filter the input dataser
        PointCloud cloud_raw, cloud;
        pcl::fromROSMsg (*cloud_in, cloud_raw);
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
          return;
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
        prism_.setInputCloud (boost::make_shared<PointCloud> (cloud));
        prism_.setInputPlanarHull (boost::make_shared<PointCloud>(cloud_hull));
        prism_.segment (cloud_object_indices);
        //ROS_INFO ("[%s] Number of object point indices: %d.", getName ().c_str (), (int)cloud_object_indices.indices.size ());
      
        pcl::PointCloud<Point> cloud_object;
        pcl::ExtractIndices<Point> extract_object_indices;
        extract_object_indices.setInputCloud (boost::make_shared<PointCloud> (cloud));
        extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
        extract_object_indices.filter (cloud_object);
        //ROS_INFO ("[%s ] Publishing number of object point candidates: %d.", getName ().c_str (), 
        //        (int)cloud_objects.points.size ());
      
      
        std::vector<pcl::PointIndices> clusters;
        cluster_.setInputCloud (boost::make_shared<PointCloud>(cloud_object));
        cluster_.setClusterTolerance (object_cluster_tolerance_);
        cluster_.setMinClusterSize (object_cluster_min_size_);
        cluster_.setSearchMethod (clusters_tree_);
        cluster_.extract (clusters);
        cloud_object_clustered_array_.clear();
        pcl::PointCloud<Point> cloud_object_clustered;
        if (int(clusters.size()) >= nr_cluster_)
        {
          for (int i = 0; i < nr_cluster_; i++)
          {
            pcl::copyPointCloud (cloud_object, clusters[i], cloud_object_clustered);
            char object_name_angle[100];
            if (save_to_files_)
            {
              sprintf (object_name_angle, "%04d",  i);
//              pcd_writer_.write (object_name_ + "_" +  object_name_angle + ".pcd", cloud_object_clustered, true);
              std::stringstream ss;
              ss << cloud_in->header.stamp;
              ROS_INFO("Saving cluster to: %s_%s.pcd", object_name_.c_str(), ss.str().c_str());
              pcd_writer_.write (object_name_ + "_" +  ss.str () + ".pcd", cloud_object_clustered, true);
            }
            cloud_objects_pub_.publish (cloud_object_clustered);
            cloud_object_clustered_array_.push_back(cloud_object_clustered);
          }
        
        
          ROS_INFO("Published %d clusters in frame: %s", nr_cluster_, cloud_object_clustered.header.frame_id.c_str());
          // cloud_objects_pub_.publish (cloud_object_clustered);
          pcl::PointCloud<Point> token;
          Point p0;
          p0.x = p0.y = p0.z  = 100.0;
          token.width = 1;
          token.height = 1;
          token.is_dense = false;
          token.points.resize(token.width * token.height);
          token.points[0] = p0;
          token.header.frame_id = cloud_object_clustered.header.frame_id;
          token.header.stamp = ros::Time::now();
          if (publish_token_)
          {
            cloud_objects_pub_.publish (token);
            ROS_INFO("Published token cluster with size %d.", token.width * token.height);
          }
        }
        else
        {
          ROS_ERROR("Only %ld clusters found with size > %d points", clusters.size(), object_cluster_min_size_);
        }
        got_cluster_ = true;
      }
    }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute the area of a 2D planar polygon patch - using a given normal
   * \param points the point cloud (planar)
   * \param normal the plane normal
   */
  double
  compute2DPolygonalArea (PointCloud &points, const std::vector<double> &normal)
    {
      int k0, k1, k2;
    
      // Find axis with largest normal component and project onto perpendicular plane
      k0 = (fabs (normal.at (0) ) > fabs (normal.at (1))) ? 0  : 1;
      k0 = (fabs (normal.at (k0)) > fabs (normal.at (2))) ? k0 : 2;
      k1 = (k0 + 1) % 3;
      k2 = (k0 + 2) % 3;
 
      // cos(theta), where theta is the angle between the polygon and the projected plane
      double ct = fabs ( normal.at (k0) );
 
      double area = 0;
      float p_i[3], p_j[3];

      for (unsigned int i = 0; i < points.points.size (); i++)
      {
        p_i[0] = points.points[i].x; p_i[1] = points.points[i].y; p_i[2] = points.points[i].z;
        int j = (i + 1) % points.points.size ();
        p_j[0] = points.points[j].x; p_j[1] = points.points[j].y; p_j[2] = points.points[j].z;

        area += p_i[k1] * p_j[k2] - p_i[k2] * p_j[k1];
      }
      area = fabs (area) / (2 * ct);

      return (area);
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
  pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;               // Planar segmentation object
  pcl::ProjectInliers<Point> proj_;               // Inlier projection object
  pcl::ExtractIndices<Point> extract_;            // Extract (too) big tables
  pcl::ConvexHull<Point> chull_;  
  pcl::ExtractPolygonalPrismData<Point> prism_;
  pcl::PointCloud<Point> cloud_objects_;
  pcl::EuclideanClusterExtraction<Point> cluster_;
  KdTreePtr clusters_tree_, normals_tree_;
  std::vector<pcl::PointCloud<Point> >cloud_object_clustered_array_;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get a string representation of the name of this class. */
  std::string getName () const { return ("ExtractClusters"); }

  //action related objects
  actionlib::SimpleActionServer<pcl_cloud_tools::GetClustersAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  pcl_cloud_tools::GetClustersFeedback feedback_;
  pcl_cloud_tools::GetClustersResult result_;
};

/* ---[ */
int
main (int argc, char** argv)
{
  ros::init (argc, argv, "extract_clusters");
  ros::NodeHandle nh("~");
  if (argc < 2)
  {
    ROS_ERROR ("usage %s <object_name>", argv[0]);
    exit(2);
  }
  std::string object_name = argv[1];
  ExtractClusters clusters (nh);
  clusters.init (5, object_name);  // 5 degrees tolerance
  ros::spin ();
}
/* ]--- */

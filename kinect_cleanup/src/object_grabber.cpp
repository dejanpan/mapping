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
  * \author Romain Thibaux, Radu Bogdan Rusu
  * \author modified: Dejan Pangercic
  * \author modified: Zoltan-Csaba Marton
  */

// ROS core
#include <ros/ros.h>
// Messages
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
#include <pcl_ros/transforms.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include "kinect_cleanup/FilterObject.h"
#include "kinect_cleanup/GrabObject.h"
#include "kinect_cleanup/GetReleasedObject.h"

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef pcl::KdTree<Point>::Ptr KdTreePtr;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ObjectGrabber 
  {
  public:
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ObjectGrabber (const ros::NodeHandle &nh) : nh_ (nh), to_select_ (false)
    {
      nh_.param("sac_distance", sac_distance_, 0.03);
      nh_.param("z_min_limit", z_min_limit_, 0.0);
      nh_.param("z_max_limit", z_max_limit_, 1.5);
      nh_.param("max_iter", max_iter_, 500);
      nh_.param("normal_distance_weight", normal_distance_weight_, 0.1);
      nh_.param("eps_angle", eps_angle_, 15.0);
      nh_.param("seg_prob", seg_prob_, 0.99);
      nh_.param("normal_search_radius", normal_search_radius_, 0.05);
      nh_.param("object_cluster_tolerance", object_cluster_tolerance_, 0.03);
      nh_.param("object_cluster_min_size", object_cluster_min_size_, 100);
      nh_.param("object_cluster_max_size", object_cluster_max_size_, 10000);
      nh_.param("k", k_, 10);
      nh_.param("base_link_head_tilt_link_angle", base_link_head_tilt_link_angle_, 0.8);
      nh_.param("min_table_inliers", min_table_inliers_, 100);
      nh_.param("cluster_min_height", cluster_min_height_, 0.01);
      nh_.param("cluster_max_height", cluster_max_height_, 0.4);

      //cloud_objects_pub_.advertise (nh_, "/moved_object", 10);
      object_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/moved_object", 10);
      object_service_ = nh_.advertiseService("/get_released_object", &ObjectGrabber::getObject, this);
      ROS_INFO ("[%s] Advertising service on: %s", getName ().c_str (), object_service_.getService ().c_str ());
      grab_service_ = nh_.advertiseService("/grab_object", &ObjectGrabber::grabDirection, this);
      ROS_INFO ("[%s] Advertising service on: %s", getName ().c_str (), object_service_.getService ().c_str ());

      vgrid_.setFilterFieldName ("z");
      vgrid_.setFilterLimits (z_min_limit_, z_max_limit_);

      seg_.setDistanceThreshold (sac_distance_);
      seg_.setMaxIterations (max_iter_);
      seg_.setNormalDistanceWeight (normal_distance_weight_);
      seg_.setOptimizeCoefficients (true);
      seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE); // TODO remove "base_link_head_tilt_link_angle" and use SACMODEL_NORMAL_PARALLEL_PLANE
      seg_.setEpsAngle(pcl::deg2rad(eps_angle_));
      seg_.setMethodType (pcl::SAC_RANSAC);
      seg_.setProbability (seg_prob_);

      proj_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
      clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
      clusters_tree_->setEpsilon (1);
      normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();

      n3d_.setKSearch (k_); // TODO use radius search
      n3d_.setSearchMethod (normals_tree_);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~ObjectGrabber ()
    {
      for (size_t i = 0; i < table_coeffs_.size (); ++i)
        delete table_coeffs_[i];
    }

  private:

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
    inputCallback (const sensor_msgs::PointCloud2ConstPtr &cloud_in)
    {
      // If nothing is pointed at
      if (!to_select_)
        return;
      // If there is no TF data available
      ros::Time time = ros::Time::now();
      tf::TransformListener tf_listener;
      bool found_transform = tf_listener.waitForTransform("right_hand", "openni_depth", time, ros::Duration(1.0));
      if (!found_transform)
        return;
      tf::StampedTransform c2h_transform;
      tf_listener.lookupTransform("right_hand", "openni_depth", time, c2h_transform);
      to_select_ = false;

      ROS_INFO_STREAM ("[" << getName ().c_str () << "] Received cloud: cloud time " << cloud_in->header.stamp);

      // Filter the input dataser
      PointCloud cloud_raw, cloud;
      pcl::fromROSMsg (*cloud_in, cloud_raw);
      vgrid_.setInputCloud (boost::make_shared<PointCloud> (cloud_raw));
      vgrid_.filter (cloud);

      // Fit a plane (the table)
      pcl::ModelCoefficients table_coeff;
      pcl::PointIndices table_inliers;
      PointCloud cloud_projected;
      pcl::PointCloud<Point> cloud_hull;

      // ---[ Estimate the point normals
      pcl::PointCloud<pcl::Normal> cloud_normals;
      n3d_.setInputCloud (boost::make_shared<PointCloud> (cloud));
      n3d_.compute (cloud_normals);
      cloud_normals_.reset (new pcl::PointCloud<pcl::Normal> (cloud_normals));

      // TODO fabs? ... use parallel to Z.cross(X)!

      //z axis in Kinect frame
      btVector3 axis(0.0, 0.0, 1.0);
      //rotate axis around x in Kinect frame for an angle between base_link and head_tilt_link + 90deg
      //todo: get angle automatically
      btVector3 axis2 = axis.rotate(btVector3(1.0, 0.0, 0.0), btScalar(base_link_head_tilt_link_angle_ + pcl::deg2rad(90.0)));
      //std::cerr << "axis: " << fabs(axis2.getX()) << " " << fabs(axis2.getY()) << " " << fabs(axis2.getZ()) << std::endl;

      seg_.setInputCloud (boost::make_shared<PointCloud> (cloud));
      seg_.setInputNormals (cloud_normals_);
      seg_.setAxis (Eigen3::Vector3f(fabs(axis2.getX()), fabs(axis2.getY()), fabs(axis2.getZ())));
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
      //ROS_INFO ("Convex hull has: %d data points.", (int)cloud_hull.points.size ());
      //cloud_pub_.publish (cloud_hull);

      // //Compute the area of the plane
      // std::vector<double> plane_normal(3);
      // plane_normal[0] = table_coeff.values[0];
      // plane_normal[1] = table_coeff.values[1];
      // plane_normal[2] = table_coeff.values[2];
      // area_ = compute2DPolygonalArea (cloud_hull, plane_normal);
      // //ROS_INFO("[%s] Plane area: %f", getName ().c_str (), area_);
      // if (area_ > (expected_table_area_ + 0.01))
      // {
      //   extract_.setInputCloud (boost::make_shared<PointCloud> (cloud));
      //   extract_.setIndices (boost::make_shared<pcl::PointIndices> (table_inliers));
      //   extract_.setNegative (true);
      //   pcl::PointCloud<Point> cloud_extracted;
      //   extract_.filter (cloud_extracted);
      //   //cloud_extracted_pub_.publish (cloud_extracted);
      //   cloud = cloud_extracted;
      // }
      //end while
      //ROS_INFO ("[%s] Publishing convex hull with: %d data points and area %lf.", getName ().c_str (), (int)cloud_hull.points.size (), area_);
      //cloud_pub_.publish (cloud_hull);

      // pcl::PointXYZRGB point_min;
      // pcl::PointXYZRGB point_max;
      // pcl::PointXYZ point_center;
      // pcl::getMinMax3D (cloud_hull, point_min, point_max);
      // //Calculate the centroid of the hull
      // point_center.x = (point_max.x + point_min.x)/2;
      // point_center.y = (point_max.y + point_min.y)/2;
      // point_center.z = (point_max.z + point_min.z)/2;

      // tf::Transform transform;
      // transform.setOrigin( tf::Vector3(point_center.x, point_center.y, point_center.z));
      // transform.setRotation( tf::Quaternion(0, 0, 0) );
      // transform_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
      //                                                           cloud_raw.header.frame_id, rot_table_frame_));

      // ---[ Get the objects on top of the table - from the raw cloud!
      pcl::PointIndices cloud_object_indices;
      prism_.setHeightLimits (cluster_min_height_, cluster_max_height_);
      prism_.setInputCloud (boost::make_shared<PointCloud> (cloud_raw));
      prism_.setInputPlanarHull (boost::make_shared<PointCloud>(cloud_hull));
      prism_.segment (cloud_object_indices);
      //ROS_INFO ("[%s] Number of object point indices: %d.", getName ().c_str (), (int)cloud_object_indices.indices.size ());

      pcl::PointCloud<Point> cloud_object;
      pcl::ExtractIndices<Point> extract_object_indices;
      //extract_object_indices.setInputCloud (cloud_all_minus_table_ptr);
      extract_object_indices.setInputCloud (boost::make_shared<PointCloud> (cloud_raw));
      extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
      extract_object_indices.filter (cloud_object);
      //ROS_INFO ("[%s ] Publishing number of object point candidates: %d.", getName ().c_str (), (int)cloud_objects.points.size ());

      std::vector<pcl::PointIndices> clusters;
      cluster_.setInputCloud (boost::make_shared<PointCloud>(cloud_object));
      cluster_.setClusterTolerance (object_cluster_tolerance_);
      cluster_.setMinClusterSize (object_cluster_min_size_);
      cluster_.setMaxClusterSize (object_cluster_max_size_);
      cluster_.setSearchMethod (clusters_tree_);
      cluster_.extract (clusters);

      // TODO take closest to ray
      for (size_t i = 0; i < clusters.size (); i++)
      {
        // compute center and see if it is close enough to ray
        int cluster_center = clusters[i].indices[clusters[i].indices.size () / 2];
        Eigen3::Vector4f pt = Eigen3::Vector4f (cloud_object.points[cluster_center].x, cloud_object.points[cluster_center].y, cloud_object.points[cluster_center].z, 0);
        Eigen3::Vector4f c = line_dir_.cross3 (line_point_ - pt); c[3] = 0;
        if (c.squaredNorm () / line_dir_squaredNorm_ > 0.1*0.1) // further then 10cm
          continue;
        // TODO make more optimal? + ERRORS/INCONSISTENCY IN PCL: second formula and in c computation!

        //tf::transformPoint();
        //Eigen3::Matrix4f transform_matrix;
        //pcl_ros::transformAsMatrix (c2h_transform, transform_matrix);

        // broadcast transform
//        tf::Transform fixed_transform;
//        fixed_transform.setOrigin (tf::Vector3(0.0, 0.0, 0.1));
//        fixed_transform.setRotation (tf::Quaternion(0, 0, 0));
//        transform_broadcaster_.sendTransform(tf::StampedTransform(fixed_transform, ros::Time::now(), "right_hand", "object_frame"));

        // transform object into right_hand frame
        pcl::PointCloud<Point> cloud_object_clustered ;
        pcl::copyPointCloud (cloud_object, clusters[i], cloud_object_clustered);
//        for (unsigned cp = 0; cp < output_cloud_.points.size (); cp++)
//        {
//          output_cloud_.points[i].x -= output_cloud_.points[0].x;
//          output_cloud_.points[i].y -= output_cloud_.points[0].y;
//          output_cloud_.points[i].z -= output_cloud_.points[0].z;
//        }
//        template <typename PointT> void transformPointCloud (const pcl::PointCloud <PointT> &cloud_in, pcl::PointCloud <PointT> &cloud_out, const tf::Transform &transform);
//        void transformPointCloud (const std::string &target_frame, const tf::Transform &net_transform, const sensor_msgs::PointCloud2 &in, sensor_msgs::PointCloud2 &out);
        sensor_msgs::PointCloud2 cluster;
        pcl::toROSMsg (cloud_object_clustered, cluster);
        pcl_ros::transformPointCloud("right_hand", (tf::Transform)c2h_transform, cluster, output_cloud_);
        //output_cloud_.header.frame_id = "right_hand"; TODO needed?
        //cloud_objects_pub_.publish (output_cloud_);
        object_pub_.publish (output_cloud_);
        ROS_INFO("Published object with %d points", (int)clusters[i].indices.size ());

        // TODO get a nicer rectangle around the object
        unsigned center_idx = cloud_object_indices.indices.at (cluster_center);
        unsigned row = center_idx / 640;
        unsigned col = center_idx - row * 640;
        ros::ServiceClient client = nh_.serviceClient<kinect_cleanup::FilterObject>("/filter_object");
        kinect_cleanup::FilterObject srv;
        srv.request.min_row = std::max (0, (int)row-50);
        srv.request.min_col = std::max (0, (int)col-50);
        srv.request.max_row = std::min (479, (int)row+50);
        srv.request.max_col = std::min (479, (int)col+50);
        srv.request.rgb = cloud_raw.points[640 * srv.request.min_row + srv.request.min_col].rgb;
        for (int i=0; i<4; i++)
          srv.request.plane_normal[i] = table_coeff.values[i];
        if (client.call(srv))
          ROS_INFO("Object filter service responded: %s", srv.response.error.c_str ());
        else
          ROS_ERROR("Failed to call object filter service!");

        // take only 1 cluster into account
        break;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool grabDirection (kinect_cleanup::GrabObject::Request  &req,
                        kinect_cleanup::GrabObject::Response &res)
    {
      to_select_ = true;
      line_point_ = Eigen3::Vector4f (req.point_line[0], req.point_line[1], req.point_line[2], 0);
      line_dir_ = Eigen3::Vector4f (req.point_line[3], req.point_line[4], req.point_line[5], 0);
      line_dir_squaredNorm_ = line_dir_.squaredNorm ();
      res.error = "grabbing enabled";
      return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool getObject (kinect_cleanup::GetReleasedObject::Request  &req,
                    kinect_cleanup::GetReleasedObject::Response &res)
    {
      //pcl::toROSMsg (output_cloud_, res.object);
      res.object = output_cloud_;
      return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the area of a 2D planar polygon patch - using a given normal
     * \param points the point cloud (planar)
     * \param normal the plane normal
     */
    /*double
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
    }*/

    // ROS stuff
    ros::NodeHandle nh_;
    tf::TransformBroadcaster transform_broadcaster_;
    tf::TransformListener tf_listener_;
    ros::Subscriber point_cloud_sub_;
    //pcl_ros::Publisher<Point> cloud_objects_pub_;
    ros::Publisher object_pub_;
    ros::ServiceServer object_service_;
    ros::ServiceServer grab_service_;

    // Switch to enable segmenting
    bool to_select_;
    Eigen3::Vector4f line_point_;
    Eigen3::Vector4f line_dir_;
    float line_dir_squaredNorm_;

    // Parameters
    double object_cluster_tolerance_, cluster_min_height_, cluster_max_height_;
    int object_cluster_min_size_, object_cluster_max_size_;
    double sac_distance_, normal_distance_weight_, z_min_limit_, z_max_limit_;
    double eps_angle_, seg_prob_, base_link_head_tilt_link_angle_;
    int k_, max_iter_, min_table_inliers_;
    double normal_search_radius_;
    
    // PCL objects
    pcl::PassThrough<Point> vgrid_;                   // Filtering object
    pcl::NormalEstimation<Point, pcl::Normal> n3d_;   //Normal estimation
    pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;               // Planar segmentation object
    pcl::ProjectInliers<Point> proj_;               // Inlier projection object
    pcl::ExtractIndices<Point> extract_;            // Extract (too) big tables
    pcl::ConvexHull2D<Point> chull_;
    pcl::ExtractPolygonalPrismData<Point> prism_;
    pcl::PointCloud<Point> cloud_objects_;
    pcl::EuclideanClusterExtraction<Point> cluster_;
    KdTreePtr clusters_tree_, normals_tree_;

    // Grabbed object in right_hand frame
    //pcl::PointCloud<Point> output_cloud_;
    sensor_msgs::PointCloud2 output_cloud_;
  
    // TODO why are these saved as fields?
    std::vector<Eigen3::Vector4d *> table_coeffs_;
    pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_; // The resultant estimated point cloud normals for \a cloud_filtered_

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get a string representation of the name of this class. */
    std::string getName () const { return ("ObjectGrabber"); }
};

/* ---[ */
int
main (int argc, char** argv)
{
  ros::init (argc, argv, "object_grabber");
  ros::NodeHandle nh("~");
  ObjectGrabber ptu_calibrator (nh);
  ros::spin ();

  return 0;
}
/* ]--- */

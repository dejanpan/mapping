/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: transform_pointcloud.cpp 30719 2010-07-09 20:28:41Z rusu $
 *
 */

/**

\author Dejan Pangercic

@b Detects handles based on Kinect's nan data.

**/

// ROS core
#include <ros/ros.h>

//pcl
#include "pcl/common/common.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/features/normal_3d.h>
#include <tf/transform_listener.h>
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/filters/extract_indices.h"
#include <pcl/ros/conversions.h>
#include <pcl/common/angles.h>
#include "pcl/segmentation/extract_clusters.h"
#include <pcl/surface/convex_hull.h>
#include "pcl_ros/transforms.h"
//pcl_ros
#include <pcl_ros/publisher.h>
//opencv
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//bullet
#include "LinearMath/btVector3.h"
//3D-2D projection
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
//for marker pose estimation
#include "visualization_msgs/Marker.h"
//local service
#include <handle_detection2D/getHandlesNAN.h>

//for find function
#include <algorithm>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef pcl::KdTree<PointT>::Ptr KdTreePtr;

//for find function
#include <algorithm>
using namespace std;
using namespace cv;
class HandleDetectorNANNode
{
protected:
  ros::NodeHandle nh_;

public:
  string output_cloud_topic_, input_cloud_topic_, output_marker_topic_;

  ros::Subscriber sub_;
  ros::ServiceServer service_;
  ros::Publisher marker_pub_;
  pcl_ros::Publisher<PointT> pub_;
  sensor_msgs::PointCloud2 output_cloud_;

  Mat nan_image, dst, canny_img, roi_image;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr biggest_plane_cloud_;
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n3d_;   //Normal estimation
  KdTreePtr normals_tree_, clusters_tree_;
  pcl::KdTree<pcl::PointXYZ>::Ptr handles_tree_;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_;               // Planar segmentation object
  pcl::EuclideanClusterExtraction<PointT> cluster_;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> h_cluster_;
  double eps_angle_, seg_prob_, sac_distance_, normal_distance_weight_, base_link_kinect_head_rgb_optical_frame_;
  double handle_length_, x_handle_position_offset_, z_handle_position_;
  int k_, max_iter_, min_table_inliers_, nr_cluster_;
  double plane_cluster_tolerance_;
  int plane_cluster_min_size_, plane_cluster_max_size_;
  tf::TransformListener tf_listener_;
  bool save_image_, save_cloud_;
  sensor_msgs::CameraInfoConstPtr cam_info;
  sensor_msgs::ImageConstPtr image_ptr;
  image_geometry::PinholeCameraModel cam_model_;
  pcl::PCDWriter pcd_writer_;
  pcl::ConvexHull<pcl::PointXYZ> chull_;
  std::string kinect_rgb_optical_frame_;
  ////////////////////////////////////////////////////////////////////////////////
  HandleDetectorNANNode  (ros::NodeHandle &n) : nh_(n)
  {
    // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
    nh_.param("input_cloud_topic", input_cloud_topic_, std::string("/camera/rgb/points"));
    output_cloud_topic_ = input_cloud_topic_ + "_plane";
//    nh_.param("to_frame", to_frame_, std::string("base_link"));
    
    nh_.param("output_marker_topic", output_marker_topic_, std::string("handle_centroid"));
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(output_marker_topic_, 0 );

    //    sub_ = nh_.subscribe (input_cloud_topic_, 1,  &HandleDetectorNANNode::cloud_cb, this);
    service_ = nh_.advertiseService("get_handles_nan", &HandleDetectorNANNode::cloud_cb, this);
    ROS_INFO ("[%s:] Listening for incoming data on topic %s", getName ().c_str (), nh_.resolveName (input_cloud_topic_).c_str ());
    pub_.advertise (nh_, output_cloud_topic_, 1);
    ROS_INFO ("[%s:] Will be publishing data on topic %s.", getName ().c_str (), nh_.resolveName (output_cloud_topic_).c_str ());
    nan_image = Mat::zeros (480, 640, CV_8UC1);
    roi_image = Mat::zeros (480, 640, CV_8UC1);
    pcl_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    biggest_plane_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    //3D handle length
    nh_.param("handle_length", handle_length_, 0.1);
    //for normal search
    nh_.param("k", k_, 30);
    nh_.param("save_image", save_image_, false);
    nh_.param("save_cloud", save_cloud_, false);
    normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointT> > ();
    n3d_.setKSearch (k_);
    n3d_.setSearchMethod (normals_tree_);
    //plane extraction
    nh_.param("sac_distance", sac_distance_, 0.02);
    nh_.param("normal_distance_weight", normal_distance_weight_, 0.05);
    nh_.param("max_iter", max_iter_, 500);
    nh_.param("eps_angle", eps_angle_, 15.0);
    nh_.param("seg_prob", seg_prob_, 0.99);
    nh_.param("min_table_inliers", min_table_inliers_, 15000);
    nh_.param("nr_cluster", nr_cluster_, 1);
    seg_.setDistanceThreshold (sac_distance_);
    seg_.setMaxIterations (max_iter_);
    seg_.setNormalDistanceWeight (normal_distance_weight_);
    seg_.setOptimizeCoefficients (true);
    seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg_.setEpsAngle(pcl::deg2rad(eps_angle_));
    seg_.setMethodType (pcl::SAC_RANSAC);
    seg_.setProbability (seg_prob_);
    //clustering of planes
    nh_.param("plane_cluster_tolerance", plane_cluster_tolerance_, 0.03);
    nh_.param("plane_cluster_min_size", plane_cluster_min_size_, 200);
    cluster_.setClusterTolerance (plane_cluster_tolerance_);
    cluster_.setSpatialLocator(0);
    cluster_.setMinClusterSize (plane_cluster_min_size_);
    //    cluster_.setMaxClusterSize (object_cluster_max_size_);
    clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointT> > ();
    clusters_tree_->setEpsilon (1);
    cluster_.setSearchMethod (clusters_tree_);

    //clustering of handles in NAN cloud
    handles_tree_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
    handles_tree_->setEpsilon (1);
    h_cluster_.setSearchMethod (handles_tree_);
    h_cluster_.setClusterTolerance (0.02);
    h_cluster_.setSpatialLocator(0);
    h_cluster_.setMinClusterSize (500);
    h_cluster_.setMaxClusterSize (4000);

    //ADA handle offset from the supporting plane
    nh_.param("x_handle_position_offset", x_handle_position_offset_, 0.03);
    //expected height of handle in base_link
    nh_.param("z_handle_position", z_handle_position_, 0.75);
    nh_.param("kinect_rgb_optical_frame", kinect_rgb_optical_frame_, std::string("/openni_rgb_optical_frame"));
  }

  ////////////////////////////////////////////////////////////////////////////////
  // lineLength2D ()
  double lineLength2D (int x1, int y1, int x2, int y2)
    {
      double dist  = sqrt ((x1-x2) * (x1-x2) + (y1-y2) * (y1-y2));
      return dist;
    }
  double lineLength2D (double x1, double y1, double x2, double y2)
    {
      double dist  = sqrt ((x1-x2) * (x1-x2) + (y1-y2) * (y1-y2));
      return dist;
    }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get a string representation of the name of this class. */
  std::string getName () const { return ("HandleDetectorNAN"); }

  int findBiggestPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, Rect & roi, pcl::PointIndices & indices_biggest_plane)
  {
    //transform point cloud into base_link
    bool found_transform = tf_listener_.waitForTransform(pcl_cloud->header.frame_id, "base_link",
							 pcl_cloud->header.stamp, ros::Duration(1.0));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZRGB>());
    if (found_transform)
    {
      //ROS_ASSERT_MSG(found_transform, "Could not transform to camera frame");
      tf::StampedTransform transform;
      tf_listener_.lookupTransform("base_link", pcl_cloud->header.frame_id, pcl_cloud->header.stamp, transform);
      pcl_ros::transformPointCloud(*pcl_cloud, *cloud_temp, transform);
      pcl_cloud = cloud_temp;
      pcl_cloud->header.frame_id = "base_link";
      pcl_cloud->header.stamp = ros::Time::now();
    }
    else
      {
        ROS_INFO("[%s:] No transform found between %s and base_link", getName().c_str(), pcl_cloud->header.frame_id.c_str());
        return -1;
      }
      //Estimate Point Normals
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
      n3d_.setInputCloud (pcl_cloud);
      n3d_.compute (*cloud_normals);

      //find transform between base_link and kinect_head_rgb_optical_frame frames
      // bool found_transform = tf_listener_.waitForTransform("kinect_head_rgb_optical_frame", "base_link",
      //                                                      pcl_cloud->header.stamp, ros::Duration(1.0));
      // tf::StampedTransform transform;
      // if (found_transform)
      // {
      //   tf_listener_.lookupTransform("kinect_head_rgb_optical_frame", "base_link", pcl_cloud->header.stamp, transform);
      //   double yaw, pitch, roll;
      //   transform.getBasis().getEulerZYX(yaw, pitch, roll);
      //   ROS_INFO("[%s:] Transform X: %f Y: %f Z: %f R: %f P: %f Y: %f", getName().c_str(), transform.getOrigin().getX(), 
      //            transform.getOrigin().getY(), transform.getOrigin().getZ(), roll, pitch, yaw);
      //   base_link_kinect_head_rgb_optical_frame_ = pitch;
      // }
      // else
      // {
      //   ROS_INFO("[%s:] No transform found between %s and base_link", getName().c_str(), pcl_cloud->header.frame_id.c_str());
      //   return -1;
      // }

      //Segment the biggest furniture_face plane
      //z axis in Kinect frame
      btVector3 axis(1.0, 0.0, 0.0);
      //rotate axis around x in Kinect frame for an angle between base_link and kinect_head_rgb_optical_frame
      //btVector3 axis2 = axis.rotate(btVector3(1.0, 0.0, 0.0), -base_link_kinect_head_rgb_optical_frame_);
      //ROS_INFO_STREAM("[" << getName() << ":]" <<" axis: " << axis2.getX() << " " << axis2.getY() << " " 
      //              << axis2.getZ());
      seg_.setAxis (Eigen::Vector3f(axis.getX(), axis.getY(), axis.getZ()));
      pcl::ModelCoefficients::Ptr table_coeff (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr table_inliers (new pcl::PointIndices ());
      seg_.setInputCloud (pcl_cloud);
      seg_.setInputNormals (cloud_normals);
      seg_.segment (*table_inliers, *table_coeff);
      ROS_INFO ("[%s] Cabinet face model: [%f, %f, %f, %f] with %d inliers.", getName ().c_str (), 
                table_coeff->values[0], table_coeff->values[1], table_coeff->values[2], table_coeff->values[3], 
                (int)table_inliers->indices.size ());

      if ((int)table_inliers->indices.size () <= min_table_inliers_)
      {
        ROS_ERROR ("[%s:] Cabinet face has to few inliers", getName().c_str());
        return -1;
      }

      //Extract the biggest cluster correponding to above inliers
      std::vector<pcl::PointIndices> clusters;
      cluster_.setInputCloud (pcl_cloud);
      cluster_.setIndices(table_inliers);
      cluster_.extract (clusters);

      ROS_INFO("[%s:] Found clusters %ld ", getName().c_str(), clusters.size());
      PointCloudPtr biggest_face (new PointCloud());
      if (int(clusters.size()) >= nr_cluster_)
      {
        for (int i = 0; i < nr_cluster_; i++)
        {
          pcl::copyPointCloud (*pcl_cloud, clusters[i], *biggest_face);
	  indices_biggest_plane = clusters[i];
        }
      }
      else
      {
        ROS_ERROR("[%s:] Only %ld clusters found with size > %d points", getName().c_str(), clusters.size(), 
                  plane_cluster_min_size_);
        return -1;
      }
      ROS_INFO("[%s:] Found biggest face with %ld points", getName().c_str(), biggest_face->points.size());

      biggest_plane_cloud_->header = biggest_face->header = pcl_cloud->header;
      biggest_plane_cloud_ = biggest_face;

//       //getMinMax
//       double min_x, min_y, max_x, max_y;
//       // min_x = min_y = numeric_limits<double>::max();
//       min_x = min_y = 1000.0;
//       max_x = max_y = -1000.0;
      
//       size_t in_min_x, in_min_y, in_max_x, in_max_y;
//       in_min_x = in_min_y = in_max_x = in_max_y = 0;
//       for (size_t i = 0; i < clusters[0].indices.size(); ++i)
//       {
//         if (pcl_cloud->points[clusters[0].indices[i]].x > max_x)
//         {
//           max_x = pcl_cloud->points[clusters[0].indices[i]].x;
//           in_max_x = clusters[0].indices[i];
//         }
//         if (pcl_cloud->points[clusters[0].indices[i]].x < min_x)
//         {
//           min_x = pcl_cloud->points[clusters[0].indices[i]].x;
//           in_min_x = clusters[0].indices[i];
//         }
//         if (pcl_cloud->points[clusters[0].indices[i]].y < min_y)
//         {
//           min_y = pcl_cloud->points[clusters[0].indices[i]].y;
//           in_min_y = clusters[0].indices[i];
//         }
//         if (pcl_cloud->points[clusters[0].indices[i]].y > max_y)
//         {
//           max_y = pcl_cloud->points[clusters[0].indices[i]].y;
//           in_max_y = clusters[0].indices[i];
//         }
//       }
//       ROS_INFO_STREAM(getName() << " min x, y: " << min_x << " " << min_y);
//       ROS_INFO_STREAM(getName() << " max x, y: " << max_x << " " << max_y);


//       ROS_INFO_STREAM(getName() << " in min x, y: " << in_min_x << " " << in_min_y);
//       ROS_INFO_STREAM(getName() << " in max x, y: " << in_max_x << " " << in_max_y);

      
//       int x1 = 0;
//       int y1 = 0;
//       int x2 = 0;
//       int y2 = 0;
//       for (size_t i = 0; i <  pcl_cloud->height; ++i)
//       {
//         // go over rows          
//         for (size_t j = 0; j < pcl_cloud->width; ++j)
//         {
//           if ((i * 640 + j) == in_min_x) 
//             x1 = j;
//           if ((i * 640 + j) == in_min_y) 
//             y1 = i;
//           if ((i * 640 + j) == in_max_x) 
//             x2 = j;
//           if ((i * 640 + j) == in_max_y) 
//             y2 = i;
//         }
//       }
// //      ROS_INFO_STREAM(getName() << " coordinates x1, y1, x2, y2: " << x1 << " " << y1 << " " << x2 << " " << y2);
//       roi.x = x1;
//       roi.y = y1;
//       roi.width = x2 - x1;
//       roi.height = y2 - y1;
      //For Debug
      pub_.publish(*biggest_face);
      ROS_INFO_STREAM(getName() << " Published biggest face with " << biggest_face->points.size() << " points.");
      return 0;  
    }

  ////////////////////////////////////////////////////////////////////////////////
  // project3DPointsToPixels (!)
  void project3DPointsToPixels (std::vector <cv::Point2d> & projected_points)
  {
    projected_points.resize(0);
    //project point cloud to image
    cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/kinect_head/camera/rgb/camera_info");
    cam_model_.fromCameraInfo(cam_info);

    for (unsigned long i = 0; i < biggest_plane_cloud_->points.size(); i++)
      {
	cv::Point3d pt_cv(biggest_plane_cloud_->points[i].x, biggest_plane_cloud_->points[i].y,
			  biggest_plane_cloud_->points[i].z);
	cv::Point2d uv;
	cam_model_.project3dToPixel(pt_cv, uv);
	projected_points.push_back(uv);
	ROS_DEBUG_STREAM("points projected: " << round(uv.x) << " " << round(uv.y));
      }
  }
  ////////////////////////////////////////////////////////////////////////////////
  // drawNANs (!)
  void drawNANs(cv::Mat & nan_image, const Rect roi, double padding = 0.0)
  {
      // go over cols
    for (size_t i = 0; i <  pcl_cloud_->height; i++)
    {
      // go over rows          
      for (size_t j = 0; j < pcl_cloud_->width; j++)
      {
        if (isnan (pcl_cloud_->points[i * 640 + j].x) ||
            isnan (pcl_cloud_->points[i * 640 + j].y) ||
            isnan (pcl_cloud_->points[i * 640 + j].z))
        {
          // if ((int)j > (roi.x + roi.width * padding) && (int)j < (roi.x + roi.width) && 
	  //     (int)i > (roi.y - roi.width * padding) && (int)i < (roi.y + roi.height))
	    nan_image.at <uint8_t>(i, j) = 255;
        }
      }
    }
  }

 void drawROI(cv::Mat & nan_image, pcl::PointIndices & roi_indices)
  {
      // go over cols
    for (size_t i = 0; i <  pcl_cloud_->height; i++)
    {
      // go over rows          
      for (size_t j = 0; j < pcl_cloud_->width; j++)
	{
	  vector<int>::iterator it = find(roi_indices.indices.begin(), roi_indices.indices.end(), i * 640 + j);
	    if (it != roi_indices.indices.end()) 
	   {
	      // found it
	      //if (isnan (pcl_cloud_->points[i * 640 + j].x) ||
		  // isnan (pcl_cloud_->points[i * 640 + j].y) ||
		  // isnan (pcl_cloud_->points[i * 640 + j].z))
		//	{
		  // if ((int)j > (roi.x + roi.width * padding) && (int)j < (roi.x + roi.width) && 
		  //     (int)i > (roi.y - roi.width * padding) && (int)i < (roi.y + roi.height))
		  nan_image.at <uint8_t>(i, j) = 255;
		  //	}
	      } 
	       else 
	       {
	      //does not exist
	      continue;
	       }
      }
    }
  }

  void findLines (cv::Mat & nan_image, vector<Vec4i> & lines)
  {
    int min_line_length = 30;
    int max_line_gap = 10;
    HoughLinesP(nan_image, lines, 1, CV_PI/180, 100, min_line_length, max_line_gap );
  }

    void erodeDilate (cv::Mat & nan_image)
  {
    int dilate_iter = 5;
    int erode_iter = 3;
    dilate(nan_image, nan_image, Mat(), Point(-1,-1), dilate_iter);
    erode(nan_image, nan_image, Mat(), Point(-1,-1), erode_iter);
    dilate(nan_image, nan_image, Mat(), Point(-1,-1), dilate_iter);
    erode(nan_image, nan_image, Mat(), Point(-1,-1), erode_iter);
  }

  void drawLines(const cv::Mat & nan_image, cv::Mat & dst, const vector<Vec4i> & lines, int max_line_length)
  {
    cvtColor(nan_image, dst, CV_GRAY2BGR);
    for( size_t i = 0; i < lines.size(); i++ )
      {
	Vec4i l = lines[i];
	float length = lineLength2D (l[0], l[1], l[2], l[3]);
	//float angle = atan2 (l[3] - l[1], l[2] - l[0]) * 180 / CV_PI;
	//if (length < 180 && (((80.0 < angle) && (angle < 110.0)) || ((-1.0 < angle) && (angle < 1.0))))
	//{
	cv::Scalar color(std::rand() % 256, std::rand() % 256, std::rand() % 256);
        if (length < max_line_length)
          line(dst, Point(l[0], l[1]), Point(l[2], l[3]), color, 3, CV_AA);
        //TODO:
        //calculate orientation and check for it
        //cerr << "angle: " << angle << endl;
        //cerr << "length: " << length << endl;
        //}
        //TODO: cluster lines and compute average x and y
      }
  }

  ////////////////////////////////////////////////////////////////////////////////
  // drawProjectedPoints (!)
  void drawProjectedPoints(std::vector <cv::Point2d> & projected_points, cv::Mat & nan_image, cv::Mat & dst)
  {
    cvtColor(nan_image, dst, CV_GRAY2BGR);
    for (uint i = 0; i < projected_points.size(); i++)
      circle(dst, cvPoint(projected_points[i].x,  projected_points[i].y), 5, cvScalar(0, 255, 0));
  }

  ////////////////////////////////////////////////////////////////////////////////
  // createNaNPointCloud (!)
  void createNaNPointCloud (cv::Mat & nan_image, pcl::PointCloud<pcl::PointXYZ>::Ptr nan_cloud)
  {
    for (int i = 0; i < nan_image.rows; i++)
      for (int j = 0; j < nan_image.cols; j++)
	{
	  if (nan_image.at<uint8_t>(i, j) == 255)
	    {
	      pcl::PointXYZ p;
	      p.x = (double)j/1000.0;
	      p.y = (double)i/1000.0;
	      p.z = 0.0;
	      nan_cloud->points.push_back(p);
	    }
	}
  }

    ////////////////////////////////////////////////////////////////////////////////
  // createNaNPointCloud (!)
  void findHandlesInNanPointCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr nan_cloud, std::vector<pcl::PointIndices> & handle_clusters)
  {
    h_cluster_.setInputCloud (nan_cloud);
    //      cluster_.setIndices(table_inliers);
    h_cluster_.extract (handle_clusters);
    ROS_INFO("%s Found %ld handle candidates", getName().c_str(), handle_clusters.size());
  }
    
  ////////////////////////////////////////////////////////////////////////////////
  // saveHandleClusters (!)
  void saveHandleClusters (std::vector<pcl::PointIndices> & handle_clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr nan_cloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    for (uint i = 0; i < handle_clusters.size(); i++)
      {
	pcl::copyPointCloud (*nan_cloud, handle_clusters[i], *cluster_cloud);
	std::stringstream ss;
	ss << ros::Time::now() << ".pcd";
	ROS_INFO ("[%s:] Cluster cloud saved to %s with points %ld", getName().c_str(), ss.str ().c_str (), cluster_cloud->points.size());
	pcd_writer_.write(ss.str (), *cluster_cloud);
	cluster_cloud->points.clear();
	sleep(0.5);
      }
  }

  ////////////////////////////////////////////////////////////////////////////////
  // recoverCentroidPose (!)
  void recoverCentroidPose(std::vector<pcl::PointIndices> & handle_clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr nan_cloud,
			   std::vector<geometry_msgs::PointStamped> & vector_point_avg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_hull;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    //chull_.setInputCloud (boost::make_shared<PointCloud> (cloud_projected));
    int count = 0;
    int count_hull_points = 0;
    //    std::vector<geometry_msgs::PointStamped> vector_point_avg;
    geometry_msgs::PointStamped point_avg;
    for (uint i = 0; i < handle_clusters.size(); i++)
      {
	//	chull_.setIndices (boost::make_shared<pcl::PointIndices> (handle_clusters[i]));
	pcl::copyPointCloud (*nan_cloud, handle_clusters[i], *cluster_cloud);
	chull_.setInputCloud(cluster_cloud);
	chull_.reconstruct (cloud_hull);
	point_avg.point.x = point_avg.point.y = point_avg.point.z = 0.0;
	cerr << "hull: " << count << endl;
	for (uint j = 0; j < cloud_hull.points.size(); j++)
	  {
	    // cerr << "x, y: " << cloud_hull.points[j].x * 1000 << " " << cloud_hull.points[j].y * 1000 << endl;
	    // cerr << "p.x: " << pcl_cloud_->at(cloud_hull.points[j].x * 1000, cloud_hull.points[j].y * 1000).x << " ";
	    // cerr << " p.y: " << pcl_cloud_->at(cloud_hull.points[j].x * 1000, cloud_hull.points[j].y * 1000).y << " ";
	    // cerr << " p.z: " << pcl_cloud_->at(cloud_hull.points[j].x * 1000, cloud_hull.points[j].y * 1000).z << endl;
	    if (!isnan (pcl_cloud_->at(cloud_hull.points[j].x * 1000, cloud_hull.points[j].y * 1000).x) ||
		!isnan (pcl_cloud_->at(cloud_hull.points[j].x * 1000, cloud_hull.points[j].y * 1000).y) ||
		!isnan (pcl_cloud_->at(cloud_hull.points[j].x * 1000, cloud_hull.points[j].y * 1000).x))
	      {
		point_avg.point.x = point_avg.point.x + pcl_cloud_->at(cloud_hull.points[j].x * 1000, cloud_hull.points[j].y * 1000).x;
		point_avg.point.y = point_avg.point.y + pcl_cloud_->at(cloud_hull.points[j].x * 1000, cloud_hull.points[j].y * 1000).y;
		point_avg.point.z = point_avg.point.z + pcl_cloud_->at(cloud_hull.points[j].x * 1000, cloud_hull.points[j].y * 1000).z;
		count_hull_points++;
	      }
	  }
	if (count_hull_points != 0)
	  {
	    point_avg.point.x = point_avg.point.x/(double)count_hull_points;
	    point_avg.point.y = point_avg.point.y/(double)count_hull_points;
	    point_avg.point.z = point_avg.point.z/(double)count_hull_points;
	    count_hull_points = 0;
	    point_avg.header.frame_id = pcl_cloud_->header.frame_id;
	    point_avg.header.stamp = ros::Time::now();
	    vector_point_avg.push_back(point_avg);
	  }
	// std::stringstream ss;
	// ss << "hull_" << count << ".pcd";
	// ROS_INFO ("[%s:] Hull cloud saved to %s", getName().c_str(), ss.str ().c_str ());
	// pcd_writer_.write(ss.str (), cloud_hull);
	// cloud_hull.points.clear();
	// cluster_cloud->points.clear();
	count++;
      }
    //    publishMarker (vector_point_avg);
  }

  void publishMarker (std::vector<geometry_msgs::PointStamped> & vector_point_avg)
  {
    visualization_msgs::Marker marker;
    for (uint i = 0; i < vector_point_avg.size(); i++)
      {
	marker.header.frame_id = vector_point_avg[i].header.frame_id;
	marker.header.stamp = ros::Time::now();
	marker.ns = "HandleCentroid";
	marker.id = i;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = vector_point_avg[i].point.x;
	marker.pose.position.y = vector_point_avg[i].point.y;
	marker.pose.position.z = vector_point_avg[i].point.z;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 1;
	marker.scale.x = .1;
	marker.scale.y = .1;
	marker.scale.z = .1;
	marker.color.a = 0.5;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	std::cerr << getName() << " MARKER COMPUTED, WITH FRAME " << marker.header.frame_id << std::endl;
	std::cerr << getName() << " Handle position " << vector_point_avg[i].point.x << " " << vector_point_avg[i].point.y
		  << " " << vector_point_avg[i].point.z << std::endl;
	marker_pub_.publish(marker);
      }
  }

  ////////////////////////////////////////////////////////////////////////////////
  // getHandleImageLength (!)
  void getHandleImageLength (std::vector<geometry_msgs::PointStamped> & vector_point_avg,     
			     std::vector<double> & expected_handle_image_length)
  {
    cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/kinect_head/camera/rgb/camera_info");
    cam_model_.fromCameraInfo(cam_info);

    for (unsigned long i = 0; i < vector_point_avg.size(); i++)
      {
	cv::Point3d pt_cv1(0, handle_length_, vector_point_avg[i].point.z);
	cv::Point3d pt_cv2(0, 0.0, vector_point_avg[i].point.z);
	cv::Point2d uv1, uv2;
	cam_model_.project3dToPixel(pt_cv1, uv1);
	cam_model_.project3dToPixel(pt_cv2, uv2);
	cerr << getName() << "Z: " << vector_point_avg[i].point.z <<  " Line length expected: " << lineLength2D(uv1.x, uv1.y, uv2.x, uv2.y) << endl;
	expected_handle_image_length.push_back(lineLength2D(uv1.x, uv1.y, uv2.x, uv2.y));
      }
  }

    ////////////////////////////////////////////////////////////////////////////////
    // rejectFalsePositives (!)
  void rejectFalsePositives (std::vector<geometry_msgs::PointStamped> & vector_point_avg)
  {
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // for (uint i = 0; i < handle_clusters.size(); i++)
    //   {
    // 	//	chull_.setIndices (boost::make_shared<pcl::PointIndices> (handle_clusters[i]));
    // 	pcl::copyPointCloud (*nan_cloud, handle_clusters[i], *cluster_cloud);
    // 	pcl::PointXYZ point_max, point_min;
    // 	getMinMax3D (*cluster_cloud, point_min, point_max);
    // 	point_min.x = point_max.x = 0.0;
    // 	cerr << getName() << " point_min: " << point_min.x << " " << point_min.y << endl;
    // 	cerr << getName() << " point_max: " << point_max.x << " " << point_max.y << endl;
    // 	cerr << getName() << " Line length actual: " << 1000 * lineLength2D(point_min.x, point_min.y, point_max.x, point_max.y) << endl;
    // 	double min_allowed_len = 1000 * lineLength2D(point_min.x, point_min.y, point_max.x, point_max.y) * 0.7;
    // 	double max_allowed_len = 1000 * lineLength2D(point_min.x, point_min.y, point_max.x, point_max.y) * 1.3;
	  
    // 	if (expected_handle_image_length[i] > min_allowed_len && expected_handle_image_length[i] < max_allowed_len)
    // 	  {
    // 	    true_pos.push_back(i); 
    // 	  }
    //   }
    //substract the ADS offset and filter out the poses with the wrong Z
    std::vector<geometry_msgs::PointStamped> temp = vector_point_avg;
    vector_point_avg.clear();
    for (uint i = 0; i < temp.size(); i++)
      {
	if (((z_handle_position_ - 0.1) < temp[i].point.z) && ((z_handle_position_ + 0.1) > temp[i].point.z ))
	  {
	    temp[i].point.x -= x_handle_position_offset_;
	    vector_point_avg.push_back (temp[i]);
	  }
      }
  }

  void transformPoints (std::vector<geometry_msgs::PointStamped> & vector_point_avg, std::string frame="base_link")
  {
    for (uint i = 0; i < vector_point_avg.size(); i++)
      {
	bool found_transform = tf_listener_.waitForTransform(vector_point_avg[i].header.frame_id, frame,
						    vector_point_avg[i].header.stamp, ros::Duration(10.0));
	if (found_transform)
	  {
	    tf_listener_.transformPoint(frame, vector_point_avg[i], vector_point_avg[i]);
	  }
	else
	  {
	    ROS_ERROR("No transform for point %d", i);
	    return;
	  }
      }
  }
    
  ////////////////////////////////////////////////////////////////////////////////
  // cloud_cb (!)
  //  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pc)
  bool cloud_cb (handle_detection2D::getHandlesNAN::Request & req, 
		 handle_detection2D::getHandlesNAN::Response & res)
  {
    sensor_msgs::PointCloud2ConstPtr pc;
    pc = ros::topic::waitForMessage<sensor_msgs::PointCloud2> ("/kinect_head/camera/rgb/points");
    pcl::fromROSMsg(*pc, *pcl_cloud_);
    ROS_INFO_STREAM("[" << getName ().c_str () << ":] Got PC with height: width: " << pcl_cloud_->height << " " 
                    <<  pcl_cloud_->width << " frame: " << pcl_cloud_->header.frame_id);
    
    //image of nans
    nan_image = Mat::zeros (480, 640, CV_8UC1);
    //find roi from 3D plane
    Rect roi;

    pcl::PointIndices indices_biggest_plane;
    if (findBiggestPlane(pcl_cloud_, roi, indices_biggest_plane) < 0)
      return false;
    
    ROS_INFO_STREAM(getName() << " indices_biggest_plane with size:  " << indices_biggest_plane.indices.size());
    // ROS_INFO_STREAM(getName() << " coordinates x1, y1, x2, y2: " << roi.x << " " << roi.y << " " 
    //                 << roi.x + roi.width << " " << roi.y + roi.height);

    //project points to 3D
    std::vector <cv::Point2d> projected_points;
    //    project3DPointsToPixels (projected_points);

    //draw projected points
    // drawProjectedPoints (projected_points, nan_image, dst);

    //TODO: find convex contour and draw nan points only inside of it

    //TODO: average over 10 measurements

    //draw NANs on the image
    drawNANs(nan_image, roi);
    drawROI(roi_image, indices_biggest_plane);

    //drawHandles
    
    // circle(dst, cvPoint(roi.x,  roi.y), 7, cvScalar(255, 0, 0));
    // circle(dst, cvPoint(roi.x + roi.width,  roi.y+roi.height), 7, cvScalar(255, 0, 0));

    erodeDilate (nan_image);
    erodeDilate (roi_image);
    cv::Mat intersection;
    cv::bitwise_and(nan_image, roi_image, intersection);
    //     imshow("roi_image", roi_image);
    // waitKey();
    // imshow("nan_image", nan_image);
    //    imshow("intersection", intersection);
    //waitKey();



    //    cv::Canny(nan_image, canny_img, 50, 200, 3);
    //TODO: get the line length in image from the depth

    //vector<Vec4i> lines;
    //findLines(canny_img, lines);

    //int max_line_length = 100;
    //drawLines(canny_img, dst, lines, max_line_length);
    
    //createNaN point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr nan_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    createNaNPointCloud (intersection, nan_cloud);

    std::vector<pcl::PointIndices> handle_clusters;
    findHandlesInNanPointCloud (nan_cloud, handle_clusters);
    //saveHandleClusters (handle_clusters, nan_cloud);

    std::vector<geometry_msgs::PointStamped> vector_point_avg;
    recoverCentroidPose(handle_clusters, nan_cloud, vector_point_avg);

    transformPoints(vector_point_avg, "base_link");

    rejectFalsePositives(vector_point_avg);
    //response to service call
    for (uint i = 0; i < vector_point_avg.size(); i++)
      {
	res.handle_poses.push_back (vector_point_avg[i]);
      }

    //    std::vector<double> expected_handle_image_length;
    //    getHandleImageLength(vector_point_avg, expected_handle_image_length);
    
    // std::vector<int> true_pos;
    // rejectFalsePositives (handle_clusters, expected_handle_image_length, nan_cloud, true_pos);
    
    // std::vector<geometry_msgs::PointStamped> vector_point_avg_minus_fp;
    // for (uint i = 0; i < true_pos.size(); i++)
    //   {
    // 	vector_point_avg_minus_fp.push_back(vector_point_avg[true_pos[i]]);
    //   }
    
    //publishMarker(vector_point_avg_minus_fp);
    publishMarker(vector_point_avg);
     if (save_cloud_)
      {
	std::stringstream ss;
	ros::Time time = ros::Time::now();
	ss << "original_" << time << ".pcd";
	ROS_INFO ("[%s:] Cloud saved to %s", getName().c_str(), ss.str ().c_str ());
	pcd_writer_.write(ss.str (), *pcl_cloud_);
	//pcd_writer_.write(ss.str (), *biggest_plane_cloud_);
	ss.str("");
	ss << "nan_cloud_" << time << ".pcd";
	ROS_INFO ("[%s:] Cloud saved to %s", getName().c_str(), ss.str ().c_str ());
	pcd_writer_.write(ss.str (), *nan_cloud);
      }

    if (save_image_)
      {
	std::stringstream ss;
	ss << ros::Time::now() << ".png";
	ROS_INFO ("[%s:] Image saved to %s", getName().c_str(), ss.str ().c_str ());
	imwrite(ss.str(), dst);
      }

    nan_cloud->points.clear();
    //    imshow("nan_image", nan_image);
    //    imshow("dst", dst);
    //waitKey(10);
    // sleep(10);
    return true;
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "handle_detector_nan");
  ros::NodeHandle n("~");
  HandleDetectorNANNode tp(n);
  ros::spin ();
  return (0);
}

/* ]--- */

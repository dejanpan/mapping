/*
 * Copyright (c) 2010, Dejan Pangercic <dejan.pangercic@cs.tum.edu>,
 Zoltan-Csaba Marton <marton@cs.tum.edu>, Nico Blodow <blodow@cs.tum.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PCL_CLOUD_ALGOS_BOX_ESTIMATION_H
#define PCL_CLOUD_ALGOS_BOX_ESTIMATION_H
#include <pcl_cloud_algos/cloud_algos.h>
//#include <mapping_msgs/PolygonalMap.h>
//#include <position_string_msgs/PositionStringList.h>
#include <triangle_mesh_msgs/TriangleMesh.h>
//#include <point_cloud_mapping/geometry/point.h>
#include <pcl_cloud_algos/pcl_cloud_algos_point_types.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <pcl/point_cloud.h>

namespace pcl_cloud_algos
{

class BoxEstimation : public CloudAlgo
{
 public:
  // should the result marker be published or only rotated
  bool publish_marker_; // TODO: add setter and getter and make it protected (also in cylinder fit)

  BoxEstimation () : CloudAlgo ()
  {
    output_box_topic_ = std::string("box_marker");
    threshold_in_ = 0.025;
    threshold_out_ = 0.00001;
    publish_marker_ = true;
  };

  typedef triangle_mesh_msgs::TriangleMesh OutputType;
  typedef sensor_msgs::PointCloud2 InputType;

  static std::string default_input_topic ()
    {return std::string ("/merged_cloud");}

  static std::string default_output_topic ()
    {return std::string ("mesh_box");};

  static std::string default_node_name ()
    {return std::string ("box_estimation_node");};

  void init (ros::NodeHandle&);
  void pre  ();
  void post ();
  std::vector<std::string> requires ();
  std::vector<std::string> provides ();
  std::string process (const boost::shared_ptr<const InputType>);
  boost::shared_ptr<const OutputType> output ();

  // Get inlier and outlier points
  virtual boost::shared_ptr<sensor_msgs::PointCloud2 > getInliers ();
  virtual boost::shared_ptr<sensor_msgs::PointCloud2 > getOutliers ();
  virtual boost::shared_ptr<sensor_msgs::PointCloud2 > getContained ();
  virtual boost::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal> > getThresholdedInliers (double eps_angle);
  virtual void computeInAndOutliers (boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZINormal> > cloud, std::vector<double> coeff, double threshold_in, double threshold_out);

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief function for actual model fitting
   * \param cloud de-noisified input point cloud message
   * \param coeff box to-be-filled-in coefficients(15 elements):
   * box center: cx, cy, cz,
   * box dimensions: dx, dy, dz,
   * box eigen axes: e1_x, e1y, e1z, e2_x, e2y, e2z, e3_x, e3y, e3z
   */
  bool find_model (boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZINormal> > cloud, std::vector<double> &coeff);

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief makes triangular mesh out of box coefficients
   * \param cloud input point cloud message
   * \param coeff box coefficients (see find_model function):
   */
  void triangulate_box (boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZINormal> > cloud, std::vector<double> &coeff);

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief publish box as marker for rvis visualisation
   * \param cloud input point cloud message
   * \param coeff box coefficients (see find_model function):
   */
  void publish_marker (boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZINormal> > cloud, std::vector<double> &coeff);

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Sets the internal box as marker for rvis visualisation.
   * \param cloud input point cloud message
   * \param coeff box coefficients (see find_model function):
   */
  void computeMarker (boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZINormal> > cloud, std::vector<double> coeff);

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Returns the internal box as marker for rvis visualisation
   * \param cloud input point cloud message
   * \param coeff box coefficients (see find_model function):
   */
  visualization_msgs::Marker getMarker () { return marker_; }

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Returns the computed model coefficients
   * \param cloud input point cloud message
   * \param coeff box coefficients (see find_model function):
   */
  std::vector<double> getCoeff () { return coeff_; }

  ros::Publisher createPublisher (ros::NodeHandle& nh)
  {
    ros::Publisher p = nh.advertise<OutputType> (default_output_topic (), 5);
    return p;
  }
 protected:
  boost::shared_ptr<OutputType> mesh_;
  std::vector<int> inliers_;
  std::vector<int> outliers_;
  std::vector<int> contained_;
  double threshold_in_;
  double threshold_out_;

  boost::shared_ptr<pcl::PointCloud <pcl::PointXYZINormal> > cloud_;

  ros::NodeHandle nh_;

  //model rviz publisher
  ros::Publisher marker_pub_;
  ros::Publisher inliers_pub_;
  ros::Publisher outliers_pub_;
  ros::Publisher contained_pub_;

  //box coefficients: cx, cy, cz, dx, dy, dz, e1_x, e1y, e1z, e2_x, e2y, e2z, e3_x, e3y, e3z
  std::vector<double> coeff_;
  pcl::PointXYZINormal box_centroid_;
  visualization_msgs::Marker marker_;

  //publish box as marker
  std::string output_box_topic_;

  //point color
  float r_, g_, b_;
  //lock point cloud
  //boost::mutex lock;
};

}
#endif



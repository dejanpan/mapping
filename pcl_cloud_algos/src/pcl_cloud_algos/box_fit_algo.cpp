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

///NEW PCL INCLUDES

#include <pcl/features/feature.h>
#include <pcl/io/io.h>

#include <stdlib.h>

#include <vector>
#include <iostream>

// include all descriptors for you
#include <ias_descriptors_3d/all_descriptors.h>

// kd-tree, getPointCloud and computeCentroid
//#include <point_cloud_mapping/kdtree/kdtree_ann.h>
//#include <point_cloud_mapping/geometry/point.h>
//#include <point_cloud_mapping/geometry/nearest.h>

//ros
#include <sensor_msgs/PointCloud2.h>
//#include <geometry_msgs/PointStamped.h>
//#include <geometry_msgs/Point32.h>
#include <ros/ros.h>

//cloud_algos
#include <pcl_cloud_algos/box_fit_algo.h>

//bullet TODO: maybe replace that function with one from Eigen
#include <tf/tf.h>

// Eigen
#include <Eigen/Array>

#include <angles/angles.h>

#include <triangle_mesh_msgs/TriangleMesh.h>


using namespace std;
using namespace pcl_cloud_algos;

void BoxEstimation::init (ros::NodeHandle& nh)
{
  nh_ = nh;
  nh_.param("output_box_topic", output_box_topic_, output_box_topic_);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>(output_box_topic_, 0 );
  inliers_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("inliers", 0 );
  outliers_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("outliers", 0 );
  contained_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("contained", 0 );
  coeff_.resize(15);
  r_ = g_ = 0.0;
  b_ = 1.0;
}

void BoxEstimation::pre  ()
{
  nh_.param("output_box_topic", output_box_topic_, output_box_topic_);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>(output_box_topic_, 0 );
  nh_.param("threshold_in", threshold_in_, threshold_in_);
  nh_.param("threshold_out", threshold_out_, threshold_out_);
  nh_.param("publish_marker", publish_marker_, publish_marker_);
}

void BoxEstimation::post ()
{
}

std::vector<std::string> BoxEstimation::requires ()
{
  std::vector<std::string> ret;
  ret.push_back (std::string("index"));
  return ret;
}

std::vector<std::string> BoxEstimation::provides ()
{
  return std::vector<std::string>();
}

std::string BoxEstimation::process (const boost::shared_ptr<const InputType> input)
{
  //converting to pcl format
  pcl::fromROSMsg(*input, *cloud_);

  if (verbosity_level_ > 0) ROS_INFO ("[BoxEstimation] PointCloud message received on %s with %d points", default_input_topic().c_str (), (int)cloud_->points.size ());

  // Compute model coefficients
  bool model_found = find_model (cloud_, coeff_);

  // Check if a valid model was found
  if (!model_found)
  {
    output_valid_ = false;
    return std::string ("no model found");
  }
  else
  {
    // Create mesh as output
    triangulate_box (cloud_, coeff_);

    // Publish fitted box on marker topic
    if (verbosity_level_ > 0) ROS_INFO ("[BoxEstimation] Publishing box marker on topic %s.", nh_.resolveName (output_box_topic_).c_str ());
    if (publish_marker_)
      publish_marker (cloud_, coeff_);
    else
      computeMarker (cloud_, coeff_);

    // Get which points verify the model and which don't
    // cloud_ = input;
    computeInAndOutliers (cloud_, coeff_, threshold_in_, threshold_out_);
    inliers_pub_.publish (getInliers ());
    outliers_pub_.publish (getOutliers ());
    contained_pub_.publish (getContained ());

    output_valid_ = true;
    return std::string ("ok");
  }
}

boost::shared_ptr<const BoxEstimation::OutputType> BoxEstimation::output ()
{
  return mesh_;
};

boost::shared_ptr<sensor_msgs::PointCloud2> BoxEstimation::getOutliers ()
{
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal> > ret (new pcl::PointCloud<pcl::PointXYZINormal>);
  boost::shared_ptr<sensor_msgs::PointCloud2> ret_msg (new sensor_msgs::PointCloud2);
  //ROS_INFO("created PointCloud object: 0x%x", (void*) ret.get()); - ZOLI COMMENTED THIS TO GET RID OF WARNING, SUPPOSING WAS ONLY DEBUG :)

  ret->header = cloud_->header;
  //int channel_index = getChannelIndex (cloud_, "index");
  //int channel_line = getChannelIndex (cloud_, "line");
  //if (channel_line != -1)
  //{
  //  ret->channels.resize(2);
  //  ret->channels[1].name = cloud_->channels[channel_line].name;
  //  ret->channels[1].values.resize(outliers_.size());
  //  ret->channels[0].name = cloud_->channels[channel_index].name;
  //  ret->channels[0].values.resize(outliers_.size());
  //}
  //else if (channel_index != -1)
  //{
  //  ret->channels.resize(1);
  //  ret->channels[0].name = cloud_->channels[channel_index].name;
  //  ret->channels[0].values.resize(outliers_.size());
  //}

  ret->points.resize(outliers_.size());
  for (unsigned int i = 0; i < outliers_.size (); i++)
  {
    ret->points.at(i) = cloud_->points.at (outliers_.at (i));
    //if (channel_index != -1)
    //  ret->channels[0].values.at(i) = cloud_->channels[channel_index].values.at (outliers_.at (i));
    //if (channel_line != -1)
    //  ret->channels[1].values.at(i) = cloud_->channels[channel_line].values.at (outliers_.at (i));
  }
  pcl::toROSMsg(*ret, *ret_msg);
  return ret_msg;
}

boost::shared_ptr<sensor_msgs::PointCloud2> BoxEstimation::getInliers ()
{
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal> > ret (new pcl::PointCloud<pcl::PointXYZINormal> ());
  boost::shared_ptr<sensor_msgs::PointCloud2> ret_msg (new sensor_msgs::PointCloud2);
  pcl::copyPointCloud (*cloud_, inliers_, *ret);
  //ret->points.reserve (inliers_.size ());
  //ret->header = cloud_->header;
  //for (unsigned int i = 0; i < inliers_.size (); i++)
  //  ret->points.push_back (cloud_->points.at (inliers_.at (i)));
  pcl::toROSMsg(*ret, *ret_msg);
  return ret_msg;
}

boost::shared_ptr<sensor_msgs::PointCloud2> BoxEstimation::getContained ()
{
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal> > ret (new pcl::PointCloud<pcl::PointXYZINormal> ());
  boost::shared_ptr<sensor_msgs::PointCloud2> ret_msg (new sensor_msgs::PointCloud2);
  pcl::copyPointCloud (*cloud_, contained_, *ret);
  //ret->points.reserve (contained_.size ());
  //ret->header = cloud_->header;
  //for (unsigned int i = 0; i < contained_.size (); i++)
  //  ret->points.push_back (cloud_->points.at (contained_.at (i)));
  pcl::toROSMsg(*ret, *ret_msg);
  return ret_msg;
}

boost::shared_ptr<pcl::PointCloud <pcl::PointXYZINormal> > BoxEstimation::getThresholdedInliers (double eps_angle)
{
  //int nxIdx = getChannelIndex(cloud_, "nx");
  //if (nxIdx == -1)
  //  return getInliers ();
  //else
  //{
    Eigen::Matrix3d axes = Eigen::Matrix3d::Map(&coeff_[6]).transpose ();
    //Eigen::Matrix3f axes = Eigen::Matrix3d::Map(&coeff_[6]).cast<float> ().transpose ();
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal> > ret (new pcl::PointCloud<pcl::PointXYZINormal> ());
    ret->points.reserve (inliers_.size ());
    ret->header = cloud_->header;
    for (unsigned int i = 0; i < inliers_.size (); i++)
    {
      Eigen::Vector3d normal (cloud_->points.at (inliers_.at(i)).normal[0],
                              cloud_->points.at (inliers_.at(i)).normal[1],
                              cloud_->points.at (inliers_.at(i)).normal[2]);

      // TODO: include top inliers to cylinders!
      for (int d = 0; d < 3; d++)
      {
        double cosine = fabs (normal.dot (axes.row(d)));
        if (cosine > 1) cosine = 1;
        if (cosine < -1) cosine = -1;
        if (acos (cosine) < eps_angle)
        {
          ret->points.push_back (cloud_->points.at (inliers_.at (i)));
          break;
        }
      }
    }
    return ret;
  //}
}

void BoxEstimation::computeInAndOutliers (boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZINormal> > cloud, std::vector<double> coeff, double threshold_in, double threshold_out)
{
  //Eigen::Matrix3f axes = Eigen::Matrix3d::Map(&coeff[6]).cast<float> ().transpose ();
  Eigen::Matrix3d axes = Eigen::Matrix3d::Map(&coeff[6]).transpose ();
  //std::cerr << "the 3 axes:\n" << axes << std::endl;
  //std::cerr << "threshold: " << threshold << std::endl;

  // get inliers and outliers
  inliers_.resize (0);
  outliers_.resize (0);
  contained_.resize (0);
  for (unsigned i = 0; i < cloud->points.size (); i++)
  {
    // compute point-center
    Eigen::Vector3d centered (cloud->points[i].x - coeff[0], cloud->points[i].y - coeff[1], cloud->points[i].z - coeff[2]);
    // project (point-center) on axes and check if inside or outside the +/- dimensions
    bool inlier = false;
    bool outlier = false;
    for (int d = 0; d < 3; d++)
    {
      double dist = fabs (centered.dot (axes.row(d)));
      if (dist > coeff[3+d]/2 + threshold_out)
      {
        outlier = true;
        break;
      }
      else if (fabs (dist - coeff[3+d]/2) <= threshold_in)
      {
        inlier = true;
        break;
      }
    }
    if (inlier)
      inliers_.push_back(i);
    else if (outlier)
      outliers_.push_back(i);
    else
      contained_.push_back(i);
  }
  if (verbosity_level_ > 0) ROS_INFO("[BoxEstimation] %ld points verify model, %ld are outside of it, and %ld are contained in it", inliers_.size(), outliers_.size(), contained_.size());
}

////////////////////////////////////////////////////////////////////////////////
/**
 * actual model fitting happens here
 */
bool BoxEstimation::find_model (boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZINormal> > cloud, std::vector<double> &coeff)
{
  //Eigen::Vector4f centroid;
  //pcl::compute3DCentroid (*cloud, centroid);
  //coeff[0] = box_centroid_.x;
  //coeff[1] = box_centroid_.y;
  //coeff[2] = box_centroid_.z;

  //// ----------------------------------------------
  //// Read point cloud data and create Kd-tree that represents points.
  //// We will compute features for all points in the point cloud.
  //std::vector<const geometry_msgs::Point32*> interest_pts;
  //interest_pts.reserve(cloud->points.size());
  ////std::vector<const geometry_msgs::Point32*> interest_pts(data.points.size());
  //for (size_t i = 0 ; i < cloud->points.size() ; i++)
  //{
  //  interest_pts.push_back(&(cloud->points[i]));
  //}

  //cloud_kdtree::KdTreeANN data_kdtree(*cloud);

  //// ----------------------------------------------
  //// SpectralAnalysis is not a descriptor, it is a class that holds
  //// intermediate data to be used/shared by different descriptors.
  //// It performs eigen-analyis of local neighborhoods and extracts
  //// eigenvectors and values.  Here, we set it to look at neighborhoods
  //// within a radius of 5.0 around each interest point.
  //double r = 0.1;
  //SpectralAnalysis sa(r);
  //BoundingBoxSpectral bbox_spectral(r, sa);
  //OrientationTangent o_tangent(1, 0, 0, sa);
  //// ----------------------------------------------
  //// Put all descriptors into a vector
  //vector<Descriptor3D*> descriptors_3d;
  ////   descriptors_3d.push_back(&shape_spectral);
  ////   descriptors_3d.push_back(&spin_image1);
  ////   descriptors_3d.push_back(&spin_image2);
  ////   descriptors_3d.push_back(&o_normal);
  //descriptors_3d.push_back(&o_tangent);
  ////   descriptors_3d.push_back(&position);
  //descriptors_3d.push_back(&bbox_spectral);

  //// ----------------------------------------------
  //// Iterate over each descriptor and compute features for each point in the point cloud.
  //// The compute() populates a vector of vector of floats, i.e. a feature vector for each
  //// interest point.  If the features couldn't be computed successfully for an interest point,
  //// its feature vector has size 0
  //unsigned int nbr_descriptors = descriptors_3d.size();
  //vector<std::vector<std::vector<float> > > all_descriptor_results(nbr_descriptors);
  //for (unsigned int i = 0 ; i < nbr_descriptors ; i++)
  //{
  //  descriptors_3d[i]->compute(*cloud, data_kdtree, interest_pts, all_descriptor_results[i]);
  //}

  //std::vector<float>& ot = all_descriptor_results[0][0];
  //if (verbosity_level_ > 1) cout << "Orientation tangent size: " <<  ot.size() << endl;
  //for (size_t i = 0 ; i < ot.size() ; i++)
  //  if (verbosity_level_ > 1) cerr << "Orientation tangent value(s): " << ot[i] << endl;

  ////   // ----------------------------------------------
  ////   // Print out the bounding box dimension features for the first point 0
  //std::vector<float>& pt0_bbox_features = all_descriptor_results[1][0];
  //if (verbosity_level_ > 1) cout << "Bounding box features size: " <<  pt0_bbox_features.size() << endl;
  //for (size_t i = 0 ; i < pt0_bbox_features.size() ; i++)
  //{
  //  if (i < 12)
  //  {
  //    coeff[i+3] = pt0_bbox_features[i];
  //  }
  //  else
  //    if (verbosity_level_ > -1) ROS_WARN("[BoxEstimation] Box dimensions bigger than 3 - unusual");
  //}
  //if (verbosity_level_ > 0) ROS_INFO("[BoxEstimation] Box dimensions x: %f, y: %f, z: %f ", pt0_bbox_features[0],  pt0_bbox_features[1],  pt0_bbox_features[2]);
  //if (verbosity_level_ > 0) ROS_INFO("[BoxEstimation] Eigen vectors: \n\t%f %f %f \n\t%f %f %f \n\t%f %f %f", pt0_bbox_features[3], pt0_bbox_features[4],
  //         pt0_bbox_features[5], pt0_bbox_features[6], pt0_bbox_features[7], pt0_bbox_features[8],
  //         pt0_bbox_features[9], pt0_bbox_features[10],pt0_bbox_features[11]);

  // TODO: we should get feedback on success from the compute function
  //return true;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * \brief triangulates box
 */
void BoxEstimation::triangulate_box(boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZINormal> > cloud, std::vector<double> &coeff)
{
  pcl::PointXYZINormal current_point;
  triangle_mesh_msgs::Triangle triangle;
  mesh_ = boost::shared_ptr <BoxEstimation::OutputType> (new BoxEstimation::OutputType);
  mesh_->points.resize(8);
  mesh_->triangles.resize(12);
  mesh_->header = cloud->header;

  int counter = 0;

  // create box vertices
  for (int i = -1; i <= 1; i = i+2)
  {
    for (int j = -1; j <= 1; j = j+2)
    {
      for (int k = -1; k <= 1; k = k+2)
      {
        // Movement along axes
        double moving[3];
        moving[0] = i * coeff[3]/2;
        moving[1] = j * coeff[4]/2;
        moving[2] = k * coeff[5]/2;

        // Move box center
        current_point.x = coeff[0] + moving[0]*coeff[6+0] + moving[1]*coeff[9+0] + moving[2]*coeff[12+0];
        current_point.y = coeff[1] + moving[0]*coeff[6+1] + moving[1]*coeff[9+1] + moving[2]*coeff[12+1];
        current_point.z = coeff[2] + moving[0]*coeff[6+2] + moving[1]*coeff[9+2] + moving[2]*coeff[12+2];

        //current_point.x = coeff[0] + i * coeff[3]/2;
        //current_point.y = coeff[1] + j * coeff[4]/2;
        //current_point.z = coeff[2] + k * coeff[5]/2;

        mesh_->points[counter++].x = current_point.x;
        mesh_->points[counter++].y = current_point.y;
        mesh_->points[counter++].z = current_point.z;
      }
    }
  }

  // fill in the box sides (2 triangles per side)
  counter = 0;
  triangle.i = 0, triangle.j = 1, triangle.k = 2;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 0, triangle.j = 1, triangle.k = 4;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 0, triangle.j = 2, triangle.k = 6;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 0, triangle.j = 6, triangle.k = 4;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 1, triangle.j = 4, triangle.k = 5;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 5, triangle.j = 4, triangle.k = 6;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 5, triangle.j = 7, triangle.k = 6;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 7, triangle.j = 6, triangle.k = 2;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 7, triangle.j = 3, triangle.k = 2;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 1, triangle.j = 2, triangle.k = 3;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 1, triangle.j = 3, triangle.k = 7;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 1, triangle.j = 5, triangle.k = 7;
  mesh_->triangles[counter++] = triangle;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * \brief publishes model marker (to rviz)
 */
void BoxEstimation::publish_marker (boost::shared_ptr<const pcl::PointCloud <pcl::PointXYZINormal> > cloud, std::vector<double> &coeff)
{
  computeMarker (cloud, coeff);
  marker_pub_.publish (marker_);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * \brief computes model marker (to rviz)
 */
void BoxEstimation::computeMarker (boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZINormal> > cloud, std::vector<double> coeff)
{
  btMatrix3x3 box_rot (coeff[6], coeff[7], coeff[8],
                       coeff[9], coeff[10], coeff[11],
                       coeff[12], coeff[13], coeff[14]);
  btMatrix3x3 box_rot_trans (box_rot.transpose());
  btQuaternion qt;
  box_rot_trans.getRotation(qt);

  //marker_.header.frame_id = "base_link";
  //marker_.header.stamp = ros::Time();
  marker_.header = cloud->header;
  marker_.ns = "BoxEstimation";
  marker_.id = 0;
  marker_.type = visualization_msgs::Marker::CUBE;
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.pose.position.x = coeff[0];
  marker_.pose.position.y = coeff[1];
  marker_.pose.position.z = coeff[2];
  marker_.pose.orientation.x = qt.x();
  marker_.pose.orientation.y = qt.y();
  marker_.pose.orientation.z = qt.z();
  marker_.pose.orientation.w = qt.w();
  marker_.scale.x = coeff[3];
  marker_.scale.y = coeff[4];
  marker_.scale.z = coeff[5];
  marker_.color.a = 0.3;
  marker_.color.r = 0.0;
  marker_.color.g = 1.0;
  marker_.color.b = 0.0;
  std::cerr << "BOX MARKER COMPUTED, WITH FRAME " << marker_.header.frame_id << std::endl;
}

#ifndef NO_BOXFIT_NODE
#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <BoxEstimation> (argc, argv);
}
#endif
#endif


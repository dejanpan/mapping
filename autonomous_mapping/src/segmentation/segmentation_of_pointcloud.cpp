/*
 * Copyright (c) 2011, Lucian Cosmin Goron <goron@cs.tum.edu>, Zoltan-Csaba Marton <marton@cs.tum.edu>
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



// ros dependencies
#include "ros/ros.h"

// terminal tools dependecies
#include "pcl/console/parse.h"

// pcl dependencies
#include "pcl/io/pcd_io.h"

#include "pcl/surface/convex_hull.h"

#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/filters/statistical_outlier_removal.h"

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/impl/ransac.hpp"

#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/extract_polygonal_prism_data.h"

// pcl visualization dependencies
#include "pcl/visualization/pcl_visualizer.h"

// pcl ias sample consensus dependencies
#include "pcl_ias_sample_consensus/pcl_sac_model_orientation.h"

// dos pcl dependencies
#include "dos_pcl/segmentation/door_detection_by_color_and_fixture.h"



// Set up the right type definition of points
typedef pcl::PointXYZ PointT;
//typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointXYZINormal PointT;



// Filtering's Parameters
double height_of_floor = 0.025; /// [percentage]
double height_of_ceiling = 0.050; /// [percentage]
double height_of_walls = 0.750; /// [percentage]

// Segmentation's Parameters
double epsilon_angle = 0.010; /// [radians]
double plane_threshold = 0.100; /// [meters]
int minimum_plane_inliers = 1000; /// [points]
int maximum_plane_iterations = 1000; /// [iterations]

// Clustering's Parameters
int minimum_size_of_plane_cluster = 100; /// [points]
double plane_inliers_clustering_tolerance = 0.100; /// [meters]
int minimum_size_of_handle_cluster = 25; /// [points]
double handle_clustering_tolerance = 0.050; /// [meters]

// Segmentation's Parameters
double cluster_tolerance = 0.020; /// [meters]
double fixture_cluster_tolerance = 0.025; /// [meters]
double center_radius = 0.085; /// 
double init_radius = 0.035; ///
double color_radius = 0.050; ///

int std_limit = 2; ///
int min_pts_per_cluster = 150; /// [points]
int fixture_min_pts_per_cluster = 150; /// [points]

// Visualization's Parameters
bool step = false;
bool clean = false;
bool verbose = false;
int size_of_points = 1;

// Optional parameters
bool find_box_model = true;
bool segmentation_by_color_and_fixture = false;



/*

float getRGB (float r, float g, float b)
{
  int res = (int (r * 255) << 16) | (int (g * 255) << 8) | int (b * 255);
  double rgb = *(float*)(&res);
  return (rgb);
}

*/

float getRGB (float r, float g, float b)
{
  int res = (int (r * 255) << 16) | (int (g * 255) << 8) | int (b * 255);
  double rgb = *reinterpret_cast<float*>(&res);
  return (rgb);
}

///*

void getMinAndMax (boost::shared_ptr<const pcl::PointCloud <PointT> > cloud_, Eigen::VectorXf model_coefficients, boost::shared_ptr<pcl::SACModelOrientation<pcl::Normal> > model, std::vector<int> &min_max_indices, std::vector<float> &min_max_distances)
{

  boost::shared_ptr<std::vector<int> > inliers = model->getIndices();
  boost::shared_ptr<std::vector<int> > indices = model->getIndices();

  // Initialize result vectors
  min_max_indices.resize (6);
  min_max_distances.resize (6);
  min_max_distances[0] = min_max_distances[2] = min_max_distances[4] = +DBL_MAX;
  min_max_distances[1] = min_max_distances[3] = min_max_distances[5] = -DBL_MAX;

  // The 3 coordinate axes are nm, nc and axis_
  Eigen::Vector3f nm;
  nm[0] = model_coefficients[0];
  nm[1] = model_coefficients[1];
  nm[2] = model_coefficients[2];
  //Eigen::Vector3f nm = Eigen::Vector3d::Map(&(*model_coefficients)[0]).cast<float> ();
  Eigen::Vector3f nc = model->axis_.cross (nm);

  // Find minimum and maximum distances from origin along the three axes
  for (std::vector<int>::iterator it = inliers->begin (); it != inliers->end (); it++)
  //for (unsigned i = 0; i < inliers.size (); i++)
  {
    // @NOTE inliers is a list of indices of the indices_ array!
    Eigen::Vector3f point (cloud_->points[(*indices)[*it]].x, cloud_->points[(*indices)[*it]].y, cloud_->points[(*indices)[*it]].z);
    //Eigen::Vector3f point (cloud_->points[indices_[*it]].x - center.x, cloud_->points[indices_[*it]].y - center.y, cloud_->points[indices_[*it]].z - center.z);
    //Eigen::Vector3f point (cloud_->points[indices_[inliers[i]]].x, cloud_->points[indices_[inliers[i]]].y, cloud_->points[indices_[inliers[i]]].z);
    double dists[3];
    dists[0] = nm.dot(point);
    dists[1] = nc.dot(point);
    dists[2] = model->axis_.dot(point);
    for (int d=0; d<3; d++)
    {
      if (min_max_distances[2*d+0] > dists[d]) { min_max_distances[2*d+0] = dists[d]; min_max_indices[2*d+0] = *it; }
      if (min_max_distances[2*d+1] < dists[d]) { min_max_distances[2*d+1] = dists[d]; min_max_indices[2*d+1] = *it; }
    }
  }

}

//*/

////////////////////////////////////////////////////////////////////////////////
/**
 * actual model fitting happens here
 */

///*

bool findBoxModel (boost::shared_ptr<const pcl::PointCloud <PointT> > cloud, pcl::PointCloud<pcl::Normal> nrmls, std::vector<double> &coeff, double eps_angle_ = 0.1, double success_probability_ = 0.99, int verbosity_level_ = 1)
{

  if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Looking for box in a cluster of %u points", (unsigned)cloud->points.size ());

  // Compute center point
  //cloud_geometry::nearest::computeCentroid (*cloud, box_centroid_);

/* 
  pcl::PointCloud<pcl::Normal> nrmls ;
  nrmls.header = cloud->header;
  nrmls.points.resize(cloud->points.size());
  for(size_t i = 0 ; i < cloud->points.size(); i++)
  {
    nrmls.points[i].normal[0] = cloud->points[i].normal[0];
    nrmls.points[i].normal[1] = cloud->points[i].normal[1];
    nrmls.points[i].normal[2] = cloud->points[i].normal[2];
  }
*/

  // Create model
  pcl::SACModelOrientation<pcl::Normal>::Ptr model = boost::make_shared<pcl::SACModelOrientation<pcl::Normal> >(nrmls.makeShared ());

  model->axis_[0] = 0 ;
  model->axis_[1] = 0 ;
  model->axis_[2] = 1 ;

  //model->setDataSet ((sensor_msgs::PointCloud*)(cloud.get())); // TODO: this is nasty :)
  if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Axis is (%g,%g,%g) and maximum angular difference %g",
      model->axis_[0], model->axis_[1], model->axis_[2], eps_angle_);

  // Check probability of success and decide on method
  Eigen::VectorXf refined;
  std::vector<int> inliers;
  /// @NOTE: inliers are actually indexes in the indices_ array, but that is not set (by default it has all the points in the correct order)
  if (success_probability_ > 0 && success_probability_ < 1)
  {
    if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Using RANSAC with stop probability of %g and model refinement", success_probability_);

    // Fit model using RANSAC
    pcl::RandomSampleConsensus<pcl::Normal> *sac = new pcl::RandomSampleConsensus<pcl::Normal> (model, eps_angle_);
    sac->setProbability (success_probability_);
    if (!sac->computeModel ())
    {
      if (verbosity_level_ > -2) ROS_ERROR ("[RobustBoxEstimation] No model found using the angular threshold of %g!", eps_angle_);
      return false;
    }

    // Get inliers and refine result
    sac->getInliers(inliers);
    if (verbosity_level_ > 1) cerr << "number of inliers: " << inliers.size () << endl;
    // Exhaustive search for best model
    std::vector<int> best_sample;
    std::vector<int> best_inliers;
    Eigen::VectorXf model_coefficients;
    for (unsigned i = 0; i < cloud->points.size (); i++)
    {
      std::vector<int> selection (1);
      selection[0] = i;
      model->computeModelCoefficients (selection, model_coefficients);

      model->selectWithinDistance (model_coefficients, eps_angle_, inliers);
      if (best_inliers.size () < inliers.size ())
      {
        best_inliers = inliers;
        best_sample = selection;
      }
    }

    // Check if successful and save results
    if (best_inliers.size () > 0)
    {
      model->computeModelCoefficients (best_sample, refined);
      //model->getModelCoefficients (refined);
      /// @NOTE: making things transparent for the outside... not really needed
      inliers = best_inliers;
      //model->setBestModel (best_sample);
      //model->setBestInliers (best_inliers);
      // refine results: needs inliers to be set!
      // sac->refineCoefficients(refined);
    }
    /// @NOTE best_model_ contains actually the samples used to find the best model!
    //model->computeModelCoefficients(model->getBestModel ());
    //Eigen::VectorXf original;
    //model->getModelCoefficients (original);
    //if (verbosity_level_ > 1) cerr << "original direction: " << original[0] << " " << original[1] << " " << original[2] << ", found at point nr " << original[3] << endl;
    //sac->refineCoefficients(refined);
   // if (verbosity_level_ > 1) cerr << "refitted direction: " << refined.at (0) << " " << refined.at (1) << " " << refined.at (2) << ", initiated from point nr " << refined.at (3) << endl;
   // if (refined[3] == -1)
   //   refined = original;
  }
  else
  {
    if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Using exhaustive search in %ld points", (long int) cloud->points.size ());

    // Exhaustive search for best model
    std::vector<int> best_sample;
    std::vector<int> best_inliers;
    Eigen::VectorXf model_coefficients;
    for (unsigned i = 0; i < cloud->points.size (); i++)
    {
      std::vector<int> selection (1);
      selection[0] = i;
      model->computeModelCoefficients (selection, model_coefficients);

      model->selectWithinDistance (model_coefficients, eps_angle_, inliers);
      if (best_inliers.size () < inliers.size ())
      {
        best_inliers = inliers;
        best_sample = selection;
      }
    }

    // Check if successful and save results
    if (best_inliers.size () > 0)
    {
      model->computeModelCoefficients (best_sample, refined);
      //model->getModelCoefficients (refined);
      /// @NOTE: making things transparent for the outside... not really needed
      inliers = best_inliers;
      //model->setBestModel (best_sample);
      //model->setBestInliers (best_inliers);
      // refine results: needs inliers to be set!
      // sac->refineCoefficients(refined);
    }
    else
    {
      if (verbosity_level_ > -2) ROS_ERROR ("[RobustBoxEstimation] No model found using the angular threshold of %g!", eps_angle_);
      return false;
    }
  }

  // Save fixed axis
  coeff[12+0] = model->axis_[0];
  coeff[12+1] = model->axis_[1];
  coeff[12+2] = model->axis_[2];

  // Save complementary axis (cross product)
  coeff[9+0] = model->axis_[1]*refined[2] - model->axis_[2]*refined[1];
  coeff[9+1] = model->axis_[2]*refined[0] - model->axis_[0]*refined[2];
  coeff[9+2] = model->axis_[0]*refined[1] - model->axis_[1]*refined[0];

  // Save principle axis (corrected)
  refined[0] = - (model->axis_[1]*coeff[9+2] - model->axis_[2]*coeff[9+1]);
  refined[1] = - (model->axis_[2]*coeff[9+0] - model->axis_[0]*coeff[9+2]);
  refined[2] = - (model->axis_[0]*coeff[9+1] - model->axis_[1]*coeff[9+0]);

  ROS_INFO("refined[0]: %f", refined[0]);
  ROS_INFO("refined[1]: %f", refined[1]);
  ROS_INFO("refined[2]: %f", refined[2]);
  coeff[6+0] = refined[0];
  coeff[6+1] = refined[1];
  coeff[6+2] = refined[2];

//  // Save complementary axis (AGIAN, JUST TO MAKE SURE)
//  coeff[9+0] = model->axis_[1]*refined[2] - model->axis_[2]*refined[1];
//  coeff[9+1] = model->axis_[2]*refined[0] - model->axis_[0]*refined[2];
//  coeff[9+2] = model->axis_[0]*refined[1] - model->axis_[1]*refined[0];

  // Compute minimum and maximum along each dimension for the whole cluster
  std::vector<int> min_max_indices;
  std::vector<float> min_max_distances;
  //boost::shared_ptr<vector<int> > indices (new vector<int>);
  //indices = model->getIndices();

  //model->getMinAndMax (&refined, &inliers, min_max_indices, min_max_distances);
  //getMinAndMax (&refined, model->getIndices (), min_max_indices, min_max_distances);
  getMinAndMax (cloud, refined, model, min_max_indices, min_max_distances);
  //vector<int> min_max_indices = model->getMinAndMaxIndices (refined);

  //cerr << min_max_distances.at (1) << " " << min_max_distances.at (0) << endl;
  //cerr << min_max_distances.at (3) << " " << min_max_distances.at (2) << endl;
  //cerr << min_max_distances.at (5) << " " << min_max_distances.at (4) << endl;

  // Save dimensions
  coeff[3+0] = min_max_distances.at (1) - min_max_distances.at (0);
  coeff[3+1] = min_max_distances.at (3) - min_max_distances.at (2);
  coeff[3+2] = min_max_distances.at (5) - min_max_distances.at (4);

  // Distance of box's geometric center relative to origin along orientation axes
  double dist[3];
  dist[0] = min_max_distances[0] + coeff[3+0] / 2;
  dist[1] = min_max_distances[2] + coeff[3+1] / 2;
  dist[2] = min_max_distances[4] + coeff[3+2] / 2;

  // Compute position of the box's geometric center in XYZ
  coeff[0] = dist[0]*coeff[6+0] + dist[1]*coeff[9+0] + dist[2]*coeff[12+0];
  coeff[1] = dist[0]*coeff[6+1] + dist[1]*coeff[9+1] + dist[2]*coeff[12+1];
  coeff[2] = dist[0]*coeff[6+2] + dist[1]*coeff[9+2] + dist[2]*coeff[12+2];
  //coeff[0] = box_centroid_.x + dist[0]*coeff[6+0] + dist[1]*coeff[9+0] + dist[2]*coeff[12+0];
  //coeff[1] = box_centroid_.y + dist[0]*coeff[6+1] + dist[1]*coeff[9+1] + dist[2]*coeff[12+1];
  //coeff[2] = box_centroid_.z + dist[0]*coeff[6+2] + dist[1]*coeff[9+2] + dist[2]*coeff[12+2];
  if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Cluster center x: %g, y: %g, z: %g", coeff[0], coeff[1], coeff[2]);

  // Print info
  if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Dimensions x: %g, y: %g, z: %g",
      coeff[3+0], coeff[3+1], coeff[3+2]);
  if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Direction vectors: \n\t%g %g %g \n\t%g %g %g \n\t%g %g %g",
      coeff[3+3], coeff[3+4], coeff[3+5],
      coeff[3+6], coeff[3+7], coeff[3+8],
      coeff[3+9], coeff[3+10],coeff[3+11]);

  return true;
}

//*/

void getAxesOrientedSurfaces (pcl::PointCloud<PointT> &input_cloud,
                              pcl::PointCloud<pcl::Normal> &normals_cloud,
                              Eigen::Vector3f axis,
                              double epsilon_angle,
                              double plane_threshold,
                              int minimum_plane_inliers,
                              int maximum_plane_iterations,
                              int minimum_size_of_plane_cluster,
                              double plane_inliers_clustering_tolerance,
                              std::vector<pcl::PointCloud<PointT>::Ptr> &planar_surfaces,
                              std::vector<std::string> &planar_surfaces_ids,
                              std::vector<pcl::PointIndices::Ptr> &planar_surfaces_indices,
                              std::vector<pcl::ModelCoefficients::Ptr> &planar_surfaces_coefficients,
                              pcl::visualization::PCLVisualizer &viewer)
{

  // Count number of fitted planes 
  int plane_fit = 0;

  // Stop condition for fitting
  bool stop_planes = false;


  // Count one hundred mishaps
  int counter = 0;

  do
  {
    // Create the segmentation object and declare variables
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> segmentation_of_planes;
    pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients ());

    // Set all the parameters for segmenting vertical planes
    segmentation_of_planes.setOptimizeCoefficients (true);
    segmentation_of_planes.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    segmentation_of_planes.setNormalDistanceWeight (0.05);
    segmentation_of_planes.setMethodType (pcl::SAC_RANSAC);
    segmentation_of_planes.setDistanceThreshold (plane_threshold);
    segmentation_of_planes.setMaxIterations (maximum_plane_iterations);
    segmentation_of_planes.setAxis (axis);
    segmentation_of_planes.setEpsAngle (epsilon_angle);
    segmentation_of_planes.setInputCloud (input_cloud.makeShared());
    segmentation_of_planes.setInputNormals (normals_cloud.makeShared());

    // Obtain the plane inliers and coefficients
    segmentation_of_planes.segment (*plane_inliers, *plane_coefficients);

    if ( verbose )
    {
      ROS_INFO ("Plane has %5d inliers with parameters A = %f B = %f C = %f and D = %f found in maximum %d iterations", (int) plane_inliers->indices.size (), 
          plane_coefficients->values [0], plane_coefficients->values [1], plane_coefficients->values [2], plane_coefficients->values [3], maximum_plane_iterations);
    }

    // Check if the fitted circle has enough inliers in order to be accepted
    if ((int) plane_inliers->indices.size () < minimum_plane_inliers) 
    {
      ROS_ERROR ("NOT ACCEPTED !");

      counter++;

      cerr <<" COUNTER = " << counter << endl ;

      // TODO limit the number of re-fitting for better computation time

      if ( counter == 25 )
      {
        // No need for fitting planes anymore
        stop_planes = true;
      }
    }
    else
    {
      ROS_WARN ("ACCEPTED !");

      // Reset counter
      counter = 0;

      // ----------------------------------- //
      // Start processing the accepted plane //
      // ----------------------------------- //

      // Point cloud of plane inliers
      pcl::PointCloud<PointT>::Ptr pivot_input_cloud (new pcl::PointCloud<PointT> ());

      // Point cloud of plane inliers
      pcl::PointCloud<PointT>::Ptr plane_inliers_cloud (new pcl::PointCloud<PointT> ());

      // Extract the circular inliers from the input cloud
      pcl::ExtractIndices<PointT> extraction_of_plane_inliers;
      // Set point cloud from where to extract
      extraction_of_plane_inliers.setInputCloud (input_cloud.makeShared());
      // Set which indices to extract
      extraction_of_plane_inliers.setIndices (plane_inliers);
      // Return the points which represent the inliers
      extraction_of_plane_inliers.setNegative (false);
      // Call the extraction function
      extraction_of_plane_inliers.filter (*plane_inliers_cloud);
      // Return the remaining points of inliers
      extraction_of_plane_inliers.setNegative (true);
      // Call the extraction function
      extraction_of_plane_inliers.filter (*pivot_input_cloud);

      // UPDATE input_cloud VARIABLE //
      input_cloud = *pivot_input_cloud;

      // Point cloud of plane inliers
      pcl::PointCloud<pcl::Normal>::Ptr pivot_normals_cloud (new pcl::PointCloud<pcl::Normal> ());

      // Extract the normals of plane inliers 
      pcl::ExtractIndices<pcl::Normal> extraction_of_normals;

      // Set normals cloud from where to extract
      extraction_of_normals.setInputCloud (normals_cloud.makeShared());
      // Return the remaining normals
      extraction_of_normals.setNegative (true);
      // Set which indices to extract
      extraction_of_normals.setIndices (plane_inliers);
      // Call the extraction function
      extraction_of_normals.filter (*pivot_normals_cloud);

      // UPDATE input_cloud VARIABLE //
      normals_cloud = *pivot_normals_cloud;

      /*

      // Create ID for visualization
      std::stringstream id_of_plane;
      id_of_plane << "PLANE_" << ros::Time::now();

      // Add point cloud to viewer
      viewer.addPointCloud (plane_inliers_cloud, id_of_plane.str());
      // Set the size of points for cloud
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points, id_of_plane.str()); 

      // Wait or not wait
      if ( step )
      {
        // And wait until Q key is pressed
        viewer.spin ();
      }

      // Remove or not remove the cloud from viewer
      if ( clean )
      {
        // Remove the point cloud data
        viewer.removePointCloud (id_of_plane.str());

        // Wait or not wait
        if ( step )
        {
          // And wait until Q key is pressed
          viewer.spin ();
        }
      }

      */



      ///*

      // Vector of clusters from inliers
      std::vector<pcl::PointIndices> plane_clusters;
      // Build kd-tree structure for clusters
      pcl::KdTreeFLANN<PointT>::Ptr plane_clusters_tree (new pcl::KdTreeFLANN<PointT> ());

      // Instantiate cluster extraction object
      pcl::EuclideanClusterExtraction<PointT> clustering_of_plane_inliers;
      // Set as input the cloud of circle inliers
      clustering_of_plane_inliers.setInputCloud (plane_inliers_cloud);
      // Radius of the connnectivity threshold
      clustering_of_plane_inliers.setClusterTolerance (plane_inliers_clustering_tolerance);
      // Minimum size of clusters
      clustering_of_plane_inliers.setMinClusterSize (minimum_size_of_plane_cluster);
      // Provide pointer to the search method
      clustering_of_plane_inliers.setSearchMethod (plane_clusters_tree);
      // Call the extraction function
      clustering_of_plane_inliers.extract (plane_clusters);

      //*/



      /*

      if ( verbose )
      {
        ROS_WARN ("Plane model has %d clusters where", plane_clusters.size());
        for (int c = 0; c < (int) plane_clusters.size(); c++)
          ROS_WARN ("  Cluster %d has %d points", c, (int) plane_clusters.at(c).indices.size());
      }

      */

      // Point clouds which represent the clusters of the plane inliers
      std::vector<pcl::PointCloud<PointT>::Ptr> plane_clusters_clouds;

      for (int c = 0; c < (int) plane_clusters.size(); c++)
      {
        // Local variables
        pcl::PointIndices::Ptr pointer_of_plane_cluster (new pcl::PointIndices (plane_clusters.at(c)));
        pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);

        // Extract the circular inliers from the input cloud
        pcl::ExtractIndices<PointT> extraction_of_plane_clusters;
        // Set point cloud from where to extract
        extraction_of_plane_clusters.setInputCloud (plane_inliers_cloud);
        // Set which indices to extract
        extraction_of_plane_clusters.setIndices (pointer_of_plane_cluster);
        // Return the points which represent the inliers
        extraction_of_plane_clusters.setNegative (false);
        // Call the extraction function
        extraction_of_plane_clusters.filter (*cluster);

        // Save cluster
        plane_clusters_clouds.push_back (cluster);

        // Save planar surface
        planar_surfaces.push_back (cluster);

        // Save coefficients of plane's planar surface
        planar_surfaces_coefficients.push_back (plane_coefficients);

        // Save indices of planar surfaces
        planar_surfaces_indices.push_back (pointer_of_plane_cluster);

        if ( verbose )
        {
          ROS_INFO ("  Planar surface %d has %d points", c, (int) cluster->points.size());
        }
      }



      for (int c = 0; c < (int) plane_clusters.size(); c++)
      {
        // Create ID for visualization
        std::stringstream id_of_surface;
        id_of_surface << "SURFACE_" << ros::Time::now();

        // Save id of planar surface
        planar_surfaces_ids.push_back (id_of_surface.str());

        // Add point cloud to viewer
        viewer.addPointCloud (plane_clusters_clouds.at(c), id_of_surface.str());

        // Set the size of points for cloud
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points, id_of_surface.str()); 

        // Wait or not wait
        if ( step )
        {
          // And wait until Q key is pressed
          viewer.spin ();
        }

        // Remove or not remove the cloud from viewer
        if ( clean )
        {
          // Remove the point cloud data
          viewer.removePointCloud (id_of_surface.str());
          // Wait or not wait
          if ( step )
          {
            // And wait until Q key is pressed
            viewer.spin ();
          }
        }
      }


      /*

      // Remove the point cloud data
      viewer.removePointCloud (id_of_plane.str());

      */

      // ------------------------------------- //
      // End of processing the accepted circle //
      // ------------------------------------- //

    }

    // number of fitted planes
    plane_fit++;

    // --------------------------------------- //
    // Check for stop condition and print info //
    // --------------------------------------- //

    // Print the number of points left for model fitting
    if ( (int) input_cloud.points.size () < minimum_plane_inliers )
      ROS_ERROR ("%d < %d | Stop !", (int) input_cloud.points.size (), minimum_plane_inliers);
    else
      if ( (int) input_cloud.points.size () > minimum_plane_inliers )
        ROS_INFO ("%d > %d | Continue... ", (int) input_cloud.points.size (), minimum_plane_inliers);
      else
        ROS_INFO ("%d = %d | Continue... ", (int) input_cloud.points.size (), minimum_plane_inliers);

  } while ((int) input_cloud.points.size () > minimum_plane_inliers && stop_planes == false);
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Main routine of the method. Segmentation of point cloud data.
 */
int main (int argc, char** argv)
{

  // --------------------------------------------------------------- //
  // ------------------ Check and parse arguments ------------------ //
  // --------------------------------------------------------------- //

  // Argument check and info about
  if (argc < 2)
  {
    ROS_INFO (" ");
    ROS_INFO ("Syntax is: %s <input>.pcd <options>", argv[0]);
    ROS_INFO ("  where <options> are:");
    ROS_INFO ("    -height_of_floor X                       = Height of floor layer of point cloud data.");
    ROS_INFO ("    -height_of_ceiling X                     = Height of ceiling layer of point cloud data.");
    ROS_INFO ("    -height_of_walls X                       = Minimum height of walls in the point cloud data.");
    ROS_INFO (" ");
    ROS_INFO ("    -epsilon_angle X                         = The maximum allowed difference between the plane normal and the given axis.");
    ROS_INFO ("    -plane_threshold X                       = Distance to the fitted plane model.");
    ROS_INFO ("    -minimum_plane_inliers X                 = Minimum number of inliers of the fitted plane in order to be accepted.");
    ROS_INFO ("    -maximum_plane_iterations X              = Maximum number of interations for fitting the plane model.");
    ROS_INFO (" ");
    ROS_INFO ("    -minimum_size_of_plane_cluster           = Minimum cluster size of plane inliers.");
    ROS_INFO ("    -plane_inliers_clustering_tolerance      = The clustering tolerance of plane inliers");
    ROS_INFO ("    -minimum_size_of_handle_cluster          = Minimum cluster size of handle points.");
    ROS_INFO ("    -handle_clustering_tolerance             = The clustering tolerance of handle points.");
    ROS_INFO (" ");
    ROS_INFO ("    -step B                                  = Wait or not wait.");
    ROS_INFO ("    -clean B                                 = Remove or not remove.");
    ROS_INFO ("    -verbose B                               = Display step by step info.");
    ROS_INFO ("    -size_of_points D                        = Set the size of points");
    ROS_INFO (" ");
    return (-1);
  }

  // Take only the first .pcd file into account
  std::vector<int> pFileIndicesPCD = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (pFileIndicesPCD.size () == 0)
  {
    ROS_ERROR ("No .pcd file given as input!");
    return (-1);
  }

  // Parse the arguments for filtering
  pcl::console::parse_argument (argc, argv, "-height_of_floor", height_of_floor);
  pcl::console::parse_argument (argc, argv, "-height_of_ceiling", height_of_ceiling);
  pcl::console::parse_argument (argc, argv, "-height_of_walls", height_of_walls);

  // Parse arguments for fitting plane models
  pcl::console::parse_argument (argc, argv, "-epsilon_angle", epsilon_angle);
  pcl::console::parse_argument (argc, argv, "-plane_threshold", plane_threshold);
  pcl::console::parse_argument (argc, argv, "-minimum_plane_inliers", minimum_plane_inliers);
  pcl::console::parse_argument (argc, argv, "-maximum_plane_iterations", maximum_plane_iterations);

  // Parse arguments for clustering
  pcl::console::parse_argument (argc, argv, "-minimum_size_of_plane_cluster", minimum_size_of_plane_cluster);
  pcl::console::parse_argument (argc, argv, "-plane_inliers_clustering_tolerance", plane_inliers_clustering_tolerance);
  pcl::console::parse_argument (argc, argv, "-minimum_size_of_handle_cluster", minimum_size_of_handle_cluster);
  pcl::console::parse_argument (argc, argv, "-handle_clustering_tolerance", handle_clustering_tolerance);

  // Parse the arguments for segmenting by color and fixture
  pcl::console::parse_argument (argc, argv, "-cluster_tolerance", cluster_tolerance);
  pcl::console::parse_argument (argc, argv, "-fixture_cluster_tolerance", fixture_cluster_tolerance);
  pcl::console::parse_argument (argc, argv, "-center_radius", center_radius);
  pcl::console::parse_argument (argc, argv, "-init_radius", init_radius);
  pcl::console::parse_argument (argc, argv, "-color_radius", color_radius);

  pcl::console::parse_argument (argc, argv, "-std_limit", std_limit);
  pcl::console::parse_argument (argc, argv, "-min_pts_per_cluster", min_pts_per_cluster);
  pcl::console::parse_argument (argc, argv, "-fixture_min_pts_per_cluster", fixture_min_pts_per_cluster);

  // Parse arguments for visualization
  pcl::console::parse_argument (argc, argv, "-step", step);
  pcl::console::parse_argument (argc, argv, "-clean", clean);
  pcl::console::parse_argument (argc, argv, "-verbose", verbose);
  pcl::console::parse_argument (argc, argv, "-size_of_points", size_of_points);

  // Parsing the optional arguments
  pcl::console::parse_argument (argc, argv, "-find_box_model", find_box_model);
  pcl::console::parse_argument (argc, argv, "-segmentation_by_color_and_fixture", segmentation_by_color_and_fixture);

  // ----------------------------------------------------- //
  // ------------------ Initializations ------------------ //
  // ----------------------------------------------------- //

  // Initialize random number generator
  srand (time(0));

  // Initialize ros time
  ros::Time::init();

  // Declare the timer
  pcl::console::TicToc tt;

  // Starting timer
  tt.tic ();

  if ( verbose )
  {
    // Displaying when the timer starts
    ROS_WARN ("Timer started !");
  }

  // ---------------------------------------------------------------- //
  // ------------------ Visualize point cloud data ------------------ //
  // ---------------------------------------------------------------- //

  // Open a 3D viewer
  pcl::visualization::PCLVisualizer viewer ("3D VIEWER");
  // Set the background of viewer
  viewer.setBackgroundColor (0.0, 0.0, 0.0);
  // Add system coordiante to viewer
  viewer.addCoordinateSystem (0.75f);
  // Parse the camera settings and update the internal camera
  viewer.getCameraParameters (argc, argv);
  // Update camera parameters and render
  viewer.updateCamera ();

  // --------------------------------------------------------------- //
  // ------------------ Load the point cloud data ------------------ //
  // --------------------------------------------------------------- //

  // Input point cloud data
  pcl::PointCloud<PointT>::Ptr input_cloud (new pcl::PointCloud<PointT> ());

  // Auxiliary
  pcl::PointCloud<PointT>::Ptr auxiliary_input_cloud (new pcl::PointCloud<PointT> ());

  // Load point cloud data
  if (pcl::io::loadPCDFile (argv[pFileIndicesPCD[0]], *input_cloud) == -1)
  {
    ROS_ERROR ("Couldn't read file %s", argv[pFileIndicesPCD[0]]);
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int) (input_cloud->points.size ()), argv[pFileIndicesPCD[0]], pcl::getFieldsList (*input_cloud).c_str ());

 
  // Add the input cloud
  viewer.addPointCloud (input_cloud, "INPUT");

  // Color the cloud in white
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "INPUT");

  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points - 1, "INPUT"); 

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Remove or not remove the cloud from viewer
  if ( clean )
  {
    // Remove the point cloud data
    viewer.removePointCloud ("INPUT");

    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }
  }

  // TODO declare a working cloud and always update it 
  // TODO never modify the original input cloud

  // Auxiliary input cloud
  pcl::io::loadPCDFile (argv [pFileIndicesPCD [0]], *auxiliary_input_cloud);

  // Get position of dot in path of file 
  std::string file = argv [pFileIndicesPCD[0]];
  size_t dot = file.find (".");
  size_t slash = file.find_last_of ("/");
  std::string directory = file.substr (slash, dot - slash);

  // TODO insert option for statistical outlier removal -statistical_outlier_removal B alog with its parameters

  /*

  // ------------------------------------------------------------------------ //
  // ------------------ Removal of outliers for input data ------------------ //
  // ------------------------------------------------------------------------ //

  // Filtered point cloud
  pcl::PointCloud<PointT>::Ptr filtered_cloud (new pcl::PointCloud<PointT> ());

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<PointT> sor;
  // Set which point cloud to filter
  sor.setInputCloud (input_cloud);
  // Set number of points for mean distance estimation
  sor.setMeanK (50);
  // Set the standard deviation multiplier threshold
  sor.setStddevMulThresh (1.0);
  // Call the filtering method
  sor.filter (*filtered_cloud);

  ROS_INFO ("Statistical Outlier Removal ! before: %d points | after: %d points | filtered: %d points", input_cloud->points.size (),  filtered_cloud->points.size (), input_cloud->points.size () - filtered_cloud->points.size ());

  // Add the filtered cloud 
  viewer.addPointCloud (filtered_cloud, "FILTERED");
  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points - 1, "FILTERED"); 

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Remove the point cloud data
  viewer.removePointCloud ("INPUT");

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Save the filtered cloud to the working cloud
  *input_cloud = *filtered_cloud;

  */


  
  // The minimum and maximum height of point cloud
  PointT point_with_minimum_height, point_with_maximum_height;
  pcl::getMinMax3D (*input_cloud, point_with_minimum_height, point_with_maximum_height);
  double height_of_cloud = point_with_maximum_height.z - point_with_minimum_height.z;

  height_of_floor = height_of_cloud * height_of_floor;
  height_of_ceiling = height_of_cloud * height_of_ceiling;
  height_of_walls = height_of_cloud * height_of_walls;

  if ( verbose )
  {
    ROS_INFO ("The point with minimum height is (%6.3f,%6.3f,%6.3f)", point_with_minimum_height.x, point_with_minimum_height.y, point_with_minimum_height.z);
    ROS_INFO ("The point with maximum height is (%6.3f,%6.3f,%6.3f)", point_with_maximum_height.x, point_with_maximum_height.y, point_with_maximum_height.z);
    ROS_INFO ("The height of point cloud is %5.3f meters", height_of_cloud);

    ROS_INFO ("Height of floor is %5.3f meters", height_of_floor);
    ROS_INFO ("Height of ceiling is %5.3f meters", height_of_ceiling);
    ROS_INFO ("Height of walls is %5.3f meters", height_of_walls);
  } 



  // -------------------------------------------------------------------- //
  // ------------------ Start with filtering the cloud ------------------ //
  // -------------------------------------------------------------------- //
  
  // Create the filtering object
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName ("z");

  // Point cloud of floor points
  pcl::PointCloud<PointT>::Ptr floor_cloud (new pcl::PointCloud<PointT> ());

  // Set floor limits
  pass.setFilterLimits (point_with_minimum_height.z, point_with_minimum_height.z + height_of_floor);

  // Call the filtering function
  pass.setFilterLimitsNegative (false);
  pass.filter (*floor_cloud);
  pass.setFilterLimitsNegative (true);
  pass.filter (*input_cloud);

  // Create file name for saving
  std::string floor_filename = argv [pFileIndicesPCD [0]];
  floor_filename.insert (dot, "-floor");

  // Save these points to disk
  pcl::io::savePCDFile (floor_filename, *floor_cloud);

  if ( verbose )
  {
    // Show the floor's number of points
    ROS_INFO ("The floor has %d points and was saved to data/floor.pcd", (int) floor_cloud->points.size ());
  }
 
  // Point cloud of ceiling points
  pcl::PointCloud<PointT>::Ptr ceiling_cloud (new pcl::PointCloud<PointT> ());

  // Set ceiling limits
  pass.setFilterLimits (point_with_maximum_height.z - height_of_ceiling, point_with_maximum_height.z);

  // Call the filtering function
  pass.setFilterLimitsNegative (false);
  pass.filter (*ceiling_cloud);
  pass.setFilterLimitsNegative (true);
  pass.filter (*input_cloud);

  // Create file name for saving
  std::string ceiling_filename = argv [pFileIndicesPCD [0]];
  ceiling_filename.insert (dot, "-ceiling");

  // Save these points to disk
  pcl::io::savePCDFile (ceiling_filename, *ceiling_cloud);

  if ( verbose )
  {
    // Show the ceiling's number of points
    ROS_INFO ("The ceiling has %d points and was saved to data/ceiling.pcd", (int) ceiling_cloud->points.size ());
  }

  // Point cloud of walls points
  pcl::PointCloud<PointT>::Ptr walls_cloud (new pcl::PointCloud<PointT> ());

  // Save the cloud with the walls
  *walls_cloud = *input_cloud;

  // Create file name for saving
  std::string walls_filename = argv [pFileIndicesPCD [0]];
  walls_filename.insert (dot, "-walls");

  // Save these points to disk
  pcl::io::savePCDFile (walls_filename, *walls_cloud);

  if ( verbose )
  {
    // Show the walls' number of points
    ROS_INFO ("The walls have %d points and were saved to data/walls.pcd", (int) walls_cloud->points.size ());
  }


 
  // -------------------------------------------------------------- //
  // ------------------ Visulize filtered clouds ------------------ //
  // -------------------------------------------------------------- //

  // Add point cloud to viewer
  viewer.addPointCloud (floor_cloud, "FLOOR");
  // Color the cloud in red
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "FLOOR");
  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points - 1, "FLOOR"); 

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Remove or not remove the cloud from viewer
  if ( clean )
  {
    // Remove the point cloud data
    viewer.removePointCloud ("FLOOR");

    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }
  }

  // Add the input cloud
  viewer.addPointCloud (walls_cloud, "WALLS");
  // Color the cloud in green
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "WALLS");
  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points - 1, "WALLS"); 

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Remove or not remove the cloud from viewer
  if ( clean )
  {
    // Remove the point cloud data
    viewer.removePointCloud ("WALLS");

    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }
  }

  /*

  // Add point cloud to viewer
  viewer.addPointCloud (ceiling_cloud, "CEILING");
  // Color the cloud in blue
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "CEILING");
  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points - 1, "CEILING"); 

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Remove or not remove the cloud from viewer
  if ( clean )
  {
    // Remove the point cloud data
    viewer.removePointCloud ("CEILING");

    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }
  }

  */



  // ------------------------------------------------------ //
  // ------------------ Lose the ceiling ------------------ //
  // ------------------------------------------------------ //

  // Remove the point cloud data
  viewer.removePointCloud ("INPUT");

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Remove the point cloud data
  viewer.removePointCloud ("FLOOR");

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // ------------------------------------------------------------------- //
  // ------------------ Estiamte 3D normals of points ------------------ //
  // ------------------------------------------------------------------- //
 
  // Point cloud of normals
  pcl::PointCloud<pcl::Normal>::Ptr normals_cloud (new pcl::PointCloud<pcl::Normal> ());
  // Build kd-tree structure for normals
  pcl::KdTreeFLANN<PointT>::Ptr normals_tree (new pcl::KdTreeFLANN<PointT> ());

  // Create object for normal estimation
  pcl::NormalEstimation<PointT, pcl::Normal> estimation_of_normals;
  // Provide pointer to the search method
  estimation_of_normals.setSearchMethod (normals_tree);
  // Set for which point cloud to compute the normals
  estimation_of_normals.setInputCloud (input_cloud);
  // Set number of k nearest neighbors to use
  estimation_of_normals.setKSearch (50);
  // Estimate the normals
  estimation_of_normals.compute (*normals_cloud);

  if ( verbose )
  {
    ROS_INFO ("Remaning cloud has %d points", (int) input_cloud->points.size());
    ROS_INFO ("With %d normals of course", (int) normals_cloud->points.size());
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Find Box Model of a point cloud

  Eigen::Vector3f X, Y, Z; 

  if (find_box_model)
  {
    pcl::VoxelGrid<PointT> vgrid;
    vgrid.setLeafSize (0.05, 0.05, 0.05);
    vgrid.setInputCloud (input_cloud);
    pcl::PointCloud<PointT>::Ptr cloud_downsampled (new pcl::PointCloud<PointT> ()); 
//    pcl::PointCloud<PointT> cloud_downsampled;
    vgrid.filter (*cloud_downsampled);
    std::vector<double> coefficients (15, 0.0);

    //bool yes_found_model = find_model (cloud_downsampled.makeShared(), *normals_cloud, coefficients);

    if ( findBoxModel (cloud_downsampled, *normals_cloud, coefficients) )
    {
      ROS_INFO("Box Model found");
      X = Eigen::Vector3f (coefficients [6+0], coefficients [6+1], coefficients [6+2]); 
      Y = Eigen::Vector3f (coefficients [9+0], coefficients [9+1], coefficients [9+2]); 
      Z = Eigen::Vector3f (coefficients[12+0], coefficients[12+1], coefficients[12+2]); 
    }

    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }

    ///*
    // Add the input cloud
    viewer.addPointCloud (cloud_downsampled, "DOWNSAMPLED");
    // Color the cloud in green
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "DOWNSAMPLED");
    // Set the size of points for cloud
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points * 2, "DOWNSAMPLED");

    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }

    // Remove the point cloud data
    viewer.removePointCloud ("DOWNSAMPLED");

    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }
    //*/

    // Create file name for saving
    std::string downsampled_filename = argv [pFileIndicesPCD [0]];
    downsampled_filename.insert (dot, "-downsampled");

    // Save these points to disk
    pcl::io::savePCDFile (downsampled_filename, *cloud_downsampled);
  }
  else 
  {
    // If no vectors, the stick with the unit vectors
    X = Eigen::Vector3f (1.0, 0.0, 0.0); 
    Y = Eigen::Vector3f (0.0, 1.0, 0.0); 
    Z = Eigen::Vector3f (0.0, 0.0, 1.0); 
  }

  /*
  // Already computed axes for PointXYZINormal cloud
  X = Eigen::Vector3f (-0.99613, -0.08719,  0.00000);
  Y = Eigen::Vector3f ( 0.08719, -0.99613,  0.00000);
  Z = Eigen::Vector3f ( 0.00000,  0.00000,  1.00000);
  */

  /*
  // Already computed axes for PointXYRGB cloud
  X = Eigen::Vector3f (-0.02082,  0.99978, -0.00000);
  Y = Eigen::Vector3f (-0.99978, -0.02082,  0.00000);
  Z = Eigen::Vector3f ( 0.00000,  0.00000,  1.00000);
  */

  /*
  // Already computed axes for PointXYZ cloud
  X = Eigen::Vector3f (-0.15565, -0.98777,  0.00000);
  Y = Eigen::Vector3f ( 0.98777, -0.15565,  0.00000);
  Z = Eigen::Vector3f ( 0.00000,  0.00000,  1.00000);
  */

  if ( verbose )
  {
    ROS_INFO ("The furniture surfaces will be detected along the following axes:");
    ROS_INFO ("  X = Eigen::Vector3f (%8.5f, %8.5f, %8.5f)", X[0], X[1], X[2]);
    ROS_INFO ("  Y = Eigen::Vector3f (%8.5f, %8.5f, %8.5f)", Y[0], Y[1], Y[2]);
    ROS_INFO ("  Z = Eigen::Vector3f (%8.5f, %8.5f, %8.5f)", Z[0], Z[1], Z[2]);

    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }
  }

  // ------------------------------------------------------------------------------------- //
  // ------------------ Segment horizontal and vertical planar surfaces ------------------ //
  // ------------------------------------------------------------------------------------- //

  // Point clouds which represent the clusters of the plane inliers
  std::vector<pcl::PointCloud<PointT>::Ptr> planar_surfaces;

  // String ids of planar surfaces
  std::vector<std::string> planar_surfaces_ids;

  // Indices of planar surfaces
  std::vector<pcl::PointIndices::Ptr> planar_surfaces_indices;

  // Coefficients of planar surfaces
  std::vector<pcl::ModelCoefficients::Ptr> planar_surfaces_coefficients;

  // Pivot Auxiliary 
  pcl::PointCloud<PointT>::Ptr pivot_auxiliary_input_cloud (new pcl::PointCloud<PointT> ());

  // Give it some points
  *pivot_auxiliary_input_cloud = *input_cloud;

  ROS_ERROR ("Z axis aligned planes");
//  Eigen::Vector3f Z = Eigen::Vector3f (0.0, 0.0, 1.0); 
//  Eigen::Vector3f Z = Eigen::Vector3f (0.0, 0.0, 1.0); 
  getAxesOrientedSurfaces (*input_cloud, *normals_cloud, Z, epsilon_angle, plane_threshold, minimum_plane_inliers, maximum_plane_iterations, minimum_size_of_plane_cluster, plane_inliers_clustering_tolerance, planar_surfaces, planar_surfaces_ids, planar_surfaces_indices, planar_surfaces_coefficients, viewer);

  ROS_ERROR ("Y axis aligned planes");
//  Eigen::Vector3f Y = Eigen::Vector3f (0.0, 1.0, 0.0); 
//  Eigen::Vector3f Y = Eigen::Vector3f (0.0782345, -0.966764, 0.0);
  getAxesOrientedSurfaces (*input_cloud, *normals_cloud, Y, epsilon_angle, plane_threshold, minimum_plane_inliers, maximum_plane_iterations, minimum_size_of_plane_cluster, plane_inliers_clustering_tolerance, planar_surfaces, planar_surfaces_ids, planar_surfaces_indices, planar_surfaces_coefficients, viewer);

  ROS_ERROR ("X axis aligned planes");
//  Eigen::Vector3f X = Eigen::Vector3f (1.0, 0.0, 0.0); 
//  Eigen::Vector3f X = Eigen::Vector3f (-0.966764, -0.0782345, 0.0);
  getAxesOrientedSurfaces (*input_cloud, *normals_cloud, X, epsilon_angle, plane_threshold, minimum_plane_inliers, maximum_plane_iterations, minimum_size_of_plane_cluster, plane_inliers_clustering_tolerance, planar_surfaces, planar_surfaces_ids, planar_surfaces_indices, planar_surfaces_coefficients, viewer);



  // ------------------------------------------------------------------ //
  // ------------------ Lose the walls ans the floor ------------------ //
  // ------------------------------------------------------------------ //

  // Remove the point cloud data
  viewer.removePointCloud ("FLOOR");

  // Remove the point cloud data
  viewer.removePointCloud ("WALLS");

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }



  // --------------------------------------------------------------------------- //
  // ------------------ Sort out furniture and walls surfaces ------------------ //
  // --------------------------------------------------------------------------- //

  // // // // // //
  // Save all points of planar patches and handles which make up the furniture
  // // // // // //
  pcl::PointCloud<PointT>::Ptr furniture (new pcl::PointCloud<PointT> ());

  // Type of furniture 
  std::vector<std::string> type_of_furniture;

  if ( verbose )
  {
    ROS_INFO (" ");
    ROS_INFO ("THERE ARE %d PLANAR SURFACES", (int) planar_surfaces.size());
    ROS_INFO ("AND THERE ARE %d STRING IDS", (int) planar_surfaces_ids.size());
    ROS_INFO (" ");
  }

  // Point clouds which represent the surfaces of furniture complaints
  std::vector<pcl::PointCloud<PointT>::Ptr> furniture_surfaces;

  // String ids of furniture surfaces
  std::vector<std::string> furniture_surfaces_ids;

  // Point clouds which represent the surfaces of walls
  std::vector<pcl::PointCloud<PointT>::Ptr> surfaces_of_walls;

  // String ids for surfaces of walls
  std::vector<std::string> surfaces_of_walls_ids;

  // Point clouds which represent the horizontal surfaces of furniture
  std::vector<pcl::PointCloud<PointT>::Ptr> horizontal_furniture_surfaces;

  // Point clouds which represent the vertical surfaces of furniture
  std::vector<pcl::PointCloud<PointT>::Ptr> vertical_furniture_surfaces;

  // Vector of ids of projected surfaces
  std::vector<std::string> projs_ids;

  for (int surface = 0; surface < (int) planar_surfaces.size(); surface++)
  {
    // The minimum and maximum height of each planar surface
    PointT point_with_minimum_3D_values, point_with_maximum_3D_values;
    pcl::getMinMax3D (*planar_surfaces.at (surface), point_with_minimum_3D_values, point_with_maximum_3D_values);

    if ( verbose )
    {
      ROS_INFO ("FOR PLANAR SURFACE %2d MIN HEIGHT IS %f AND MAX IS %f METERS", surface, point_with_minimum_3D_values.z, point_with_maximum_3D_values.z);
    }

    if ( point_with_maximum_3D_values.z > height_of_walls )
    {
      // Save surface of wall
      surfaces_of_walls.push_back (planar_surfaces.at (surface));

      // Save id of that surface
      surfaces_of_walls_ids.push_back (planar_surfaces_ids.at (surface));

      // Remove the point cloud data
      viewer.removePointCloud (planar_surfaces_ids.at (surface));

      // Wait or not wait
      if ( step )
      {
        // And wait until Q key is pressed
        viewer.spin ();
      }
    }
    else
    {

      // // // // // //
      // Save all points of planar patches and handles which make up the furniture
      // // // // // //
      *furniture += *planar_surfaces.at (surface);

      // Sort horizontal and vertical pieces of furniture
      if ( std::abs(planar_surfaces_coefficients.at (surface)->values[2]) > std::max (std::abs(planar_surfaces_coefficients.at (surface)->values[0]), std::abs(planar_surfaces_coefficients.at (surface)->values[1])) )
      {
        type_of_furniture.push_back ("HORIZONTAL");
        ROS_WARN ("HORIZONTAL");

        // Save the surface furniture as horizontal
        horizontal_furniture_surfaces.push_back (planar_surfaces.at (surface));
      }
      else
      {
        type_of_furniture.push_back ("VERTICAL");
        ROS_WARN ("VERTICAL");

        // Save the surface furniture as vertical
        vertical_furniture_surfaces.push_back (planar_surfaces.at (surface));
      }

      ROS_INFO ("  planar surfaces indices size: %d ", (int) planar_surfaces_indices.at (surface)->indices.size());
      ROS_INFO ("  table model: [%f, %f, %f, %f]", planar_surfaces_coefficients.at (surface)->values[0], planar_surfaces_coefficients.at (surface)->values[1], planar_surfaces_coefficients.at (surface)->values[2], planar_surfaces_coefficients.at (surface)->values[3]);

      // TODO separate the part with projection of points from the sorting of surfaces

//      pcl::PointCloud<PointT> cloud_projected;
      pcl::PointCloud<PointT>::Ptr cloud_projected (new pcl::PointCloud<PointT> ()); 
      // Project the table inliers using the planar model coefficients    
      pcl::ProjectInliers<PointT> proj_;   
      proj_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
      proj_.setInputCloud (planar_surfaces.at (surface));
      proj_.setModelCoefficients (planar_surfaces_coefficients.at (surface));
      proj_.filter (*cloud_projected);

      // // // // // //
      // Save all points of planar patches and handles which make up the furniture
      // // // // // //
//      *furniture += cloud_projected;
      // pcl::ProjectInliers DOES NOT KEEP RGB INFORMATION

      // Remove the point cloud data
      viewer.removePointCloud (planar_surfaces_ids.at (surface));

      // Wait or not wait
      if ( step )
      {
        // And wait until Q key is pressed
        viewer.spin ();
      }

      // Visualize Projecte Poitns
      std::stringstream id_of_proj;
      id_of_proj << "PROJ_" << ros::Time::now();
      viewer.addPointCloud (cloud_projected, id_of_proj.str());
      // Set the size of points for cloud
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points * 2, id_of_proj.str());

      // Save id of handle
      projs_ids.push_back (id_of_proj.str());

      // Wait or not wait
      if ( step )
      {
        // And wait until Q key is pressed
        viewer.spin ();
      }

      // Save furniture surface
//      furniture_surfaces.push_back (planar_surfaces.at (surface));
      furniture_surfaces.push_back (cloud_projected);

      // Save id of that surface
      furniture_surfaces_ids.push_back (planar_surfaces_ids.at (surface));
    }
  }

  if ( verbose )
  {
    ROS_INFO (" ");
    ROS_INFO ("%d FURNITURE SURFACES", (int) furniture_surfaces.size());
    ROS_INFO ("AND %d SURFACES OF WALLS", (int) surfaces_of_walls.size());
    ROS_INFO (" ");
  }

   ROS_ERROR (" OVER !!! ");
 // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }
   ROS_ERROR (" OVER !!! ");
 // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }
   ROS_ERROR (" OVER !!! ");
 // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }


  // ---------------------------------------------------------------------- //
  // ------------------ Extraction of polygon prism data ------------------ //
  // ---------------------------------------------------------------------- //

  // // // // // //
  // Save all points of furniture fixtures
  // // // // // //
  pcl::PointCloud<PointT> fixtures;

  // Vector of ids of handles
  std::vector<std::string> handle_ids;

  // Vector of ids of handles clusters
  std::vector<std::string> handles_clusters_ids;

  for (int furniture = 0; furniture < (int) furniture_surfaces.size(); furniture++)
  {
    
    // Select only the vertical furniture pieces
    if ( type_of_furniture. at (furniture) == "VERTICAL" )
    { 
 
      ROS_WARN ("VERTICAL");

      ROS_INFO ("Furniture surface %d has %d points", furniture, (int) furniture_surfaces.at (furniture)->points.size ());

      pcl::PointCloud<PointT> cloud_hull;
      
      // Create a Convex Hull representation of the projected inliers
      pcl::ConvexHull<PointT> chull_;  
      chull_.setInputCloud (furniture_surfaces.at (furniture));
      ROS_ERROR (" BEFORE IT WORKS ");
      chull_.reconstruct (cloud_hull);      
      ROS_ERROR (" AFTERWARDS NOT ");

      ROS_INFO ("Convex hull %d has %d data points.", furniture, (int) cloud_hull.points.size ());

      /*
      // Visualize Convex Hulls
      std::stringstream id_of_hull;
      id_of_hull << "HULL_" << ros::Time::now();
      // Visualize hull of cloud 
      viewer.addPointCloud (cloud_hull, id_of_hull.str());
      // Set the size of points for cloud
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points, id_of_hull.str());     
      // Wait or not wait
      if ( step )
      {
      // And wait until Q key is pressed
      viewer.spin ();
      }
      */
      pcl::PointIndices::Ptr cloud_object_indices (new pcl::PointIndices ());

      // ---[ Get the objects on top of the table
      pcl::ExtractPolygonalPrismData<PointT> prism_;
      prism_.setHeightLimits (0.025, 0.100);
      prism_.setInputCloud (auxiliary_input_cloud);
      prism_.setInputPlanarHull (cloud_hull.makeShared());
      prism_.setViewPoint (0.0, 0.0, 1.5);
      prism_.segment (*cloud_object_indices);
      
      ROS_INFO ("For %d the number of object point indices is %d", furniture, (int) cloud_object_indices->indices.size ());

      if ( (int) cloud_object_indices->indices.size () != 0 )
      {
        // Extract handles
        //pcl::PointCloud<PointT> handle_cloud;
        pcl::PointCloud<PointT>::Ptr handle_cloud (new pcl::PointCloud<PointT> ()); 
        pcl::ExtractIndices<PointT> extraction_of_handle_inliers;
        // Set point cloud from where to extract
        extraction_of_handle_inliers.setInputCloud (auxiliary_input_cloud);
        // Set which indices to extract
        extraction_of_handle_inliers.setIndices (cloud_object_indices);
        // Return the points which represent the inliers
        extraction_of_handle_inliers.setNegative (false);
        // Call the extraction function
        extraction_of_handle_inliers.filter (*handle_cloud);

        // // // // // //
        // Save all points of furniture fixtures
        // // // // // //
        //      *furniture += handle_cloud;
        fixtures += *handle_cloud;

        // Visualize handles
        std::stringstream id_of_handle;
        id_of_handle << "HANDLE_" << ros::Time::now();
        // Visualize handle
        viewer.addPointCloud (handle_cloud, id_of_handle.str());
        // Set the size of points for cloud
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points * 3, id_of_handle.str());     
        // Wait or not wait
        if ( step )
        {
          // And wait until Q key is pressed
          viewer.spin ();
        }

        // TODO extract clusters of handle clouds

        // Vector of clusters from handles
        std::vector<pcl::PointIndices> handle_clusters;
        // Build kd-tree structure for clusters
        pcl::KdTreeFLANN<PointT>::Ptr handle_clusters_tree (new pcl::KdTreeFLANN<PointT> ());

        // Instantiate cluster extraction object
        pcl::EuclideanClusterExtraction<PointT> clustering_of_handles;
        // Set as input the cloud of handle
        clustering_of_handles.setInputCloud (handle_cloud);
        // Radius of the connnectivity threshold
        clustering_of_handles.setClusterTolerance (handle_clustering_tolerance);
        // Minimum size of clusters
        clustering_of_handles.setMinClusterSize (minimum_size_of_handle_cluster);
        // Provide pointer to the search method
        clustering_of_handles.setSearchMethod (handle_clusters_tree);
        // Call the extraction function
        clustering_of_handles.extract (handle_clusters);

        // Point clouds which represent the clusters of the handle
        std::vector<pcl::PointCloud<PointT>::Ptr> handle_clusters_clouds;

        // Vector of the indices of handles
        std::vector<pcl::PointIndices::Ptr> handle_indices;

        for (int c = 0; c < (int) handle_clusters.size(); c++)
        {
          // Local variables
          pcl::PointIndices::Ptr pointer_of_handle_cluster (new pcl::PointIndices (handle_clusters.at(c)));
          pcl::PointCloud<PointT>::Ptr cluster_of_handle (new pcl::PointCloud<PointT>);

          // Extract handle points from the input cloud
          pcl::ExtractIndices<PointT> extraction_of_handle_clusters;
          // Set point cloud from where to extract
          extraction_of_handle_clusters.setInputCloud (handle_cloud);
          // Set which indices to extract
          extraction_of_handle_clusters.setIndices (pointer_of_handle_cluster);
          // Return the points which represent the inliers
          extraction_of_handle_clusters.setNegative (false);
          // Call the extraction function
          extraction_of_handle_clusters.filter (*cluster_of_handle);

          // Fit a line to the cluster of handle //

          // TODO try to cluster the inleirs of line handles, usefull for the cases where handles are clustered togheter due to noise

          // Create the segmentation object and declare variables
          pcl::SACSegmentation<PointT> segmentation_of_line;
          pcl::PointIndices::Ptr line_inliers (new pcl::PointIndices ());
          pcl::ModelCoefficients::Ptr line_coefficients (new pcl::ModelCoefficients ());

          // Set all the parameters for segmenting vertical planes
          segmentation_of_line.setOptimizeCoefficients (true);
          segmentation_of_line.setModelType (pcl::SACMODEL_LINE);
          //segmentation_of_line.setNormalDistanceWeight (0.05);
          segmentation_of_line.setMethodType (pcl::SAC_RANSAC);
          segmentation_of_line.setDistanceThreshold (0.025);
          segmentation_of_line.setMaxIterations (maximum_plane_iterations);
          //segmentation_of_line.setAxis (axis);
          segmentation_of_line.setEpsAngle (epsilon_angle);
          segmentation_of_line.setInputCloud (cluster_of_handle);
          //segmentation_of_line.setInputNormals (normals_cloud.makeShared());

          // Obtain the line inliers and coefficients
          segmentation_of_line.segment (*line_inliers, *line_coefficients);

          if ( verbose )
          {
            ROS_INFO ("Line has %3d inliers with parameters P1 = (%6.3f,%6.3f,%6.3f) and P2 = (%6.3f,%6.3f,%6.3f) found in maximum %d iterations",
                (int) line_inliers->indices.size (), line_coefficients->values [0], line_coefficients->values [1], line_coefficients->values [2], line_coefficients->values [3],  line_coefficients->values [4], line_coefficients->values [5], maximum_plane_iterations);
          }

          // Check if the fitted line has enough inliers in order to be accepted
          if ((int) line_inliers->indices.size () < 10 /*minimum_size_of_handle_cluster*/)
          {
            ROS_WARN ("NOT ACCEPTED !");
          }
          else
          {
            ROS_WARN ("ACCEPTED !");

            // ---------------------------- //
            // Start the extraction process //
            // ---------------------------- //

            // Point cloud of line inliers
            pcl::PointCloud<PointT>::Ptr line_inliers_cloud (new pcl::PointCloud<PointT> ());

            // Extract the circular inliers from the input cloud
            pcl::ExtractIndices<PointT> extraction_of_line;
            // Set point cloud from where to extract
            extraction_of_line.setInputCloud (cluster_of_handle);
            // Set which indices to extract
            extraction_of_line.setIndices (line_inliers);

            // Return the points which represent the inliers
            extraction_of_line.setNegative (false);
            // Call the extraction function
            extraction_of_line.filter (*line_inliers_cloud);

            /*

               ROS_INFO ("Line has %d inliers", line_inliers_cloud->points.size());
               ROS_INFO ("%d points remain after extraction", filtered_cloud->points.size ());

            */

            // --------------------------- //
            // Start visualization process //
            // --------------------------- //

            // Create id for visualization
            std::stringstream id_of_line;
            id_of_line << "LINE_" << ros::Time::now();

            // Add point cloud to viewer
            viewer.addPointCloud (line_inliers_cloud, id_of_line.str());
            // Set the size of points for cloud
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points * 4, id_of_line.str()); 

            // Wait or not wait
            if ( step )
            {
              // And wait until Q key is pressed
              viewer.spin ();
            }

            // Save id of handles clusters
            handles_clusters_ids.push_back (id_of_line.str());

          }

          // // // // // //
          // Save all points of furniture fixtures
          // // // // // //
          //        *furniture += *cluster_of_handle;
          //        fixtures += *cluster_of_handle;

          if ( verbose )
          {
            ROS_INFO ("  Cluster of handle %d has %d points", c, (int) cluster_of_handle->points.size());
          }

          // Create id for visualization
          std::stringstream id_of_handle_cluster;
          id_of_handle_cluster << "HANDLE_CLUSTER_" << ros::Time::now();
          /*
          // Add point cloud to viewer
          viewer.addPointCloud (cluster_of_handle, id_of_handle_cluster.str());
          // Set the size of points for cloud
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points * 4, id_of_handle_cluster.str()); 

          // Wait or not wait
          if ( step )
          {
          // And wait until Q key is pressed
          viewer.spin ();
          }

          // Remove or not remove the cloud from viewer
          if ( clean )
          {
          // Remove the point cloud data
          viewer.removePointCloud (id_of_handle.str());
          // Wait or not wait
          if ( step )
          {
          // And wait until Q key is pressed
          viewer.spin ();
          }
          }
          */
          // Save cluster of handle
          handle_clusters_clouds.push_back (cluster_of_handle);

          // Save indices of handles
          handle_indices.push_back (pointer_of_handle_cluster);

          // Save id of handle
          handle_ids.push_back (id_of_handle.str());

          //  OUT OF SERVICE BEACUSE OF THE FITTING OF LINES FOR FIXTURES 
          //        // Save id of handles clusters
          //        handles_clusters_ids.push_back (id_of_line.str());
        }

        // Remove the cloud of handle
        viewer.removePointCloud (id_of_handle.str());
      }
    }
  }

  /*

  // // // // // //
  // Save cloud with furniture to disk
  // // // // // //

  // Create file name for saving
  std::string furniture_filename = argv [pFileIndicesPCD [0]];
  furniture_filename.insert (dot, "-furniture");

  // Save these points to disk
  pcl::io::savePCDFile (furniture_filename, *furniture);

  // // // // // //
  // Save cloud with fixtures to disk
  // // // // // //

  // Create file name for saving
  std::string fixtures_filename = argv [pFileIndicesPCD [0]];
  fixtures_filename.insert (dot, "-fixtures");

  // Save these points to disk
  pcl::io::savePCDFile (fixtures_filename, fixtures);

  */

  /*

  double red24bit = getRGB (1.0, 0.0, 0.0);
  ROS_INFO (" red24bit = %.50f ", red24bit);

  for (int p = 0; p < (int) fixtures.points.size (); p++)
  {
    ROS_INFO (" RGB %.50f ", fixtures.points[p].rgb);

    ROS_INFO (" RGB %f ", floor (fixtures.points[p].rgb * pow(10, 40)));

    if ( furniture->points[p].rgb == red24bit )
    {
      ROS_INFO (" RGB %.100f ", fixtures.points[p].rgb);
    }
  }

  */

  // // // // // //
  // Save furniture and fixtures to disk
  // // // // // //

  pcl::PointCloud<PointT>::Ptr furniture_and_fixtures (new pcl::PointCloud<PointT> ());
  *furniture_and_fixtures += *furniture;
  *furniture_and_fixtures += fixtures;

  // Create file name for saving
  std::string furniture_and_fixtures_filename = argv [pFileIndicesPCD [0]];
  furniture_and_fixtures_filename.insert (dot, "-furniture_and_fixtures");

  // Save these points to disk
  pcl::io::savePCDFile (furniture_and_fixtures_filename, *furniture_and_fixtures);

  if ( verbose )
  {
    ROS_WARN (" ");
    ROS_WARN ("FURNITURE HAS %d POINTS", (int) furniture->points.size());
    ROS_WARN ("FIXTURES HAVE %d POINTS", (int) fixtures.points.size());
    ROS_WARN (" ");
  }

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  /*

  ///////////////////////////////////////////////////////////////////
  // Clean the 3D viewer for the segmentation by color and fixture //
  //////////////////////////////////////////////////////////////////

  for (int id = 0; id < (int) handles_clusters_ids.size (); id++)
  {
    // Remove the cloud of handle
    viewer.removePointCloud (handles_clusters_ids.at(id));

    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }
  }

  for (int id = 0; id < (int) projs_ids.size (); id++)
  {
    // Remove the cloud of handle
    viewer.removePointCloud (projs_ids.at(id));

    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }
  }

  ////////////////////////////////
  // The 3D viewer is now clean //
  ////////////////////////////////

  */

  /*

  if ( verbose )
  {
    ROS_INFO (" ");
    ROS_INFO ("THERE ARE %d FURNITURE SURFACES OUT OF WHICH", (int) furniture_surfaces.size());
    ROS_INFO ("%d ARE HORIZONTAL AND", (int) horizontal_furniture_surfaces.size());
    ROS_INFO ("%d ARE VERTICAL", (int) indices_of_points_on_the_surfaces.size());
    ROS_INFO (" ");
  }

  */

  /*

  // Add the input cloud
  viewer.addPointCloud (furniture_and_fixtures, "FURNITURE_AND_FIXTURES");

  // Color the cloud in white
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "FURNITURE_AND_FIXTURES");

  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points, "FURNITURE_AND_FIXTURES"); 

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Remove or not remove the cloud from viewer
  if ( clean )
  {
    // Remove the point cloud data
    viewer.removePointCloud ("FURNITURE_AND_FIXTURES");

    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }
  }

  int number_of_furniture_points = furniture->points.size ();
 
  // Declare the indices of furniture points
  pcl::PointIndices furniture_indices;

  for (int idx = 0; idx < number_of_furniture_points; idx++)
    furniture_indices.indices.push_back (idx);

  // Pointer of the indices of furniture points
  pcl::PointIndices::Ptr furniture_indices_pointer (new pcl::PointIndices (furniture_indices));

  // Declare the indices of fixture points
  pcl::PointIndices fixtures_indices;

  for (int idx = number_of_furniture_points; idx < (int) furniture_and_fixtures->points.size(); idx++)
    fixtures_indices.indices.push_back (idx);

  // Pointer of the indices of fixture points
  pcl::PointIndices::Ptr fixtures_indices_pointer (new pcl::PointIndices (fixtures_indices));

  if ( verbose )
  {
    ROS_INFO ("The number of furniture points is %d", number_of_furniture_points);
    ROS_INFO ("Size of furniture indices is %d", (int) furniture_indices.indices.size ());
    ROS_INFO ("Size of fixture %d", (int) fixtures_indices.indices.size ());
  }

  */

  /*

  // Clusters segmented by color and fixture
  std::vector<pcl::PointIndices> clusters;

  // Create the object for segmenting by color and fixture
  dos_pcl::DoorDetectionByColorAndFixture<PointT> segmentation_by_color_and_fixture;

  segmentation_by_color_and_fixture.setInputCloud (furniture_and_fixtures);
  segmentation_by_color_and_fixture.setIndices (furniture_indices_pointer);
  segmentation_by_color_and_fixture.setFixtureIndices (fixtures_indices_pointer);

  segmentation_by_color_and_fixture.setClusterTolerance (cluster_tolerance);
  segmentation_by_color_and_fixture.setFixtureClusterTolerance (fixture_cluster_tolerance);
  segmentation_by_color_and_fixture.setCenterRadius (center_radius);
  segmentation_by_color_and_fixture.setInitRadius (init_radius);
  segmentation_by_color_and_fixture.setColorRadius (color_radius);

  segmentation_by_color_and_fixture.setSTDLimit (std_limit);
  segmentation_by_color_and_fixture.setMinClusterSize (min_pts_per_cluster);
  segmentation_by_color_and_fixture.setFixtureMinClusterSize (fixture_min_pts_per_cluster);

  // Extract the clusters by color and fixture
  segmentation_by_color_and_fixture.extract (clusters);

  // Save the clouds which represet the individual clusters
  std::vector<pcl::PointCloud<PointT>::Ptr > clusters_clouds;

  for (int c = 0; c < (int) clusters.size(); c++)
  {
    pcl::PointCloud<PointT>::Ptr cluster_cloud (new pcl::PointCloud<PointT>);
    pcl::PointIndices::Ptr cluster_pointer (new pcl::PointIndices (clusters.at (c)));

    // Extract handle points from the input cloud
    pcl::ExtractIndices<PointT> extraction_of_clusters;
    // Set point cloud from where to extract
    extraction_of_clusters.setInputCloud (furniture_and_fixtures);
    // Set which indices to extract
    extraction_of_clusters.setIndices (cluster_pointer);
    // Return the points which represent the inliers
    extraction_of_clusters.setNegative (false);
    // Call the extraction function
    extraction_of_clusters.filter (*cluster_cloud);

    // Save the clouds which represet the individual clusters
    clusters_clouds.push_back (cluster_cloud);

    if ( verbose )
    {
      ROS_INFO ("  Cluster %d has %5d points", c, (int) cluster_cloud->points.size());
    }

    // Create id for visualization
    std::stringstream id_of_cluster;
    id_of_cluster << "CLUSTER_" << ros::Time::now();

    // Add point cloud to viewer
    viewer.addPointCloud (cluster_cloud, id_of_cluster.str());

    // Set the size of points for cloud
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points + 1, id_of_cluster.str()); 

    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }

    // Remove or not remove the cloud from viewer
    if ( clean )
    {
      // Remove the point cloud data
      viewer.removePointCloud (id_of_cluster.str());
      // Wait or not wait
      if ( step )
      {
        // And wait until Q key is pressed
        viewer.spin ();
      }
    }
  }

  */

  if ( verbose )
  {
    // Displaying the overall time
    ROS_WARN ("Finished in %5.3g [s] !", tt.toc ());
  }

  // And wait until Q key is pressed
  viewer.spin ();

  return (0);
}

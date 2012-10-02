/*
 * Copyright (c) 2011, Lucian Cosmin Goron <goron@cs.tum.edu>
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
#include "terminal_tools/parse.h"

// pcl dependencies
#include "pcl/io/pcd_io.h"
#include "pcl/features/normal_3d.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/extract_polygonal_prism_data.h"

// pcl visualization dependencies
#include "pcl_visualization/pcl_visualizer.h"



// Definition of point type 
typedef pcl::PointXYZ PointT;



// Segmentation's Parameters
double epsilon_angle = 0.010; /// [radians]
double plane_threshold = 0.100; /// [meters]
int minimum_plane_inliers = 1000; /// [points]
int maximum_plane_iterations = 1000; /// [iterations]

// Visualization's Parameters
bool step = false;
bool clean = false;
bool verbose = false;
int size_of_points = 1;



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Main routine of the method. Segmentation of planar surface and handle.
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
    ROS_INFO ("  where <options> are: ");
    ROS_INFO ("    -epsilon_angle X                        = ");
    ROS_INFO ("    -plane_threshold X                      = ");
    ROS_INFO ("    -minimum_plane_inliers X                = ");
    ROS_INFO ("    -maximum_plane_iterations X             = ");
    ROS_INFO (" ");
    ROS_INFO ("    -step B                                 = wait or not wait");
    ROS_INFO ("    -clean B                                = remove or not remove");
    ROS_INFO ("    -verbose B                              = display step by step info");
    ROS_INFO ("    -size_of_points D                       = set the size of points");
    ROS_INFO (" ");
    return (-1);
  }

  // Take only the first .pcd file into account
  std::vector<int> pFileIndicesPCD = terminal_tools::parse_file_extension_argument (argc, argv, ".pcd");
  if (pFileIndicesPCD.size () == 0)
  {
    ROS_ERROR ("No .pcd file given as input!");
    return (-1);
  }

  // Parse arguments for fitting plane models
  terminal_tools::parse_argument (argc, argv, "-epsilon_angle", epsilon_angle);
  terminal_tools::parse_argument (argc, argv, "-plane_threshold", plane_threshold);
  terminal_tools::parse_argument (argc, argv, "-minimum_plane_inliers", minimum_plane_inliers);
  terminal_tools::parse_argument (argc, argv, "-maximum_plane_iterations", maximum_plane_iterations);

  // Parse arguments for visualization
  terminal_tools::parse_argument (argc, argv, "-step", step);
  terminal_tools::parse_argument (argc, argv, "-clean", clean);
  terminal_tools::parse_argument (argc, argv, "-verbose", verbose);
  terminal_tools::parse_argument (argc, argv, "-size_of_points", size_of_points);

  // ----------------------------------------------------- //
  // ------------------ Initializations ------------------ //
  // ----------------------------------------------------- //

  // Initialize random number generator
  srand (time(0));

  // Initialize ros time
  ros::Time::init();

  // Declare the timer
  terminal_tools::TicToc tt;

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
  pcl_visualization::PCLVisualizer viewer ("3D VIEWER");
  // Set the background of viewer
  viewer.setBackgroundColor (1.0, 1.0, 1.0);
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
 
  // Load point cloud data
  if (pcl::io::loadPCDFile (argv[pFileIndicesPCD[0]], *input_cloud) == -1)
  {
    ROS_ERROR ("Couldn't read file %s", argv[pFileIndicesPCD[0]]);
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int) (input_cloud->points.size ()), argv[pFileIndicesPCD[0]], pcl::getFieldsList (*input_cloud).c_str ());

  // Add the input cloud
  viewer.addPointCloud (*input_cloud, "INPUT");

  // Color the cloud in white
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "INPUT");

  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points - 1, "INPUT"); 

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

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

  // -------------------------------------------------------------------- //
  // ------------------ Declare working data structure ------------------ //
  // -------------------------------------------------------------------- //

  // Working point cloud data
  pcl::PointCloud<PointT>::Ptr working_cloud (new pcl::PointCloud<PointT> ());

  // Never modify the original point cloud data 
  *working_cloud = *input_cloud;

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
  estimation_of_normals.setInputCloud (working_cloud);
  // Set number of k nearest neighbors to use
  estimation_of_normals.setKSearch (50);
  // Estimate the normals
  estimation_of_normals.compute (*normals_cloud);

  if ( verbose )
  {
    ROS_INFO ("Remaning cloud has %d points", (int) working_cloud->points.size());
    ROS_INFO ("With %d normals of course", (int) normals_cloud->points.size());
  }

  // ------------------------------------------------------------ //
  // ------------------ Segment planar surface ------------------ //
  // ------------------------------------------------------------ //

  // Create the segmentation object and declare variables
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> segmentation_of_plane;
  pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices ());
  pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients ());

  // Set all the parameters for segmenting vertical planes
  segmentation_of_plane.setOptimizeCoefficients (true);
  segmentation_of_plane.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  segmentation_of_plane.setNormalDistanceWeight (0.05);
  segmentation_of_plane.setMethodType (pcl::SAC_RANSAC);
  segmentation_of_plane.setDistanceThreshold (plane_threshold);
  segmentation_of_plane.setMaxIterations (maximum_plane_iterations);
//  Eigen::Vector3f X = Eigen::Vector3f (-0.966764, -0.0782345, 0.0);
//  Eigen::Vector3f Y = Eigen::Vector3f (0.0782345, -0.966764, 0.0);
//  segmentation_of_plane.setAxis ();
  segmentation_of_plane.setEpsAngle (epsilon_angle);
  segmentation_of_plane.setInputCloud (working_cloud);
  segmentation_of_plane.setInputNormals (normals_cloud);

  // Obtain the plane inliers and coefficients
  segmentation_of_plane.segment (*plane_inliers, *plane_coefficients);

  if ( verbose )
  {
    ROS_INFO ("Plane has %5d inliers with parameters A = %f B = %f C = %f and D = %f found in maximum %d iterations", (int) plane_inliers->indices.size (), 
        plane_coefficients->values [0], plane_coefficients->values [1], plane_coefficients->values [2], plane_coefficients->values [3], maximum_plane_iterations);
  }

  // Check if the fitted circle has enough inliers in order to be accepted
  if ((int) plane_inliers->indices.size () < minimum_plane_inliers) 
  {
    if ( verbose )
    {
      ROS_ERROR ("NOT ACCEPTED !");
    }
  }
  else
  {
    if ( verbose )
    {
      ROS_WARN ("ACCEPTED !");
    }

    // ------------------------------------------------------------------------

    // Point cloud of plane inliers
    pcl::PointCloud<PointT>::Ptr plane_cloud (new pcl::PointCloud<PointT> ());

    // Extract the circular inliers from the input cloud
    pcl::ExtractIndices<PointT> extraction_of_inliers;
    extraction_of_inliers.setInputCloud (working_cloud);
    extraction_of_inliers.setIndices (plane_inliers);
    extraction_of_inliers.setNegative (false);
    extraction_of_inliers.filter (*plane_cloud);
    extraction_of_inliers.setNegative (true);
    extraction_of_inliers.filter (*working_cloud);

    // Extract the normals of plane inliers 
    pcl::ExtractIndices<pcl::Normal> extraction_of_normals;
    extraction_of_normals.setInputCloud (normals_cloud);
    extraction_of_normals.setIndices (plane_inliers);
    extraction_of_normals.setNegative (true);
    extraction_of_normals.filter (*normals_cloud);

    if ( verbose )
    {
      ROS_INFO ("Remaning cloud has %d points", (int) working_cloud->points.size());
      ROS_INFO ("With %d normals of course", (int) normals_cloud->points.size());
    }

    // Create id for visualization
    std::stringstream id_of_plane;
    id_of_plane << "PLANE_" << ros::Time::now ();

    // Add point cloud to viewer
    viewer.addPointCloud (*plane_cloud, id_of_plane.str());

    // Set the size of points for cloud
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points, id_of_plane.str()); 

    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }

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

    // Create id for saving
    std::string file_of_plane = argv [pFileIndicesPCD[0]];
    size_t dot_of_plane = file_of_plane.find (".");
    file_of_plane.insert (dot_of_plane, "-plane");

    // Save these points to disk
    pcl::io::savePCDFile (file_of_plane, *plane_cloud);

    // ------------------------------------------------------------------------

    // Declare the projected point cloud
    pcl::PointCloud<PointT> projected_cloud;

    // Project the table inliers using the planar model coefficients    
    pcl::ProjectInliers<PointT> projection_on_plane;   
    projection_on_plane.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    projection_on_plane.setInputCloud (plane_cloud);
    projection_on_plane.setModelCoefficients (plane_coefficients);
    projection_on_plane.filter (projected_cloud);

    if ( verbose )
    {
      ROS_INFO ("Furniture surface has %d points", (int) projected_cloud.points.size ());
    }

    // Create id for visualization
    std::stringstream id_of_projected;
    id_of_projected << "PROJECTED_" << ros::Time::now ();

    // Add point cloud to viewer
    viewer.addPointCloud (projected_cloud, id_of_projected.str());

    // Set the size of points for cloud
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points, id_of_projected.str());

    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }

    // Create id for saving
    std::string file_of_projected = argv [pFileIndicesPCD[0]];
    size_t dot_of_projected = file_of_projected.find (".");
    file_of_projected.insert (dot_of_projected, "-projected");

    // Save these points to disk
    pcl::io::savePCDFile (file_of_projected, projected_cloud);

    // ------------------------------------------------------------------------

    // Declare the hull of cloud
    pcl::PointCloud<PointT> hull_cloud;

    // Create a Convex Hull representation of the projected inliers
    pcl::ConvexHull<PointT> hull;  
    hull.setInputCloud (projected_cloud.makeShared ());
    ROS_ERROR (" BEFORE IT WORKS ");
    hull.reconstruct (hull_cloud);      
    ROS_ERROR (" AFTERWARDS NOT ");

    if ( verbose )
    {
      ROS_INFO ("Convex hull has %d points.", (int) hull_cloud.points.size ());
    }

    // Create id for visualization
    std::stringstream id_of_hull;
    id_of_hull << "HULL_" << ros::Time::now ();

    // Add point cloud to viewer
    viewer.addPointCloud (hull_cloud, id_of_hull.str());

    // Set the size of points for cloud
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points, id_of_hull.str());

    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }

    if ( clean )
    {
      // Remove the point cloud data
      viewer.removePointCloud (id_of_hull.str());

      // Wait or not wait
      if ( step )
      {
        // And wait until Q key is pressed
        viewer.spin ();
      } 
    }

    // Create id for saving
    std::string file_of_hull = argv [pFileIndicesPCD[0]];
    size_t dot_of_hull = file_of_hull.find (".");
    file_of_hull.insert (dot_of_hull, "-hull");

    // Save these points to disk
    pcl::io::savePCDFile (file_of_hull, hull_cloud);

    // ------------------------------------------------------------------------

    // Declare the indices od handle
    pcl::PointIndices::Ptr handle_indices (new pcl::PointIndices ());

    //  Get the objects on top of surface
    pcl::ExtractPolygonalPrismData<PointT> prism;
    prism.setHeightLimits (0.025, 0.100);
    prism.setInputCloud (input_cloud);
    prism.setInputPlanarHull (hull_cloud.makeShared());
    prism.setViewPoint (0.0, 0.0, 1.5);
    prism.segment (*handle_indices);

    if ( verbose )
    {
      ROS_INFO ("The number of handle indices %d", (int) handle_indices->indices.size ());
    }

    // Declare the cloud which represents the handle
    pcl::PointCloud<PointT> handle_cloud;

    // Extract the handle 
    pcl::ExtractIndices<PointT> extraction_of_handle;
    extraction_of_handle.setInputCloud (input_cloud);
    extraction_of_handle.setIndices (handle_indices);
    extraction_of_handle.setNegative (false);
    extraction_of_handle.filter (handle_cloud);

    // Create id for visualization
    std::stringstream id_of_handle;
    id_of_handle << "HANDLE_" << ros::Time::now ();

    // Add point cloud to viewer
    viewer.addPointCloud (handle_cloud, id_of_handle.str());

    // Set the size of points for cloud
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points, id_of_handle.str());

    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }

    // Create id for saving
    std::string file_of_handle = argv [pFileIndicesPCD[0]];
    size_t dot_of_handle = file_of_handle.find (".");
    file_of_handle.insert (dot_of_handle, "-handle");

    // Save these points to disk
    pcl::io::savePCDFile (file_of_handle, handle_cloud);

    // ------------------------------------------------------------------------

  }

  if ( verbose )
  {
    // Displaying the overall time
    ROS_WARN ("Finished in %5.3g [s] !", tt.toc ());
  }

  // And wait until Q key is pressed
  viewer.spin ();

  return (0);
}

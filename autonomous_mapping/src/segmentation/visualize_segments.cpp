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
////#include "pcl/features/normal_3d.h"
//#include "pcl/surface/convex_hull.h"
//#include "pcl/filters/extract_indices.h"
//#include "pcl/filters/project_inliers.h"
//#include "pcl/sample_consensus/method_types.h"
//#include "pcl/segmentation/sac_segmentation.h"
//#include "pcl/segmentation/extract_clusters.h"
//#include "pcl/segmentation/extract_polygonal_prism_data.h"

// pcl visualization dependencies
#include "pcl_visualization/pcl_visualizer.h"



// Definition of point type
typedef pcl::PointXYZ PointT;



// Visualization's Parameters
bool step = false;
bool clean = false;
bool verbose = false;
int size_of_hull = 10;
int size_of_projected = 1;



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
    ROS_INFO ("    -step B                                 = wait or not wait");
    ROS_INFO ("    -clean B                                = remove or not remove");
    ROS_INFO ("    -verbose B                              = display step by step info");
    ROS_INFO ("    -size_of_hull D                         = set the size of hull points");
    ROS_INFO ("    -size_of_projected D                    = set the size of projected points");
    ROS_INFO (" ");
    return (-1);
  }

  // Take only the first .pcd file into account
  std::vector<int> pFileIndicesPCD = terminal_tools::parse_file_extension_argument (argc, argv, ".pcd");
  if (pFileIndicesPCD.size () == 0)
  {
    ROS_ERROR ("No .pcd files given as input !");
    return (-1);
  }

  // Parse arguments for visualization
  terminal_tools::parse_argument (argc, argv, "-step", step);
  terminal_tools::parse_argument (argc, argv, "-clean", clean);
  terminal_tools::parse_argument (argc, argv, "-verbose", verbose);
  terminal_tools::parse_argument (argc, argv, "-size_of_hull", size_of_hull);
  terminal_tools::parse_argument (argc, argv, "-size_of_projected", size_of_projected);

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

  // ----------------------------------------------------------------- //
  // ------------------ Visualize point clouds data ------------------ //
  // ----------------------------------------------------------------- //

  // Open a 3D viewer
  pcl_visualization::PCLVisualizer viewer ("3D VIEWER");
  // Set the background of viewer
  viewer.setBackgroundColor (1.0, 1.0, 1.0);
  // Parse the camera settings and update the internal camera
  viewer.getCameraParameters (argc, argv);
  // Update camera parameters and render
  viewer.updateCamera ();

  // ---------------------------------------------------------------- //
  // ------------------ Load the point clouds data ------------------ //
  // ---------------------------------------------------------------- //

  // Cloud of projected clouds
  pcl::PointCloud<PointT>::Ptr cloud_of_projected (new pcl::PointCloud<PointT> ());

  // Cloud of hull clouds
  pcl::PointCloud<PointT>::Ptr cloud_of_hull (new pcl::PointCloud<PointT> ());

  // Vector of projected clouds
  std::vector<pcl::PointCloud<PointT>::Ptr> vector_of_projected;

  // Vector of hull clouds
  std::vector<pcl::PointCloud<PointT>::Ptr> vector_of_hull;

  for (int cloud = 0; cloud < (int) pFileIndicesPCD.size (); cloud++)
  {
    // Set the name of file
    std::string file = argv [pFileIndicesPCD [cloud]];

    if ( ( (int) file.find ("projected") != -1 ) || ( (int) file.find ("hull") != -1 ) )
    {
      // Input poin cloud data
      pcl::PointCloud<PointT>::Ptr input (new pcl::PointCloud<PointT> ());

      // Load point cloud data
      if (pcl::io::loadPCDFile (argv [pFileIndicesPCD [cloud]], *input) == -1)
      {
        ROS_ERROR ("Couldn't read file %s", argv [pFileIndicesPCD [cloud]]);
        return (-1);
      }
      ROS_INFO ("Loaded %5d data points from %s with the following fields: %s", (int) (input->points.size ()), argv [pFileIndicesPCD [cloud]], pcl::getFieldsList (*input).c_str ());

      if ( (int) file.find ("projected") != -1 )
      {
        // Add input to projected cloud
        *cloud_of_projected += *input;

        // Add input to projected vector
        vector_of_projected.push_back (input);
      }
      else if ( (int) file.find ("hull") != -1 )
      {
        // Add input to hull cloud
        *cloud_of_hull += *input;

        // Add input to hull vector
        vector_of_hull.push_back (input);
      }
    }
  }

  // ---------------------------------------------------- //
  // ------------------ Visualizations ------------------ //
  // ---------------------------------------------------- //

/*
  // Add the input cloud
  viewer.addPointCloud (*cloud_of_projected, "PROJECTED");

  // Color the cloud in white
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "PROJECTED");

  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size_of_projected, "PROJECTED");

  // Wait or not wait
  if ( step )
  {
  // And wait until Q key is pressed
  viewer.spin ();
  }

*/

///*

  for (int cloud = 0; cloud < (int) vector_of_projected.size (); cloud++)
  {
    // Create id for visualization
    std::stringstream id_of_projected;
    id_of_projected << "PROJECTED_" << ros::Time::now();

    // Visualize handle
    viewer.addPointCloud (*vector_of_projected.at (cloud), id_of_projected.str());

    // Set the size of points for cloud
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size_of_projected, id_of_projected.str());     

  }

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

//*/

///*

  // Add the input cloud
  viewer.addPointCloud (*cloud_of_hull, "HULL");

  // Color the cloud in white
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "HULL");

  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size_of_hull, "HULL");

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

//*/

/*

  for (int cloud = 0; cloud < (int) vector_of_hull.size (); cloud++)
  {
    // Create id for visualization
    std::stringstream id_of_hull;
    id_of_hull << "HULL_" << ros::Time::now();

    // Visualize handle
    viewer.addPointCloud (*vector_of_hull.at (cloud), id_of_hull.str());

    // Set the size of points for cloud
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size_of_hull, id_of_hull.str());     

  }

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

*/

  if ( verbose )
  {
    // Displaying the overall time
    ROS_WARN ("Finished in %5.3g [s] !", tt.toc ());
  }

  return (0);
}

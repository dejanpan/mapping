
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

#ifndef CLOUD_ALGOS_DEPTH_IMAGE_TRIANGULATION_H
#define CLOUD_ALGOS_DEPTH_IMAGE_TRIANGULATION_H

#include <pcl_cloud_algos/cloud_algos.h>
#include <pcl_cloud_algos/pcl_cloud_algos_point_types.h>

// For Extra Eigen functions
#include <Eigen3/Core>
#include <Eigen3/LU> // matrix inversion
#include <Eigen3/Geometry> // cross product

// TriangleMesh to Output Triangles
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <triangle_mesh_msgs/TriangleMesh.h>

// boost
#include <boost/thread/mutex.hpp>

namespace pcl_cloud_algos
{

class DepthImageTriangulation : public CloudAlgo
{
 public:
  //! \brief triangle points
  struct triangle 
  {  
    int a,b,c;
  };

  // Input/Output type
  typedef sensor_msgs::PointCloud2 InputType;
  typedef triangle_mesh_msgs::TriangleMesh OutputType;
  
  // Topic name to advertise
  static std::string default_output_topic ()
  {
    return std::string ("cloud_triangulated");
  }
  
  // Topic name to subscribe to
  static std::string default_input_topic ()
    {
      return std::string ("cloud_pcd");
    }

  // Node name
  static std::string default_node_name ()
    {
      return std::string ("depth_image_triangulation_node");
    }

  // Algorithm methods
  void init (ros::NodeHandle&);
  void pre  ();
  void post ();
  std::vector<std::string> requires ();
  std::vector<std::string> provides ();
  std::string process (const boost::shared_ptr<const InputType>&);
  boost::shared_ptr<const OutputType> output ();

  /**
   * \brief  get scan and point id for hokuyo scans
   * \param sensor_msg::PointCloud
   */
  void get_scan_and_point_id (pcl::PointCloud<pcl::PointXYZINormalScanLine> &cloud_in);

  /**
   * \brief  computes distance between 2 points
   * \param cloud_in
   * \param int a
   * \param int b
   */
  float dist_3d (const pcl::PointCloud<pcl::PointXYZINormalScanLine> &cloud_in, int a, int b);

  /**
   * \brief  writes triangulation result to a VTK file for visualization purposes
   * \param output[] output vtk file
   * \param triangles vector of triangles
   * \param &cloud_in
   * \param nr_tr number of triangles
   */
  void write_vtk_file (std::string output, std::vector<triangle> triangles, const pcl::PointCloud<pcl::PointXYZINormalScanLine> &cloud_in, int nr_tr);

  // Constructor-Destructor
  DepthImageTriangulation () : CloudAlgo ()
  {
    max_length = 0.05;
    max_index_ = max_line_ = 0;
    write_to_vtk_ = true;
    save_pcd_ = false;
    line_nr_in_channel_ = index_nr_in_channel_ = -1;
  }

  // DepthImageTriangulation () { }
  ros::Publisher createPublisher (ros::NodeHandle& nh)
  {
    ros::Publisher p = nh.advertise<OutputType> (default_output_topic (), 5);
    return p;
  }

private:
  //! \brief lock the function when restoring line id
  boost::mutex  cloud_lock_;
  
  // ROS stuff
  ros::NodeHandle nh_;

  //! \brief working point cloud
  pcl::PointCloud<pcl::PointXYZINormalScanLine> cloud_with_line_;
  
  //! \brief max index and max line in point cloud
  int max_index_, max_line_;
  
  //! \brief channel indices for line and index in point cloud msg
  signed int line_nr_in_channel_, index_nr_in_channel_;

  //! \brief max allowed length between triangle's line segments
  float max_length;

  //! \brief write output to vtk yes/no, save PCD file yes/no
  bool write_to_vtk_, save_pcd_;

  //! \brief resultant output triangulated map
  boost::shared_ptr<OutputType> mesh_;
};
}
#endif


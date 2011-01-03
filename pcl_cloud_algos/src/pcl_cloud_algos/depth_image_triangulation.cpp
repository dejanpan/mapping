
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

#include <map>
#include <iostream>
#include <algorithm>

#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>

#include <ros/this_node.h>
#include <pcl/io/pcd_io.h>

#include <pcl_cloud_algos/cloud_algos.h>
#include <pcl_cloud_algos/depth_image_triangulation.h>
#include <pcl_cloud_algos/pcl_cloud_algos_point_types.h>

using namespace pcl;
using namespace pcl_cloud_algos;

////////////////////////////////////////////////////////////////////////////////
void DepthImageTriangulation::get_scan_and_point_id (pcl::PointCloud<pcl::PointXYZINormalScanLine> &cloud_in)
{
  int line_id = 0;
  int point_id = 0;
  int temp_point_id = 0;

  ros::Time ts = ros::Time::now ();

  // TODO try it w/ and w/o -1 //
  for (unsigned int k = 0; k < cloud_in.points.size()-1; k++)
  {
    cloud_in.points[k].line = line_id;
    point_id = cloud_in.points[k].index;
    temp_point_id = cloud_in.points[k+1].index;
    
    // new line found
    if (temp_point_id < point_id)
    {
      line_id++;
      // find max point index in the whole point cloud
      if (point_id > max_index_)
        max_index_ = point_id;
    }
  }
  
  // nr of lines in point cloud
  max_line_ = line_id;

  if (save_pcd_)
  {
    if (verbosity_level_ > 0) ROS_INFO ("Saving PCD containing line numbers as cloud_line.pcd in ROS home.");
    pcl::PCDWriter pcd_writer;
    pcd_writer.write ("cloud_line.pcd", cloud_in, false);
  }

#ifdef DEBUG
  if (verbosity_level_ > 1) ROS_INFO ("Nr lines: %d, Max point ID: %d Completed in %f seconds", max_line_, max_index_, (ros::Time::now () - ts).toSec ());
#endif
}

////////////////////////////////////////////////////////////////////////////////
float DepthImageTriangulation::dist_3d (const pcl::PointCloud<pcl::PointXYZINormalScanLine> &cloud_in, int a, int b)
{
  return sqrt( (cloud_in.points[a].x - cloud_in.points[b].x) * (cloud_in.points[a].x - cloud_in.points[b].x)
               + (cloud_in.points[a].y - cloud_in.points[b].y) * (cloud_in.points[a].y - cloud_in.points[b].y) 
               + (cloud_in.points[a].z - cloud_in.points[b].z) * (cloud_in.points[a].z - cloud_in.points[b].z) );
}

////////////////////////////////////////////////////////////////////////////////
void DepthImageTriangulation::init (ros::NodeHandle &nh)
{
  // node handler and publisher 
  nh_ = nh; 
  ROS_INFO ("Depth Image Triangulation Node initialized");
  nh_.param("save_pcd", save_pcd_, save_pcd_);
}

////////////////////////////////////////////////////////////////////////////////
void DepthImageTriangulation::pre () 
  {

  }

////////////////////////////////////////////////////////////////////////////////
void DepthImageTriangulation::post ()
  {
 
  }

////////////////////////////////////////////////////////////////////////////////
std::vector<std::string> DepthImageTriangulation::requires () 
  {
    std::vector<std::string> requires;
    // requires 3D coordinates
    requires.push_back("x");
    requires.push_back("y");
    requires.push_back("z");
    // requires index
    requires.push_back("index");
    return requires;
  }

////////////////////////////////////////////////////////////////////////////////
std::vector<std::string> DepthImageTriangulation::provides ()
  {
    std::vector<std::string> provides;
    provides.push_back ("Triangled Mesh");
    return provides;
  }

////////////////////////////////////////////////////////////////////////////////
std::string DepthImageTriangulation::process (const boost::shared_ptr<const DepthImageTriangulation::InputType>& cloud_in)
{
#ifdef DEBUG
  if (verbosity_level_ > 1) ROS_INFO ("\n");
  if (verbosity_level_ > 1) ROS_INFO ("PointCloud msg with size %ld received", cloud_in->points.size());
#endif
  
  // we assume here having either SICK LMS400 data which has line and index coeff
  // or
  // Hokuyo UTM 30LX which only returns index coeff, line has to be computed

  ros::Time ts = ros::Time::now ();

  // convert from sensor_msgs::PointCloud2 to pcl::PointCloud<T>
  fromROSMsg (*cloud_in, cloud_with_line_);
 
  // lock down the point cloud 
  boost::mutex::scoped_lock lock (cloud_lock_);

  // <cloud_in> gets processed into <cloud_with_line_> and <max_line_> with <max_index_> are computed in get_scan_and_point_id ()
  get_scan_and_point_id (cloud_with_line_);

  // print the size of point cloud 
  if (verbosity_level_ > 0) ROS_INFO ("max line: %d, max index: %d", max_line_, max_index_);

  std::vector<triangle> tr;
  tr.resize (2*max_line_*max_index_);  
  mesh_ = boost::shared_ptr<DepthImageTriangulation::OutputType>(new DepthImageTriangulation::OutputType);
  mesh_->points.resize (0);
  mesh_->triangles.resize (0);
  mesh_->intensities.resize (0);

  // number of triangles
  int nr = 0; 
  int nr_tr;
   
  int a = 0, b, c, d, e;
  bool skipped = false;
    
  // scan line in a point cloud 
  for (int i = 0; i <=  max_line_; i++)
  {
    // laser beam in a scan line
    for (int j = 0; j <= max_index_; j++)
    {
      if (cloud_with_line_.points[a].line == i && cloud_with_line_.points[a].index == j)
      {
        ROS_INFO ("found point a = %d\n", a);

        b = -1;
        c = -1;
        d = -1;
        e = -1;
        
        // find top right corner
        if ((unsigned int)a+1 < cloud_with_line_.points.size() && cloud_with_line_.points[a+1].line == i && cloud_with_line_.points[a+1].index == j+1)
          b = a+1;

        ROS_INFO ("resolved point b = %d\n", b);

        // go to next line
        int test = a;
        while ((unsigned int)test < cloud_with_line_.points.size() &&  cloud_with_line_.points[test].line < i+1)
          test++;

        ROS_INFO ("resolved next line\n");

        // if next line exists
        if ((unsigned int)test < cloud_with_line_.points.size() && cloud_with_line_.points[test].line == i+1)
        {
          // a skipped triangle exists because of missing 'a'
          if (skipped)
          {
            // reset var
            skipped = false;

            // go to column j-1
            while ((unsigned int)test < cloud_with_line_.points.size() &&  cloud_with_line_.points[test].index < j-1)
              test++;

            // if not at the end of dataset
            if ((unsigned int)test <  cloud_with_line_.points.size())
            {
              // if column exists
              if (cloud_with_line_.points[test].line == i+1 && cloud_with_line_.points[test].index)
              {
                e = test;
                test++;
              }
            }
          }
          else
          {
            // go to column j
            while ((unsigned int)test < cloud_with_line_.points.size() && cloud_with_line_.points[test].index < j)
              test++;
          }

          // if not at the end of dataset
          if ((unsigned int)test < cloud_with_line_.points.size())
          {
          // if column exists
            if (cloud_with_line_.points[test].line == i+1 && cloud_with_line_.points[test].index == j)
            {
              c = test;
              if ((unsigned int)c+1 < cloud_with_line_.points.size() && cloud_with_line_.points[c+1].line == i+1 && cloud_with_line_.points[c+1].index == j+1)
                d = c+1;
            }
            // if next column was found
            else if (cloud_with_line_.points[test].line == i+1 && cloud_with_line_.points[test].index == j+1)
              d = test;
          }
        }

        if (c != -1)
        {
          float AC = dist_3d (cloud_with_line_, a, c);
          if (e != -1)
          {
            // a c e
            // ROS_INFO ("a c e\n");
            float AE = dist_3d (cloud_with_line_, a, e);
            float CE = dist_3d (cloud_with_line_, c, e);
            if (AC < max_length && CE < max_length && AE < max_length)
            {
              tr[nr].a = a;
              tr[nr].b = c;
              tr[nr].c = e;
              nr++;
            }
          }

          if (b != -1)
          {
            // a b c
            // ROS_INFO ("a b c\n");
            float AB = dist_3d (cloud_with_line_, a, b);
            float BC = dist_3d (cloud_with_line_, b, c);
            float AC = dist_3d (cloud_with_line_, a, c);
            if (AB < max_length && BC < max_length && AC < max_length)
            {
              tr[nr].a = a;
              tr[nr].b = b;
              tr[nr].c = c;
              nr++;
            }

            if (d != -1)
            {
              // b c d
              // ROS_INFO ("b c d\n");
              float BD = dist_3d (cloud_with_line_, b, d);
              float CD = dist_3d (cloud_with_line_, c, d);
              if (BD < max_length && BC < max_length && CD < max_length)
              {
                tr[nr].a = b;
                tr[nr].b = c;
                tr[nr].c = d;
                nr++;
              }
            }
          }
          else if (d != -1)
          {
            // a c d
            // ROS_INFO ("a c d\n");
            float AD = dist_3d (cloud_with_line_, a, d);
            float CD = dist_3d (cloud_with_line_, c, d);
            if (AD < max_length && CD < max_length && AC < max_length)
            {
              tr[nr].a = a;
              tr[nr].b = c;
              tr[nr].c = d;
              nr++;
            }
          }
        }
        else if (b != -1 && d != -1)
        {
          // a b d
          // ROS_INFO ("a b d\n");
          float AB = dist_3d (cloud_with_line_, a, b);
          float AD = dist_3d (cloud_with_line_, a, d);
          float BD = dist_3d (cloud_with_line_, b, d);
          if (AD < max_length && BD < max_length && AB < max_length)
          {
            tr[nr].a = a;
            tr[nr].b = b;
            tr[nr].c = d;
            nr++;
          }
        }

        // move to next point
        a++;

        // skipped = false;
        if ((unsigned int)a >= cloud_with_line_.points.size())
          break;
      } // END OF : top left corner found 
      else
        skipped = true;
      } // END OF : number of indexes 
    if ((unsigned int)a >= cloud_with_line_.points.size())
      break;
  } // END OF : number of lines 

  nr_tr = nr;
  tr.resize(nr);
  mesh_->header = cloud_with_line_.header;   
  mesh_->sending_node = ros::this_node::getName();   
  triangle_mesh_msgs::Triangle tr_mesh;

#ifdef DEBUG  
  if (verbosity_level_ > 1) ROS_INFO ("Triangle a: %d, b: %d, c: %d", tr[i].a, tr[i].b, tr[i].c);
#endif

  // fill up triangular mesh msg and send it on the topic
  geometry_msgs::Point32 p;
  for (unsigned int i = 0; i < cloud_with_line_.points.size(); i++)
  {
    p.x = cloud_with_line_.points[i].x;
    p.y = cloud_with_line_.points[i].y;
    p.z = cloud_with_line_.points[i].z;
    mesh_->points.push_back (p);
  }
  
  for (int i = 0; i < nr; i++)
  {
    if ((unsigned int)tr[i].a >= cloud_with_line_.points.size() || tr[i].a < 0 || isnan(tr[i].a)); 
      // printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, tr[i].a, tr[i].b, tr[i].c, cloud_with_line_.points.size());
    else if ((unsigned int)tr[i].b >= cloud_with_line_.points.size() || tr[i].b < 0 || isnan(tr[i].b));
      // printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, tr[i].a, tr[i].b, tr[i].c, cloud_with_line_.points.size());
    else if ((unsigned int)tr[i].c >= cloud_with_line_.points.size() || tr[i].c < 0 || isnan(tr[i].c)); 
      // printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, tr[i].a, tr[i].b, tr[i].c, cloud_with_line_.points.size());
    else if (tr[i].a == tr[i].b || tr[i].a == tr[i].c || tr[i].b == tr[i].c);
      // printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, tr[i].a, tr[i].b, tr[i].c, cloud_with_line_.points.size());
    else
    {
      tr_mesh.i = tr[i].a;
      tr_mesh.j = tr[i].c;
      tr_mesh.k = tr[i].b;
      mesh_->triangles.push_back(tr_mesh);
    } 
  }

  // fill in intensities (needed e.g. laser-to-camera calibration)
  std::vector<sensor_msgs::PointField> fields;
  int iIdx = getFieldIndex (cloud_with_line_, "intensity", fields);
  if (iIdx == -1)
  {
    if (verbosity_level_ > -1) ROS_WARN ("[DepthImageTriangulaton] \"intensites\" channel does not exist");
  }
  else
  {
    mesh_->intensities.resize (cloud_with_line_.points.size());
    for (unsigned int i = 0; i < cloud_with_line_.points.size(); i++)
      mesh_->intensities[i] = cloud_with_line_.points[i].intensities;
  }

  // write to vtk file for display in e.g. vtk viewer
  if (write_to_vtk_)
    write_vtk_file ("data/triangles.vtk", tr, cloud_with_line_, nr);
  if (verbosity_level_ > 0) ROS_INFO ("Triangulation with %d triangles completed in %g seconds", tr.size(), (ros::Time::now() - ts).toSec());
  return std::string("");
}

//////////////////////////////////////////////////////////////////////////////
void DepthImageTriangulation::write_vtk_file (std::string output, std::vector<triangle> triangles, const pcl::PointCloud<pcl::PointXYZINormalScanLine> &cloud_in, int nr_tr)
{
  /* writing VTK file */

  FILE *f;
  f = fopen(output.c_str(),"w");
  fprintf (f, "# vtk DataFile Version 3.0\nvtk output\nASCII\nDATASET POLYDATA\nPOINTS %d float\n", cloud_with_line_.points.size());

  for (unsigned int i = 0; i < cloud_with_line_.points.size(); i++)
  {
    fprintf (f,"%f %f %f ", cloud_with_line_.points[i].x, cloud_with_line_.points[i].y,
        cloud_with_line_.points[i].z);
    fprintf (f,"\n");
  }

  fprintf(f,"VERTICES %d %d\n", cloud_with_line_.points.size(), 2*cloud_with_line_.points.size());
  for (unsigned int i = 0; i < cloud_with_line_.points.size(); i++)
    fprintf(f,"1 %d\n", i);

  if (verbosity_level_ > 0) ROS_INFO ("vector: %d, nr: %d  ", triangles.size(), nr_tr);

  fprintf(f,"\nPOLYGONS %d %d\n", nr_tr, 4*nr_tr);
  for (int i = 0; i < nr_tr; i++)
  {
    if ((unsigned int)triangles[i].a >= cloud_with_line_.points.size() || triangles[i].a < 0 || isnan(triangles[i].a));
      //printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, triangles[i].a, triangles[i].b, triangles[i].c, cloud_with_line_.points.size());
    else if ((unsigned int)triangles[i].b >= cloud_with_line_.points.size() || triangles[i].b < 0 || isnan(triangles[i].b));
      // printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, triangles[i].a, triangles[i].b, triangles[i].c, cloud_with_line_.points.size());
    else if ((unsigned int)triangles[i].c >= cloud_with_line_.points.size() || triangles[i].c < 0 || isnan(triangles[i].c));
      // printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, triangles[i].a, triangles[i].b, triangles[i].c, cloud_with_line_.points.size());
    else if (triangles[i].a == triangles[i].b || triangles[i].a == triangles[i].c || triangles[i].b == triangles[i].c);
      // printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, triangles[i].a, triangles[i].b, triangles[i].c, cloud_with_line_.points.size());
    else
      fprintf(f,"3 %d %d %d\n",triangles[i].a, triangles[i].c, triangles[i].b);
  }
}

////////////////////////////////////////////////////////////////////////////////
boost::shared_ptr<const DepthImageTriangulation::OutputType> DepthImageTriangulation::output ()
  {
    return mesh_;
  }


#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <DepthImageTriangulation> (argc, argv);
}
#endif


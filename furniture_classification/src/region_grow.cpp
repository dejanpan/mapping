/*
 * region_grow.cpp
 *
 *  Created on: May 30, 2012
 *      Author: vsu
 */

#include <pcl17/console/parse.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/search/kdtree.h>
#include <pcl17/surface/mls.h>
#include <pcl17/segmentation/region_growing_rgb.h>
#include <pcl17/features/normal_3d.h>
#include <pcl17/visualization/pcl_visualizer.h>
#include <pcl17/filters/voxel_grid.h>

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    PCL_INFO ("Usage %s -input_file cloud.pcd -distance_thresh 0.01 -angle_thresh 30 \n", argv[0]);

    return -1;
  }

  std::string filename;
  float distance_thresh = 0.01;
  float angle_thresh = 0.01;

  pcl17::console::parse_argument(argc, argv, "-input_file", filename);
  pcl17::console::parse_argument(argc, argv, "-distance_thresh", distance_thresh);
  pcl17::console::parse_argument(argc, argv, "-angle_thresh", distance_thresh);

  pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr cloud_orig(new pcl17::PointCloud<pcl17::PointXYZRGBA>);
  pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr cloud(new pcl17::PointCloud<pcl17::PointXYZRGBA>);
  pcl17::PointCloud<pcl17::Normal>::Ptr cloud_normals(new pcl17::PointCloud<pcl17::Normal>());
  pcl17::io::loadPCDFile(filename, *cloud);

  std::cerr << "Loaded file" << std::endl;

  //pcl17::VoxelGrid<pcl17::PointXYZRGBA> grid;
  //grid.setInputCloud(cloud_orig);
  //grid.setLeafSize(0.01f, 0.01f, 0.01f);
  //grid.filter(*cloud);

  // Create a KD-Tree
  pcl17::search::KdTree<pcl17::PointXYZRGBA>::Ptr tree(new pcl17::search::KdTree<pcl17::PointXYZRGBA>);

  std::cerr << "Created tree" << std::endl;

  pcl17::NormalEstimation<pcl17::PointXYZRGBA, pcl17::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.01);
  ne.compute(*cloud_normals);

  std::cerr << "Computed normals" << std::endl;

  pcl17::RegionGrowingRGB<pcl17::PointXYZRGBA> region_growing;
  region_growing.setCloud(cloud);
  region_growing.setNormals(cloud_normals);
  region_growing.setNeighbourSearchMethod(tree);

  region_growing.setResidualTest(true);
  region_growing.setResidualThreshold(distance_thresh);

  region_growing.setCurvatureTest(true);
  region_growing.setCurvatureThreshold(0.04);


  region_growing.setSmoothMode(false);
  region_growing.setSmoothnessThreshold(angle_thresh * M_PI/180);

  region_growing.setPointColorThreshold(100.0);
  region_growing.setRegionColorThreshold(200.0);

  region_growing.segmentPoints();

  pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr segments;
  segments = region_growing.getColoredCloud();

  pcl17::io::savePCDFile("result.pcd", *segments);

  pcl17::visualization::PCLVisualizer viz;
  viz.initCameraParameters();

  pcl17::visualization::PointCloudColorHandlerRGBField<pcl17::PointXYZRGB> rgb(segments);
  viz.addPointCloud<pcl17::PointXYZRGB> (segments, rgb);
  viz.spin();

  return 0;
}

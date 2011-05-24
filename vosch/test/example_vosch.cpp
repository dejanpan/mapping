#include "vosch/vosch_tools.h"

//-------
//* main
int main( int argc, char** argv ){
  if( argc != 2 ){
    ROS_ERROR ("Need one parameter! Syntax is: %s {input_pointcloud_filename.pcd}\n", argv[0]);
    return(-1);
  }
  //* voxel size (downsample_leaf)
  const double voxel_size = 0.01;

  //* read
  pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
  readPoints( argv[1], input_cloud );
  double t1 = my_clock();

  //* compute normals
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
  computeNormal( input_cloud, cloud );
  ROS_INFO("Normal compute done in %f seconds.", my_clock()-t1);

  //* voxelize
  pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid;
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled;
  getVoxelGrid( grid, cloud, cloud_downsampled, voxel_size );

  //* extract - VOSCH -
  std::vector<float> vosch;
  extractVOSCH( grid, cloud, cloud_downsampled, vosch, 127, 127, 127, voxel_size );
  //std::vector< std::vector<float> > vosch;
  //extractVOSCH( grid, cloud, cloud_downsampled, vosch, 127, 127, 127, voxel_size, 10, 5, 5, 5 );

  ROS_INFO("VOSCH %ld", input_cloud.points.size());

  //* write
  writeFeature( "vosch.pcd", vosch );

  return(0);
}

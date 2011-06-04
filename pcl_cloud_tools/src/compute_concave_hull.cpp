#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/console/parse.h>

using namespace pcl::console;
typedef pcl::PointXYZ PointT;

int main( int argc, char** argv )
{
  // Parameter parsing
  if( argc != 3 )
  {
    std::cerr << "Syntax is: " << argv[0] << " {input_pointcloud_filename.pcd} {output_pointcloud_filename.pcd}" << std::endl;
    return(-1);
  }

  // Read input cloud
  pcl::PointCloud<PointT>::Ptr input_cloud (new pcl::PointCloud<PointT>);
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  reader.read ( argv[1], *input_cloud);
  std::cerr << "Loaded cloud with points: " << input_cloud->points.size() << std::endl;


  pcl::PointCloud<PointT>::Ptr filtered_cloud (new pcl::PointCloud<PointT>);
  pcl::PassThrough<PointT> filter;
  filter.setInputCloud (input_cloud);
  filter.setFilterFieldName ("z");
  filter.setFilterLimits (-0.5, 0.5);
  // filter.setKeepOrganized (true);
  // filter.setUserFilterValue (1000.0);
  filter.filter (*filtered_cloud);
  
  pcl::PointCloud<PointT> alpha_shape;
  pcl::PointCloud<PointT>::Ptr voronoi_centers (new pcl::PointCloud<PointT>);
  std::vector<pcl::Vertices> polygons_alpha;

  pcl::ConcaveHull<PointT> concave_hull;
  concave_hull.setInputCloud (filtered_cloud);
  concave_hull.setAlpha (1.0);
  concave_hull.setVoronoiCenters (voronoi_centers);
  concave_hull.reconstruct (alpha_shape, polygons_alpha);

  //write back filtered cloud
  std::cerr << "Saving cloud to: " << std::string(argv[2]) << " with points: " << alpha_shape.points.size() << std::endl;
  writer.write(std::string(argv[2]), alpha_shape, false);
  return(0);
}

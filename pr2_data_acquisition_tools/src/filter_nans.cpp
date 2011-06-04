#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/console/parse.h>

using namespace pcl::console;
typedef pcl::PointXYZRGB PointT;

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

  // Create the pass through filter
  pcl::PointCloud<PointT>::Ptr filtered_cloud (new pcl::PointCloud<PointT>);
  pcl::PassThrough<PointT> filter;
  filter.setInputCloud (input_cloud);
  filter.setFilterFieldName ("x");
  filter.setFilterLimits (-100.0, 100.0);
  // filter.setKeepOrganized (true);
  // filter.setUserFilterValue (1000.0);
  filter.filter (*filtered_cloud);
  
  //write back filtered cloud
  std::cerr << "Saving cloud to: " << std::string(argv[2]) << " with points: " << filtered_cloud->points.size() << std::endl;
  writer.write(std::string(argv[2]), *filtered_cloud, false);
  return(0);
}

//#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter_indices.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>

#include <vector>
#include <limits>
typedef typename pcl::PointXYZ PointIn;
typedef typename pcl::PointXYZI PointOut;
typedef pcl::PointCloud<PointIn> CloudIn;
typedef typename CloudIn::Ptr CloudInPtr;
typedef pcl::PointCloud<PointOut> CloudOut;
typedef typename CloudOut::Ptr CloudOutPtr;

  struct CloudLabelPair {
    std::string name;
    int label;
  } ;

int 
main (int argc, char** argv)
{
  //When do the scene-label pairs begin in argv table
  if (argc == 1)
    {
      pcl::console::print_info ("Syntax is: %s <-in scene_in.pcd> <-r radius> <-out scene_out.pcd> <-n subscenes in argv table>  <subscene1.pcd label1 ...> \n", argv[0]);
      exit(0);
    }

  double radius;
  pcl::console::parse_argument (argc, argv, "-r", radius);

  std::string in_cloud;
  pcl::console::parse_argument (argc, argv, "-in", in_cloud);

  std::string out_cloud;
  pcl::console::parse_argument (argc, argv, "-out", out_cloud);

  int subscene_argc;
  pcl::console::parse_argument (argc, argv, "-n", subscene_argc);

  std::vector<CloudLabelPair> all_clouds_and_labels;
  CloudLabelPair pair;
  for (int i = subscene_argc; i< argc; i=i+2)
    {
      pair.name = std::string(argv[i]);
      pair.label = atoi(argv[i+1]);
      all_clouds_and_labels.push_back(pair);
    }

  double start, stop;
  // Read in the full cloud
  pcl::PCDReader reader;
  CloudInPtr cloud (new CloudIn), query_cloud (new CloudIn);
  CloudOutPtr cloud_out (new CloudOut);
  reader.read (in_cloud, *cloud);
  pcl::copyPointCloud(*cloud, *cloud_out);
  for (uint i = 0; i < cloud_out->points.size(); i++)
    {
      cloud_out->points[i].intensity = -1;
    }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointIn>::Ptr tree (new  pcl::search::KdTree<PointIn>);
  std::vector< int > cloud_non_nan;
  std::cerr << "removing NaN points: " << cloud->points.size() << std::endl;
  pcl::removeNaNFromPointCloud	(*cloud, cloud_non_nan);
  std::cerr << "removed NaN points: " << cloud_non_nan.size() << std::endl;
  pcl::IndicesPtr cloud_non_nan_ind (new std::vector<int>);
  *cloud_non_nan_ind = cloud_non_nan;
  tree->setInputCloud (cloud, cloud_non_nan_ind);
  std::vector<int> kd_indices;
  std::vector<float> kd_distances;
  PointIn query;
  int knn = 0;
  double dist;
  int closest;
  //reading in and performing NN search for part clouds
  for (uint k = 0; k < all_clouds_and_labels.size(); k++)
    {
      reader.read (all_clouds_and_labels[k].name, *query_cloud);
      int label = all_clouds_and_labels[k].label;
      start = pcl::getTime ();
      std::cerr << "starting radiusSearch for label: " << label << " , query_cloud size: " << query_cloud->points.size() << std::endl;
      for (uint i = 0; i < query_cloud->points.size(); i++)
        {
          query = query_cloud->points[i];
          if( !pcl_isfinite( query.x ) ||
              !pcl_isfinite( query.y ) ||
              !pcl_isfinite( query.z ) )
            continue; 
          tree->radiusSearch (query, radius, kd_indices, kd_distances, knn);
          dist = std::numeric_limits<double>::max();
          closest = -1;
          if (kd_indices.size() != 0)
            {
              for (uint j = 0; j < kd_indices.size(); j++)
                {
                  if (kd_distances[j] < dist)
                    {
                      dist = kd_distances[j];
                      closest = kd_indices[j];
                    }
                }
            }
          if (closest != -1)
            {
              cloud_out->points[closest].intensity = label;
            }
        }
      stop = pcl::getTime ();
      std::cerr << "search query: " << stop - start << std::endl;
    }

  pcl::PCDWriter writer;
  writer.write (out_cloud.c_str(), *cloud_out, false); //*

  return (0);
}

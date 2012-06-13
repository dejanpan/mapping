/*
 * scan.cpp
 *
 *  Created on: Apr 17, 2012
 *      Author: vsu
 */

#include <pcl/console/parse.h>
#include <training.h>

int main(int argc, char** argv)
{

	// TODO write about dir structure
  if (argc < 5)
  {
    PCL_INFO ("Usage %s -input_dir /dir/with/pointclouds -output_file /where/to/put/database [options]\n", argv[0]);
    PCL_INFO (" * where options are:\n"
        "         -min_points_in_segment <X>  : set minimal number of points in segment to X. Default : 300\n"
        "         -num_clusters <X>           : set Number of clusters. Default : 5\n"
        "");
    return -1;
  }

  int min_points_in_segment = 300;
  int num_clusters = 5;
  std::string input_dir;
  std::string output_file;

  pcl::console::parse_argument(argc, argv, "-input_dir", input_dir);
  pcl::console::parse_argument(argc, argv, "-output_file", output_file);
  pcl::console::parse_argument(argc, argv, "-num_clusters", num_clusters);
  pcl::console::parse_argument(argc, argv, "-min_points_in_segment", min_points_in_segment);

  std::vector<featureType> features;
  pcl::PointCloud<pcl::PointXYZ> centroids;
  std::vector<std::string> classes;

  std::vector<featureType> cluster_centers;
  std::vector<int> cluster_labels;

  std::map<std::string, std::vector<std::string> > class_to_full_pointcloud;

  std::vector<std::string> files_to_process;
  get_files_to_process(input_dir, files_to_process, class_to_full_pointcloud);

  pcl::PointCloud<pcl::PointNormal> tmp;

  for (size_t i = 0; i < files_to_process.size(); i++)
  {
    std::vector<std::vector<int> > tmp_segment_indices;
    append_segments_from_file(files_to_process[i], features, centroids, classes, min_points_in_segment, tmp, tmp_segment_indices);
  }


  // Transform to model centroinds in local coordinate frame of the segment
  centroids.getMatrixXfMap() *= -1;

  featureType min, max;
  normalizeFeatures(features, min, max);

  cluster_features(features, num_clusters, cluster_centers, cluster_labels);

  databaseType database;

  create_codebook(features, centroids, classes, cluster_centers, cluster_labels, database);

  save_codebook(output_file, database, min, max, class_to_full_pointcloud);

  return 0;
}


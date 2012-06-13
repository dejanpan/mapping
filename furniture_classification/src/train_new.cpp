/*
 * region_grow.cpp
 *
 *  Created on: May 30, 2012
 *      Author: vsu
 */

#include <training.h>

#include <pcl/point_types.h>
#include <pcl/features/sgfall.h>
#include <pcl/console/parse.h>
#include <pcl/classification/PHVObjectClassifier.h>

int main(int argc, char **argv)
{

  if (argc < 5)
  {
    PCL_INFO ("Usage %s -input_dir /dir/with/pointclouds -output_dir /where/to/put/database [options]\n", argv[0]);
    PCL_INFO (" * where options are:\n"
        "         -min_points_in_segment <X>  : set minimal number of points in segment to X. Default : 300\n"
        "         -num_clusters <X>           : set Number of clusters. Default : 5\n"
        "");
    return -1;
  }

  std::string input_dir;
  std::string output_dir;

  pcl::console::parse_argument(argc, argv, "-input_dir", input_dir);
  pcl::console::parse_argument(argc, argv, "-output_dir", output_dir);

  std::map<std::string, std::vector<std::string> > class_to_full_pointcloud;
  std::vector<std::string> files_to_process;
  get_files_to_process(input_dir, files_to_process, class_to_full_pointcloud);

  pcl::PHVObjectClassifier<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<25> > oc;
  oc.setDebugFolder(output_dir + "debug/");
  oc.setDebug(true);

  pcl::SGFALLEstimation<pcl::PointNormal, pcl::Histogram<25> >::Ptr feature_estimator(new pcl::SGFALLEstimation<pcl::PointNormal, pcl::Histogram<25> >);
  oc.setFeatureEstimator(feature_estimator);




  BOOST_FOREACH(std::string &file, files_to_process)
{  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(file, *cloud);

  std::vector<std::string> st;
  boost::split(st, file, boost::is_any_of("/"), boost::token_compress_on);
  std::string class_name = st.at(st.size() - 3);

  oc.addObjectPartialView(cloud, class_name);
}

  oc.computeClassifier();

  oc.saveToFile(output_dir + "database.yaml");

return 0;
}

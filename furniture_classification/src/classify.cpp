/*
 * classify_new.cpp
 *
 *  Created on: Jun 11, 2012
 *      Author: vsu
 */

/*
 * classify.cpp
 *
 *  Created on: Apr 25, 2012
 *      Author: vsu
 */

#include <pcl/console/parse.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree.h>
#include <pcl/classification/PHVObjectClassifier.h>
#include <pcl/features/sgfall.h>
#include <sac_3dof.h>
#include <set>
#include <pcl/io/pcd_io.h>
#include <ransac_simple.h>

//typedef pcl::Histogram<pcl::SGFALL_SIZE> FeatureType;
//typedef pcl::SGFALLEstimation<pcl::PointNormal, pcl::Histogram<pcl::SGFALL_SIZE> > FeatureEstimatorType;

typedef pcl::ESFSignature640 FeatureType;
typedef pcl::ESFEstimation<pcl::PointNormal, pcl::ESFSignature640 > FeatureEstimatorType;

int main(int argc, char** argv)
{

  if (argc < 5)
  {
    PCL_INFO ("Usage %s -scene_file_name /dir/with/pointclouds -database_dir /where/to/put/database [options]\n", argv[0]);
    return -1;
  }

  std::string database_dir;
  std::string scene_file_name;

  pcl::console::parse_argument(argc, argv, "-database_dir", database_dir);
  pcl::console::parse_argument(argc, argv, "-scene_file_name", scene_file_name);

  std::vector<std::string> st;
  boost::split(st, scene_file_name, boost::is_any_of("/"), boost::token_compress_on);
  std::string scene_name = st.at(st.size() - 1);
  scene_name = scene_name.substr(0, scene_name.size() - 4);

  std::string debug_folder = scene_name + "_debug/";
  std::string output_dir = scene_name + "_result/";

  pcl::PHVObjectClassifier<pcl::PointXYZ, pcl::PointNormal, FeatureType > oc;

  pcl::SGFALLEstimation<pcl::PointNormal, FeatureType >::Ptr feature_estimator(new FeatureEstimatorType);
  oc.setFeatureEstimator(feature_estimator);

  oc.setDatabaseDir(database_dir);
  oc.loadFromFile();

  oc.setDebugFolder(debug_folder);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile(scene_file_name, *cloud);
  oc.setScene(cloud, 2.4);

  oc.classify();

  map<string, vector<pcl::PointCloud<pcl::PointNormal>::Ptr> > objects = oc.getFoundObjects();

  typedef typename map<string, vector<pcl::PointCloud<pcl::PointNormal>::Ptr> >::value_type vt;

  boost::filesystem::path out_path(output_dir);

  if (boost::filesystem::exists(out_path))
  {
    boost::filesystem::remove_all(out_path);
  }

  boost::filesystem::create_directories(out_path);

  BOOST_FOREACH(vt &v, objects)
{  for(size_t i=0; i<v.second.size(); i++)
  {
    std::stringstream ss;
    ss << output_dir << v.first << i << ".pcd";
    std::cerr << "Writing to file " << ss.str() << std::endl;
    pcl::io::savePCDFileASCII(ss.str(), *v.second[i]);
  }
}

return 0;
}


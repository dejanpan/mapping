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

int main(int argc, char** argv)
{

  std::string database_dir = "data/database/";
  std::string scene_file_name = "data/test/scenes/chairAndDesk1.pcd";
  std::string debug_folder = "data/debug_classification/";
  std::string output_dir = "data/result/";

  pcl::PHVObjectClassifier<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<25> > oc;

  pcl::SGFALLEstimation<pcl::PointNormal, pcl::Histogram<25> >::Ptr feature_estimator(new pcl::SGFALLEstimation<
      pcl::PointNormal, pcl::Histogram<25> >);
  oc.setFeatureEstimator(feature_estimator);

  oc.setDatabaseDir(database_dir);
  oc.loadFromFile();

  oc.setDebugFolder(debug_folder);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile(scene_file_name, *cloud);
  oc.setScene(cloud);

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


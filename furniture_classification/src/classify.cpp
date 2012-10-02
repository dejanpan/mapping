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

#include <pcl17/console/print.h>
#include <pcl17/console/parse.h>
#include <pcl17/sample_consensus/ransac.h>
#include <pcl17/filters/passthrough.h>
#include <pcl17/octree/octree.h>
#include <pcl17/classification/PHVObjectClassifier.h>
#include <pcl17/features/sgfall.h>
#include <sac_3dof.h>
#include <set>
#include <pcl17/io/pcd_io.h>
#include <ransac_simple.h>
#include <pcl17/features/vfh.h>

template<class FeatureType, class FeatureEstimatorType>
  void classify(string database_dir, string scene_file_name)
  {

    std::vector<std::string> st;
    boost::split(st, scene_file_name, boost::is_any_of("/"), boost::token_compress_on);
    std::string scene_name = st.at(st.size() - 1);
    scene_name = scene_name.substr(0, scene_name.size() - 4);

    std::string debug_folder = scene_name + "_debug/";
    std::string output_dir = scene_name + "_result/";

    pcl17::PHVObjectClassifier<pcl17::PointXYZ, pcl17::PointNormal, FeatureType> oc;

    typename pcl17::Feature<pcl17::PointNormal, FeatureType>::Ptr feature_estimator(new FeatureEstimatorType);
    oc.setFeatureEstimator(feature_estimator);

    oc.setDebug(true);
    oc.setDatabaseDir(database_dir);
    oc.loadFromFile();

    oc.setDebugFolder(debug_folder);

    pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud(new pcl17::PointCloud<pcl17::PointXYZ>);

    pcl17::io::loadPCDFile(scene_file_name, *cloud);
    oc.setScene(cloud, 2.4);

    oc.classify();

    map<string, vector<pcl17::PointCloud<pcl17::PointNormal>::Ptr> > objects = oc.getFoundObjects();

    typedef typename map<string, vector<pcl17::PointCloud<pcl17::PointNormal>::Ptr> >::value_type vt;

    boost::filesystem::path out_path(output_dir);

    if (boost::filesystem::exists(out_path))
    {
      boost::filesystem::remove_all(out_path);
    }

    boost::filesystem::create_directories(out_path);

    BOOST_FOREACH(vt &v, objects)
{    for(size_t i=0; i<v.second.size(); i++)
    {
      std::stringstream ss;
      ss << output_dir << v.first << i << ".pcd";
      std::cerr << "Writing to file " << ss.str() << std::endl;
      pcl17::io::savePCDFileASCII(ss.str(), *v.second[i]);
    }
  }

}

int main(int argc, char** argv)
{

  if (argc < 5)
  {
    PCL17_INFO ("Usage %s -scene_file_name /dir/with/pointclouds -database_dir /where/to/put/database [options]\n", argv[0]);
    return -1;
  }

  std::string database_dir;
  std::string scene_file_name;
  std::string features = "sgf";

  pcl17::console::parse_argument(argc, argv, "-database_dir", database_dir);
  pcl17::console::parse_argument(argc, argv, "-scene_file_name", scene_file_name);

  pcl17::console::parse_argument(argc, argv, "-features", features);

  if (features == "sgf")
  {
    classify<pcl17::Histogram<pcl17::SGFALL_SIZE>,
        pcl17::SGFALLEstimation<pcl17::PointNormal, pcl17::Histogram<pcl17::SGFALL_SIZE> > > (database_dir, scene_file_name

    );
  }
  else if (features == "esf")
  {
    classify<pcl17::ESFSignature640, pcl17::ESFEstimation<pcl17::PointNormal, pcl17::ESFSignature640> > (database_dir,
                                                                                                 scene_file_name);
  }
  else if (features == "vfh")
  {
    classify<pcl17::VFHSignature308, pcl17::VFHEstimation<pcl17::PointNormal, pcl17::PointNormal, pcl17::VFHSignature308> > (
                                                                                                                   database_dir,
                                                                                                                   scene_file_name);
  }
  else
  {
    std::cerr << "Unknown feature type " << features << " specified" << std::endl;
  }

  return 0;
}


/*
 * region_grow.cpp
 *
 *  Created on: May 30, 2012
 *      Author: vsu
 */

#include <pcl/point_types.h>
#include <pcl/features/sgfall.h>
#include <pcl/console/parse.h>
#include <pcl/classification/PHVObjectClassifier.h>
#include <pcl/features/vfh.h>

template<class FeatureType, class FeatureEstimatorType>
  void train(string input_dir, string output_dir, int num_clusters, const std::string & extermal_classifier_file)
  {
    pcl::PHVObjectClassifier<pcl::PointXYZ, pcl::PointNormal, FeatureType> oc;
    oc.setDebugFolder("debug/");
    //oc.setDebug(true);

    typename pcl::Feature<pcl::PointNormal, FeatureType>::Ptr feature_estimator(new FeatureEstimatorType);
    oc.setFeatureEstimator(feature_estimator);

    //PCL_INFO("Processing following files:\n");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    boost::filesystem::directory_iterator dir_iter(input_dir), end;

    BOOST_FOREACH(const boost::filesystem::path& class_dir, std::make_pair(dir_iter, end))
{    boost::filesystem::directory_iterator class_dir_iter(class_dir), end;
    BOOST_FOREACH(const boost::filesystem::path& model_dir, std::make_pair(class_dir_iter, end))
    {

      boost::filesystem::directory_iterator model_dir_iter(model_dir), end;
      BOOST_FOREACH(const boost::filesystem::path& v, std::make_pair(model_dir_iter, end))
      {
        if(v.filename() == "full.pcd")
        {

          pcl::io::loadPCDFile(v.c_str(), *cloud);
          oc.addObjectFullModel(cloud, class_dir.filename().c_str());
        }

        if(v.extension() == ".pcd")
        {
          pcl::io::loadPCDFile(v.c_str(), *cloud);
          oc.addObjectPartialView(cloud, class_dir.filename().c_str());
        }

      }

    }

  }
  oc.setNumberOfClusters(num_clusters);
  if (extermal_classifier_file != "")
  {
    oc.computeExternalClassifier(extermal_classifier_file);
  }
  else
  {
    oc.computeClassifier();
  }
  oc.setDatabaseDir(output_dir);
  oc.saveToFile();
}

int main(int argc, char **argv)
{

  if (argc < 5)
  {
    PCL_INFO ("Usage %s -input_dir /dir/with/pointclouds -output_dir /where/to/put/database [options]\n", argv[0]);
    PCL_INFO (" * where options are:\n"
        "         -min_points_in_segment <X>  : set minimal number of points in segment to X. Default : 300\n"
        "         -num_clusters <X>           : set Number of clusters. Default : 5\n"
        "         -features <X>               : which features to use (sgf, vfh, esf). Default : sgf\n"
        "");
    return -1;
  }

  std::string input_dir;
  std::string output_dir;
  int num_clusters = 40;
  std::string features = "sgf";
  std::string extermal_classifier_file = "";

  pcl::console::parse_argument(argc, argv, "-input_dir", input_dir);
  pcl::console::parse_argument(argc, argv, "-output_dir", output_dir);
  pcl::console::parse_argument(argc, argv, "-num_clusters", num_clusters);
  pcl::console::parse_argument(argc, argv, "-features", features);
  pcl::console::parse_argument(argc, argv, "-extermal_classifier_file", extermal_classifier_file);

  if (features == "sgf")
  {
    train<pcl::Histogram<pcl::SGFALL_SIZE>, pcl::SGFALLEstimation<pcl::PointNormal, pcl::Histogram<pcl::SGFALL_SIZE> > > (
                                                                                                                          input_dir,
                                                                                                                          output_dir,
                                                                                                                          num_clusters,
                                                                                                                          extermal_classifier_file);
  }
  else if (features == "esf")
  {
    train<pcl::ESFSignature640, pcl::ESFEstimation<pcl::PointNormal, pcl::ESFSignature640> > (input_dir, output_dir,
                                                                                              num_clusters,
                                                                                              extermal_classifier_file);
  }
  else if (features == "vfh")
  {
    train<pcl::VFHSignature308, pcl::VFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::VFHSignature308> > (
                                                                                                                input_dir,
                                                                                                                output_dir,
                                                                                                                num_clusters,
                                                                                                                extermal_classifier_file);
  }
  else
  {
    std::cerr << "Unknown feature type " << features << " specified" << std::endl;
  }

  return 0;
}

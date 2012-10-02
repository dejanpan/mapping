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

#include <pcl17/console/parse.h>
#include <pcl17/console/print.h>
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
  void eval_clustering(string database_dir, string scans_dir,const  std::string & extermal_classifier_file)
  {
    std::string debug_folder = "eval_debug/";

    for (float lmt = 0.0f; lmt <= 1.0f; lmt += 0.1f)
    {

      pcl17::PHVObjectClassifier<pcl17::PointXYZ, pcl17::PointNormal, FeatureType> oc;

      typename pcl17::Feature<pcl17::PointNormal, FeatureType>::Ptr feature_estimator(new FeatureEstimatorType);
      oc.setFeatureEstimator(feature_estimator);

      oc.setDatabaseDir(database_dir);
      oc.loadFromFile();

      oc.setDebugFolder(debug_folder);
      oc.setDebug(false);

      pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud(new pcl17::PointCloud<pcl17::PointXYZ>);

      //int iter = 0;

      oc.setLocalMaximaThreshold(lmt);

      double tp = 0, fn = 0, fp = 0;

      boost::filesystem::directory_iterator dir_iter(scans_dir), end;

      BOOST_FOREACH(const boost::filesystem::path& class_dir, std::make_pair(dir_iter, end))
{      boost::filesystem::directory_iterator class_dir_iter(class_dir), end;
      BOOST_FOREACH(const boost::filesystem::path& model_dir, std::make_pair(class_dir_iter, end))
      {

        boost::filesystem::directory_iterator model_dir_iter(model_dir), end;
        BOOST_FOREACH(const boost::filesystem::path& v, std::make_pair(model_dir_iter, end))
        {
          if((v.extension() == ".pcd") && (v.filename() != "full.pcd"))
          {
            //std::cerr << "Processing scan number " << iter << std::endl;
            //iter++;
            pcl17::io::loadPCDFile(v.c_str(), *cloud);
            oc.setScene(cloud, 2.4);

            if(extermal_classifier_file == "")
            {
              oc.eval_clustering(class_dir.filename().string(), 0.02f, tp, fn, fp);
            }
            else
            {
              oc.eval_clustering_external(class_dir.filename().string(), 0.02f, tp, fn, fp, extermal_classifier_file);
            }

            //oc.addObjectPartialView(cloud, class_dir.filename().c_str());
          }

        }

      }

    }

    //    std::cout << "True Positive: " << tp << std::endl;
    //    std::cout << "False Positive: " << fp << std::endl;
    //    std::cout << "False Negative: " << fn << std::endl;
    //    std::cout << "Precision: " << tp/(tp+fp) << std::endl;
    //    std::cout << "Recall: " << tp/(tp+fn) << std::endl;

    std::cout << lmt << " " << tp << " " << fp << " " << fn << " " << tp/(tp+fp) << " " << tp/(tp+fn) << std::endl;

  }
}

int main(int argc, char** argv)
{

  if (argc < 5)
  {
    PCL17_INFO ("Usage %s -scans_dir /dir/with/scans -database_dir /where/to/put/database [options]\n", argv[0]);
    return -1;
  }

  std::string database_dir;
  std::string scans_dir;
  std::string features = "sgf";
  std::string extermal_classifier_file = "";

  pcl17::console::parse_argument(argc, argv, "-database_dir", database_dir);
  pcl17::console::parse_argument(argc, argv, "-scans_dir", scans_dir);
  pcl17::console::parse_argument(argc, argv, "-features", features);
  pcl17::console::parse_argument(argc, argv, "-extermal_classifier_file", extermal_classifier_file);

  if (features == "sgf")
  {
    eval_clustering<pcl17::Histogram<pcl17::SGFALL_SIZE>, pcl17::SGFALLEstimation<pcl17::PointNormal, pcl17::Histogram<
        pcl17::SGFALL_SIZE> > > (database_dir, scans_dir, extermal_classifier_file

    );
  }
  else if (features == "esf")
  {
    eval_clustering<pcl17::ESFSignature640, pcl17::ESFEstimation<pcl17::PointNormal, pcl17::ESFSignature640> > (database_dir,
                                                                                                        scans_dir,
                                                                                                        extermal_classifier_file);
  }
  else if (features == "vfh")
  {
    eval_clustering<pcl17::VFHSignature308, pcl17::VFHEstimation<pcl17::PointNormal, pcl17::PointNormal, pcl17::VFHSignature308> > (
                                                                                                                          database_dir,
                                                                                                                          scans_dir,
                                                                                                                          extermal_classifier_file);
  }
  else
  {
    std::cerr << "Unknown feature type " << features << " specified" << std::endl;
  }

  return 0;
}


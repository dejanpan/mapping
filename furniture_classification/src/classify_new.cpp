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
#include <sac_3dof.h>
#include <training.h>
#include <set>

int main(int argc, char** argv)
{

  std::string database_dir = "data/database/";
  std::string scene_file_name = "data/test/scenes/chairAndDesk1.pcd";


  pcl::PHVObjectClassifier<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<25> > oc;
  oc.setDebugFolder("debug/");
  oc.setDebug(true);

  pcl::SGFALLEstimation<pcl::PointNormal, pcl::Histogram<25> >::Ptr feature_estimator(new pcl::SGFALLEstimation<
      pcl::PointNormal, pcl::Histogram<25> >);
  oc.setFeatureEstimator(feature_estimator);

  oc.setDatabaseDir(database_dir);
  oc.loadFromFile();



  return 0;
}


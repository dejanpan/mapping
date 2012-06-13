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

  std::string database_file_name = "data/database/database.yaml";


  pcl::PHVObjectClassifier<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<25> > oc;
  oc.setDebugFolder("debug/");
  oc.setDebug(true);

  pcl::SGFALLEstimation<pcl::PointNormal, pcl::Histogram<25> >::Ptr feature_estimator(new pcl::SGFALLEstimation<
      pcl::PointNormal, pcl::Histogram<25> >);
  oc.setFeatureEstimator(feature_estimator);

  oc.loadFromFile(database_file_name);
  oc.saveToFile("data/database/database1.yaml");

  return 0;
}


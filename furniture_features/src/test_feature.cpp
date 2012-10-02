/*
 * test_feature.cpp
 *
 *  Created on: Jul 11, 2012
 *      Author: vsu
 */

#include <pcl17/io/pcd_io.h>
#include <pcl17/point_cloud.h>
#include <pcl17/point_types.h>
#include <pcl17/features/esf.h>

int main(){

  pcl17::PointCloud<pcl17::PointNormal> pn;
  pcl17::io::loadPCDFile("../furniture_classification/data/debug/Cluster0/Segment10.pcd",pn);

  pcl17::ESFEstimation<pcl17::PointNormal, pcl17::ESFSignature640> e;
  e.setInputCloud(pn.makeShared());

  pcl17::PointCloud<pcl17::ESFSignature640> pc;
  e.compute(pc);

  std::cerr << pc.size() << std::endl;

  for(int i=0; i<640; i++){
    std::cerr << pc[0].histogram[i] << " ";
  }


  return 0;
}

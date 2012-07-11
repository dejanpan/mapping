/*
 * test_feature.cpp
 *
 *  Created on: Jul 11, 2012
 *      Author: vsu
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/esf.h>

int main(){

  pcl::PointCloud<pcl::PointNormal> pn;
  pcl::io::loadPCDFile("../furniture_classification/data/debug/Cluster0/Segment10.pcd",pn);

  pcl::ESFEstimation<pcl::PointNormal, pcl::ESFSignature640> e;
  e.setInputCloud(pn.makeShared());

  pcl::PointCloud<pcl::ESFSignature640> pc;
  e.compute(pc);

  std::cerr << pc.size() << std::endl;

  for(int i=0; i<640; i++){
    std::cerr << pc[0].histogram[i] << " ";
  }


  return 0;
}

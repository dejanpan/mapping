/*
 * sac_test.cpp
 *
 *  Created on: May 30, 2012
 *      Author: vsu
 */

#include <pcl17/io/pcd_io.h>
#include <pcl17/sample_consensus/ransac.h>
#include <pcl17/common/transforms.h>
#include <pcl17/search/kdtree.h>
#include <pcl17/surface/mls.h>
#include <sac_3dof.h>
#include <ransac_simple.h>
#include <pcl17/visualization/pcl_visualizer.h>

int main(int argc, char **argv)
{
  std::string filename = "data/scans/Chairs/Aluminium_Group_EA_119_0000F152-centered/full.pcd";
  std::string scene_name = "data/test/scenes/chairAndDesk1.pcd";
  //float move_x = 10.0;
  //float move_y = 20.0;
  //float rotate_z = M_PI / 3;

  pcl17::PointCloud<pcl17::PointNormal>::Ptr model(new pcl17::PointCloud<pcl17::PointNormal>), scene(new pcl17::PointCloud<
      pcl17::PointNormal>), model_transformed(new pcl17::PointCloud<pcl17::PointNormal>);

  pcl17::io::loadPCDFile(filename, *model);
  pcl17::io::loadPCDFile(scene_name, *scene);

  //Eigen::Affine3f true_transform;
  //true_transform.setIdentity();
  //true_transform.translate(Eigen::Vector3f(move_x, move_y, 0));
  //true_transform.rotate(Eigen::AngleAxisf(rotate_z, Eigen::Vector3f(0, 0, 1)));

  //std::cerr << "Transforming points with model (" << move_x << "," << move_y << "," << rotate_z << ")" << std::endl;
  //pcl17::transformPointCloudWithNormals(*model, *model_transformed, true_transform);

  //  pcl17::visualization::PCLVisualizer viz;
  //  viz.initCameraParameters();
  //
  //  viz.addPointCloudNormals<pcl17::PointNormal> (model, 1, 0.01, "cloud");
  //  viz.addPointCloudNormals<pcl17::PointNormal> (model_transformed, 1, 0.01, "cloud1");
  //  viz.spin();

  pcl17::SampleConsensusModel3DOF<pcl17::PointNormal>::Ptr
                                                       model_3dof(
                                                                  new pcl17::SampleConsensusModel3DOF<pcl17::PointNormal>(
                                                                                                                      model, 0.05f));
  model_3dof->setTarget(scene);

  pcl17::RandomSampleConsensusSimple<pcl17::PointNormal> ransac(model_3dof);
  ransac.setDistanceThreshold(.01);
  ransac.setProbability(0.99);
  ransac.computeModel();

  Eigen::VectorXf coeff(3);
  ransac.getModelCoefficients(coeff);

  std::cerr << "Ransac model (" << coeff[0] << "," << coeff[1] << "," << coeff[2] << ")" << std::endl;

  Eigen::Affine3f transform;
  transform.setIdentity();
  transform.translate(Eigen::Vector3f(coeff[0], coeff[1], 0));
  transform.rotate(Eigen::AngleAxisf(coeff[2], Eigen::Vector3f(0, 0, 1)));
  pcl17::transformPointCloudWithNormals(*model, *model_transformed, transform);

  pcl17::visualization::PCLVisualizer viz;
  viz.initCameraParameters();
  viz.updateCamera();

  pcl17::visualization::PointCloudColorHandlerCustom<pcl17::PointNormal> single_color(model_transformed, 255, 0, 0);

  viz.addPointCloud<pcl17::PointNormal> (scene);
  viz.addPointCloud<pcl17::PointNormal> (model_transformed, single_color, "cloud2");

  viz.spin();

  return 0;
}

/*
 * pcl_utils.h
 *
 *  Created on: Sep 26, 2012
 *      Author: kidson
 */

#ifndef PCL_UTILS_H_
#define PCL_UTILS_H_

#include "rgbd_registration/typedefs.h"
#include <pcl/kdtree/kdtree_flann.h>


void calculatePointCloudNormals (const PointCloudConstPtr input_cloud_ptr,
    PointCloudNormalsPtr output_cloud_ptr);

void removePointNormalsWithNaNs (const PointCloudNormalsPtr input_cloud_ptr);

void checkforNaNs (const PointCloudNormalsConstPtr input_cloud_ptr);

void writePCDToFile (const std::string& fileName, const PointCloudConstPtr cloud_ptr);

void writePCDToFile (const std::string& fileName, const PointCloudConstPtr cloud_ptr,
    const std::vector<int>& indices);
void transformAndWriteToFile (const PointCloudConstPtr cloud_in, const Eigen::Matrix4f& trafo);

void transformAndWriteToFile (const PointCloudConstPtr cloud_in, const std::vector<int>& indices,
    const Eigen::Matrix4f& trafo);


// ----------- TODO: Template ---------------------------------
void writePCDToFile (const std::string& fileName, const PointCloudNormalsConstPtr cloud_ptr);

void writePCDToFile (const std::string& fileName, const PointCloudNormalsConstPtr cloud_ptr,
    const std::vector<int>& indices);
void transformAndWriteToFile (const PointCloudNormalsConstPtr cloud_in, const Eigen::Matrix4f& trafo);

void transformAndWriteToFile (const PointCloudNormalsConstPtr cloud_in, const std::vector<int>& indices,
    const Eigen::Matrix4f& trafo);

template <class pointT>
inline void getIndicesFromMatches (typename pcl::PointCloud<pointT>::Ptr cloud_ptr, const std::vector<
    Eigen::Vector4f>& point_locations, std::vector<int>& indices)
{
  pcl::KdTreeFLANN<pointT> kdtreeNN;
  std::vector<int> pointIdxNKNSearch (1);
  std::vector<float> pointNKNSquaredDistance (1);
  kdtreeNN.setInputCloud (cloud_ptr);
  indices.clear ();
  for (size_t idx = 0; idx < point_locations.size (); idx++)
  {
    pointT test_point;
    test_point.x = point_locations[idx][0];
    test_point.y = point_locations[idx][1];
    test_point.z = point_locations[idx][2];
    kdtreeNN.nearestKSearch (test_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    indices.push_back (pointIdxNKNSearch[0]);
  }
}

#endif /* PCL_UTILS_H_ */

#ifndef POINTCLOUD_REGISTRATION_POINT_TYPES_H_
#define POINTCLOUD_REGISTRATION_POINT_TYPES_H_

#include <Eigen/Core>
#include <bitset>
#include <vector>
#include "pcl/ros/register_point_struct.h"
#include <pcl/point_types.h>

namespace pcl
{
  struct PointXYZINormal;
  struct PointXYZINormalScanIndex; //For use in ICP

  /** \brief A point structure representing the Spin Image Local Histogram. */
  struct SpinImageLocal;
  // Members: float histogram[100];

}

#include <pointcloud_registration/pointcloud_registration_point_types.hpp>  // Include struct definitions


// ==============================
// =====POINT_CLOUD_REGISTER=====
// ==============================

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZINormal,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (float, normal[0], normal_x)
                                   (float, normal[1], normal_y)
                                   (float, normal[2], normal_z)
                                   (float, curvature, curvature)
);
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZINormalScanIndex,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (float, normal[0], normal_x)
                                   (float, normal[1], normal_y)
                                   (float, normal[2], normal_z)
                                   (float, curvature, curvature)
                                   (int, scan_index, scan_index)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::SpinImageLocal,
                                   (uint32_t[100], histogram, sil)

);
#endif  //#ifndef POINTCLOUD_REGISTRATION_POINT_TYPES_H_

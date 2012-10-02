/*
 * joint_optimize_wrapper.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: kidson
 */

#ifndef JOINT_OPTIMIZE_WRAPPER_CPP_
#define JOINT_OPTIMIZE_WRAPPER_CPP_

#include "rgbd_registration/typedefs.h"

Eigen::Matrix4f performJointOptimization (PointCloudConstPtr source_cloud_ptr,
    PointCloudConstPtr target_cloud_ptr, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<
        Eigen::Vector4f> >& source_feature_3d_locations, std::vector<Eigen::Vector4f,
        Eigen::aligned_allocator<Eigen::Vector4f> >& target_feature_3d_locations,
    Eigen::Matrix4f& initial_transformation);

#endif /* JOINT_OPTIMIZE_WRAPPER_CPP_ */

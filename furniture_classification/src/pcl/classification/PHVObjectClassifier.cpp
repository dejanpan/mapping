/*
 * PHVObjectClassifier.cpp
 *
 *  Created on: Jun 7, 2012
 *      Author: vsu
 */

#include <pcl/point_types.h>
#include <pcl/features/sgfall.h>
#include <pcl/classification/PHVObjectClassifier.h>
#include <pcl/classification/impl/PHVObjectClassifier.hpp>

template class pcl::PHVObjectClassifier<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<pcl::SGFALL_SIZE> >;

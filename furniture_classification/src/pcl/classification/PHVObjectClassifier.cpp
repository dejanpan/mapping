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
#include <pcl/features/esf.h>
#include <pcl/features/vfh.h>



template class pcl::PHVObjectClassifier<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<pcl::SGFALL_SIZE> >;
template class pcl::PHVObjectClassifier<pcl::PointXYZ, pcl::PointNormal, pcl::ESFSignature640>;
template class pcl::PHVObjectClassifier<pcl::PointXYZ, pcl::PointNormal, pcl::VFHSignature308>;

/*
 * PHVObjectClassifier.cpp
 *
 *  Created on: Jun 7, 2012
 *      Author: vsu
 */

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <pcl17/point_types.h>
#include <pcl17/features/sgfall.h>
#include <pcl17/classification/PHVObjectClassifier.h>
#include <pcl17/classification/impl/PHVObjectClassifier.hpp>
#include <pcl17/features/esf.h>
#include <pcl17/features/vfh.h>




template class pcl17::PHVObjectClassifier<pcl17::PointXYZ, pcl17::PointNormal, pcl17::Histogram<pcl17::SGFALL_SIZE> >;
template class pcl17::PHVObjectClassifier<pcl17::PointXYZ, pcl17::PointNormal, pcl17::ESFSignature640>;
template class pcl17::PHVObjectClassifier<pcl17::PointXYZ, pcl17::PointNormal, pcl17::VFHSignature308>;

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_FEATURES_SGF2_H_
#define PCL_FEATURES_SGF2_H_

#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <numeric>

namespace pcl {
const int SGF2_SIZE = 1;

template<typename PointInT, typename PointOutT>
class SGF2Estimation: public Feature<PointInT, PointOutT> {

public:

	using Feature<PointInT, PointOutT>::feature_name_;
	using Feature<PointInT, PointOutT>::input_;
	using Feature<PointInT, PointOutT>::indices_;
	using Feature<PointInT, PointOutT>::search_parameter_;
	using Feature<PointInT, PointOutT>::k_;

	typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
	typedef typename Feature<PointInT, PointOutT>::PointCloudIn PointCloudIn;

	/** \brief Empty constructor. */
	SGF2Estimation() {
		feature_name_ = "SGF2Estimation";
		k_ = 1;
	}
	;

	/////////////////////////////////////////////////////////////////////////////
	void computeFeature(PointCloudOut &output) {

		PointCloud<Normal>::Ptr normals(new PointCloud<Normal> ());
		NormalEstimation<PointInT, Normal> n;

		std::vector<int> nn_indices;
		std::vector<float> nn_sqr_dists;
		Eigen::Vector4f parameters;
		Eigen::VectorXf vec(indices_->size());

		for (size_t idx = 0; idx < indices_->size(); ++idx) {
			float curvature;
			this->searchForNeighbors((*indices_)[idx], search_parameter_,
					nn_indices, nn_sqr_dists);
			n.computePointNormal(*input_, nn_indices, parameters, curvature);
			vec[idx] = curvature;
		}

		output.points[0].histogram[0] = vec.mean();

	}
	/////////////////////////////////////////////////////////////////////////////


private:

	/** \brief Make the computeFeature (&Eigen::MatrixXf); inaccessible from outside the class
	 * \param[out] output the output point cloud
	 */
	void computeFeatureEigen(pcl::PointCloud<Eigen::MatrixXf> &) {
	}
};
}

#endif  //#ifndef PCL_FEATURES_SGF2_H_

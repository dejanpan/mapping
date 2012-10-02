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

#ifndef PCL_FEATURES_SGF4_H_
#define PCL_FEATURES_SGF4_H_

#include <pcl17/features/feature.h>

namespace pcl17
{
  const int SGF4_SIZE = 6;

  template <typename PointInT, typename PointOutT>
  class SGF4Estimation : public Feature<PointInT, PointOutT>
  {

    public:

      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Feature<PointInT, PointOutT>::PointCloudIn  PointCloudIn;

      /** \brief Empty constructor. */
      SGF4Estimation ()
      {
        feature_name_ = "SGF4Estimation";
        k_ = 1;
      };


      /////////////////////////////////////////////////////////////////////////////
      void
      computeFeature (PointCloudOut &output)
      {
        // Copy the points specified by the index vector into a new cloud
        typename PointCloud<PointInT>::Ptr cloud (new PointCloud<PointInT> ());
        cloud->width = indices_->size ();
        cloud->height = 1;
        cloud->points.resize (cloud->width * cloud->height);
        for (size_t idx = 0; idx < indices_->size (); ++idx)
        {
          cloud->points[idx] = input_->points[(*indices_)[idx]];
        }


        // Compute eigenvectors and eigenvalues
        EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
        Eigen::Vector4f centroid3;
        compute3DCentroid (*cloud, centroid3);
        computeCovarianceMatrix (*cloud, centroid3, covariance_matrix);
        EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
        EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
        pcl17::eigen33 (covariance_matrix, eigen_vectors, eigen_values);


        // Compute the feature vector
        float eigen_sum = eigen_values.sum ();
        output.points[0].histogram[0] = eigen_values[1] != 0 ? eigen_values[0] / eigen_values[1] : 0;
        output.points[0].histogram[1] = eigen_values[2] != 0 ? eigen_values[1] / eigen_values[2] : 0;
        output.points[0].histogram[2] = eigen_values[2] != 0 ? eigen_values[0] / eigen_values[2] : 0;
        output.points[0].histogram[3] = eigen_sum != 0 ? eigen_values[0] / eigen_sum : 0;
        output.points[0].histogram[4] = eigen_sum != 0 ? eigen_values[1] / eigen_sum : 0;
        output.points[0].histogram[5] = eigen_sum != 0 ? eigen_values[2] / eigen_sum : 0;
      }
      /////////////////////////////////////////////////////////////////////////////


    private:

      /** \brief Make the computeFeature (&Eigen::MatrixXf); inaccessible from outside the class
       * \param[out] output the output point cloud
       */
      void
      computeFeatureEigen (pcl17::PointCloud<Eigen::MatrixXf> &) {}
  };
}

#endif  //#ifndef PCL_FEATURES_SGF4_H_

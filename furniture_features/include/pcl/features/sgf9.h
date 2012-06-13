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

#ifndef PCL_FEATURES_SGF9_H_
#define PCL_FEATURES_SGF9_H_

#include <pcl/features/feature.h>
#include <pcl/features/sgf3.h>

namespace pcl
{
  const int SGF9_SIZE = 1;

  template <typename PointInT, typename PointOutT>
  class SGF9Estimation : public Feature<PointInT, PointOutT>
  {

    public:

      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::tree_;
      using Feature<PointInT, PointOutT>::k_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Feature<PointInT, PointOutT>::PointCloudIn  PointCloudIn;

      /** \brief Empty constructor. */
      SGF9Estimation ()
      {
        feature_name_ = "SGF9Estimation";
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


        // Compute feature 3
        const int sgf3_size = 1;
        pcl::PointCloud<pcl::Histogram<sgf3_size> >::Ptr sgf3s (new pcl::PointCloud<pcl::Histogram<sgf3_size> > ());
        pcl::SGF3Estimation<PointInT, pcl::Histogram<sgf3_size> > sgf3;
        sgf3.setInputCloud (cloud);
        sgf3.compute (*sgf3s);


        // Compute eigenvectors and eigenvalues
        EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
        Eigen::Vector4f centroid3;
        compute3DCentroid (*cloud, centroid3);
        computeCovarianceMatrix (*cloud, centroid3, covariance_matrix);
        EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
        EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
        pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);


        // Compute the volume of the oriented bounding box
        float box_vol = 8 * eigen_values[0] * eigen_values[1] * eigen_values[2];


        // Compute the feature vector
        output.points[0].histogram[0] = box_vol != 0 ? sgf3s->points[0].histogram[0] / box_vol : 0;
      }
      /////////////////////////////////////////////////////////////////////////////


    private:

      /** \brief Make the computeFeature (&Eigen::MatrixXf); inaccessible from outside the class
       * \param[out] output the output point cloud
       */
      void
      computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &) {}
  };
}

#endif  //#ifndef PCL_FEATURES_SGF9_H_

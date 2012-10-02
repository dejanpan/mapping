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

#ifndef PCL_FEATURES_SGF1_H_
#define PCL_FEATURES_SGF1_H_

#include <pcl17/features/feature.h>
#include <pcl17/features/normal_3d.h>
#include <pcl17/features/boundary.h>

namespace pcl17
{
  const int SGF1_SIZE = 1;

  template <typename PointInT, typename PointOutT>
  class SGF1Estimation : public Feature<PointInT, PointOutT>
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
      SGF1Estimation ()
      {
        feature_name_ = "SGF1Estimation";
      };


      /////////////////////////////////////////////////////////////////////////////
      void
      computeFeature (PointCloudOut &output)
      {
        Eigen::Vector4f u = Eigen::Vector4f::Zero ();
        Eigen::Vector4f v = Eigen::Vector4f::Zero ();

        std::vector<int> nn_indices;
        std::vector<float> nn_sqr_dists;


        // Estimate normals first
        PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
        NormalEstimation<PointInT, Normal> n;
        n.setInputCloud (input_);
        n.setIndices (indices_);
        n.setSearchMethod (tree_);
        n.setKSearch (k_);
        n.compute (*normals);

        BoundaryEstimation<PointInT, Normal, Boundary> b;
        b.setInputNormals (normals);


        // Count the boundary points
        int nrOfBoundaryPoints = 0;
        for (size_t idx = 0; idx < indices_->size (); ++idx)
        {
          this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_sqr_dists);
          b.getCoordinateSystemOnPlane (normals->points[idx], u, v);
          if (b.isBoundaryPoint (*surface_, (*indices_)[idx], nn_indices, u, v, M_PI / 2.0))
          {
            nrOfBoundaryPoints += 1;
          }
        }


        // Compute the feature vector
        output.points[0].histogram[0] = (float) nrOfBoundaryPoints / indices_->size ();
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

#endif  //#ifndef PCL_FEATURES_SGF1_H_

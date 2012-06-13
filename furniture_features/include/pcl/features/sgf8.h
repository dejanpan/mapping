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

#ifndef PCL_FEATURES_SGF8_H_
#define PCL_FEATURES_SGF8_H_

#include <pcl/features/feature.h>
#include <pcl/features/sgf5.h>
#include <pcl/features/sgf7.h>

namespace pcl
{
  const int SGF8_SIZE = 3;

  template <typename PointInT, typename PointOutT>
  class SGF8Estimation : public Feature<PointInT, PointOutT>
  {

    public:

      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Feature<PointInT, PointOutT>::PointCloudIn  PointCloudIn;

      /** \brief Empty constructor. */
      SGF8Estimation ()
      {
        feature_name_ = "SGF8Estimation";
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


        // Compute feature 5
        const int sgf5_size = 3;
        pcl::PointCloud<pcl::Histogram<sgf5_size> >::Ptr sgf5s (new pcl::PointCloud<pcl::Histogram<sgf5_size> > ());
        pcl::SGF5Estimation<PointInT, pcl::Histogram<sgf5_size> > sgf5;
        sgf5.setInputCloud (cloud);
        sgf5.compute (*sgf5s);


        // Compute feature 7
        const int sgf7_size = 7;
        pcl::PointCloud<pcl::Histogram<sgf7_size> >::Ptr sgf7s (new pcl::PointCloud<pcl::Histogram<sgf7_size> > ());
        pcl::SGF7Estimation<PointInT, pcl::Histogram<sgf7_size> > sgf7;
        sgf7.setInputCloud (cloud);
        sgf7.compute (*sgf7s);


        // Compute the feature vector
        output.points[0].histogram[0] = sgf7s->points[0].histogram[0] != 0 ? sgf5s->points[0].histogram[0] / sgf7s->points[0].histogram[0] : 0;
        output.points[0].histogram[1] = sgf7s->points[0].histogram[1] != 0 ? sgf5s->points[0].histogram[1] / sgf7s->points[0].histogram[1] : 0;
        output.points[0].histogram[2] = sgf7s->points[0].histogram[2] != 0 ? sgf5s->points[0].histogram[2] / sgf7s->points[0].histogram[2] : 0;
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

#endif  //#ifndef PCL_FEATURES_SGF8_H_

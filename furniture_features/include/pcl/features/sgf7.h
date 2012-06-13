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

#ifndef PCL_FEATURES_SGF7_H_
#define PCL_FEATURES_SGF7_H_

#include <pcl/features/feature.h>

namespace pcl
{
  const int SGF7_SIZE = 7;

  template <typename PointInT, typename PointOutT>
  class SGF7Estimation : public Feature<PointInT, PointOutT>
  {

    public:

      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Feature<PointInT, PointOutT>::PointCloudIn  PointCloudIn;

      /** \brief Empty constructor. */
      SGF7Estimation ()
      {
        feature_name_ = "SGF7Estimation";
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
        pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);
        Eigen::Vector3f e1 (eigen_vectors (0, 0), eigen_vectors (1, 0), eigen_vectors (2, 0));
        Eigen::Vector3f e2 (eigen_vectors (0, 1), eigen_vectors (1, 1), eigen_vectors (2, 1));
        Eigen::Vector3f e3 (eigen_vectors (0, 2), eigen_vectors (1, 2), eigen_vectors (2, 2));


        // Project the cloud onto the eigenvectors
        typename PointCloud<PointXYZ>::Ptr proj_cloud (new PointCloud<PointXYZ> ());
        proj_cloud->width = cloud->width * cloud->height;
        proj_cloud->height = 1;
        proj_cloud->points.resize (proj_cloud->width * proj_cloud->height);
        for (size_t idx = 0; idx < cloud->width * cloud->height; ++idx)
        {
          Eigen::Vector3f curr_point (cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
          proj_cloud->points[idx].x = curr_point.dot (e1);
          proj_cloud->points[idx].y = curr_point.dot (e2);
          proj_cloud->points[idx].z = curr_point.dot (e3);
        }


        // Sort the projected cloud
        float proj_cloud1[cloud->width * cloud->height];
        float proj_cloud2[cloud->width * cloud->height];
        float proj_cloud3[cloud->width * cloud->height];
        for (size_t idx = 0; idx < cloud->width * cloud->height; ++idx)
        {
          proj_cloud1[idx] = proj_cloud->points[idx].x;
          proj_cloud2[idx] = proj_cloud->points[idx].y;
          proj_cloud3[idx] = proj_cloud->points[idx].z;
        }
        std::sort (proj_cloud1, proj_cloud1 + cloud->width * cloud->height);
        std::sort (proj_cloud2, proj_cloud2 + cloud->width * cloud->height);
        std::sort (proj_cloud3, proj_cloud3 + cloud->width * cloud->height);


        // Compute the median?
        float med1 = proj_cloud1[cloud->width * cloud->height / 2];
        float med2 = proj_cloud2[cloud->width * cloud->height / 2];
        float med3 = proj_cloud3[cloud->width * cloud->height / 2];


        // Compute the distance to the farthest points
        float l1_e1 = med1 - proj_cloud1[0];
        float l2_e1 = proj_cloud1[cloud->width * cloud->height - 1] - med1;
        float l1_e2 = med2 - proj_cloud2[0];
        float l2_e2 = proj_cloud2[cloud->width * cloud->height - 1] - med2;
        float l1_e3 = med3 - proj_cloud3[0];
        float l2_e3 = proj_cloud3[cloud->width * cloud->height - 1] - med3;


        // Compute the feature vector
        output.points[0].histogram[0] = l1_e1 + l2_e1;
        output.points[0].histogram[1] = l1_e2 + l2_e2;
        output.points[0].histogram[2] = l1_e3 + l2_e3;
        output.points[0].histogram[3] = l2_e1 != 0 ? l1_e1 / l2_e1 : 0;
        output.points[0].histogram[4] = l2_e2 != 0 ? l1_e2 / l2_e2 : 0;
        output.points[0].histogram[5] = l2_e3 != 0 ? l1_e3 / l2_e3 : 0;
        output.points[0].histogram[6] = l1_e2 + l2_e2 != 0 ? (l1_e1 + l2_e1) / (l1_e2 + l2_e2) : 0;
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

#endif  //#ifndef PCL_FEATURES_SGF7_H_

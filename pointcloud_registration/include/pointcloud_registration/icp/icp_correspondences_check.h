/*
 * Copyright (c) 2010, Hozefa Indorewala <indorewala@ias.in.tum.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Based on orignal code written by Radu Rusu.
 * Modified by Hozefa Indorewala
 */
#include <pointcloud_registration/icp/registration_correspondences_check.h>

#ifndef _ICP_CORRESPONDENCES_CHECK_H_
#define _ICP_CORRESPONDENCES_CHECK_H_

namespace pcl
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  template <typename PointSource, typename PointTarget>
  class IterativeClosestPointCorrespondencesCheck : public RegistrationCorrespondencesCheck<PointSource, PointTarget>
  {
    using RegistrationCorrespondencesCheck<PointSource, PointTarget>::reg_name_;
    using RegistrationCorrespondencesCheck<PointSource, PointTarget>::getClassName;
    using RegistrationCorrespondencesCheck<PointSource, PointTarget>::input_;
    using RegistrationCorrespondencesCheck<PointSource, PointTarget>::indices_;
    using RegistrationCorrespondencesCheck<PointSource, PointTarget>::target_;
    using RegistrationCorrespondencesCheck<PointSource, PointTarget>::nr_iterations_;
    using RegistrationCorrespondencesCheck<PointSource, PointTarget>::max_iterations_;
    using RegistrationCorrespondencesCheck<PointSource, PointTarget>::previous_transformation_;
    using RegistrationCorrespondencesCheck<PointSource, PointTarget>::final_transformation_;
    using RegistrationCorrespondencesCheck<PointSource, PointTarget>::transformation_;
    using RegistrationCorrespondencesCheck<PointSource, PointTarget>::transformation_epsilon_;
    using RegistrationCorrespondencesCheck<PointSource, PointTarget>::converged_;

    typedef typename RegistrationCorrespondencesCheck<PointSource, PointTarget>::PointCloudSource PointCloudSource;
    typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
    typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

    typedef typename RegistrationCorrespondencesCheck<PointSource, PointTarget>::PointCloudTarget PointCloudTarget;

    typedef PointIndices::Ptr PointIndicesPtr;
    typedef PointIndices::ConstPtr PointIndicesConstPtr;

    double radius_, epsilon_z_, epsilon_curvature_;
    int max_nn_;
    bool curvature_check_;

    public:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty constructor. */

      IterativeClosestPointCorrespondencesCheck(){reg_name_ = "IterativeClosestPointCorrespondencesCheck";};
      /** \brief Parameterized constructor. */
      IterativeClosestPointCorrespondencesCheck ( double radius, int max_nn, double epsilon_z, double epsilon_curvature, bool curvature_check):
                                                  radius_(radius),
                                                  max_nn_(max_nn),
                                                  epsilon_z_(epsilon_z),
                                                  epsilon_curvature_(epsilon_curvature),
                                                  curvature_check_(curvature_check){reg_name_ = "IterativeClosestPointCorrespondencesCheck";};

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty destructor. */
      virtual ~IterativeClosestPointCorrespondencesCheck () {};

    protected:

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Rigid transformation computation method.
        * \param output the transformed input point cloud dataset using the rigid transformation found
        */
      virtual void computeTransformation (PointCloudSource &output)
      {
        // Allocate enough space to hold the results
        std::vector<int> nn_indices (max_nn_);
        std::vector<float> nn_dists (max_nn_);


        this->nr_iterations_ = 0;
        while (!this->converged_)           // repeat until convergence
        {
          // Point cloud containing the correspondences of each point in <input, indices>
          PointCloudTarget model_corresp;
          PointCloudSource source_corresp;

          source_corresp.points.reserve(this->indices_->size());
          model_corresp.points.reserve(this->indices_->size());
          // Save the previously estimated transformation
          this->previous_transformation_ = this->transformation_;

          // Iterating over the entire index vector and  find all correspondences
          for (size_t idx = 0; idx < this->indices_->size (); idx++)
          {
            // Use radius search to look for neighbors within a user specified radius
            if (!searchForNeighbors (output, idx, radius_, max_nn_, nn_indices, nn_dists))
            {
              continue;
            }

            for(size_t i = 0 ; i < nn_indices.size(); i++)
            {
              // Check if the difference between the z coordinates is within user specified limits
              if(fabs(output.points[idx].z - this->target_->points[nn_indices[i]].z) < epsilon_z_)
              {
                if(curvature_check_)
                {
                  // Check if the difference between the curvature values is within user specified limits
                  if(fabs(output.points[idx].curvature - this->target_->points[nn_indices[i]].curvature) > epsilon_curvature_)
                  {
                    continue;
                  }
                }
                source_corresp.points.push_back (output.points[idx]);
                model_corresp.points.push_back (this->target_->points[nn_indices[i]]);
                break;
              }
            }
          }
          // Zero the Z coordinate value since the robot moved in the x & y plane only
          for(size_t i = 0; i < source_corresp.points.size(); i++)
          {
            source_corresp.points[i].z = 0.0;
            model_corresp.points[i].z = 0.0;
          }

          // Estimate the transform
          estimateRigidTransformationSVD (source_corresp, model_corresp, this->transformation_);

          // Tranform the data
          transformPointCloud (output, output, this->transformation_);

          // Obtain the final transformation
          this->final_transformation_ = this->transformation_ * this->final_transformation_;

          //Compute transformation change
          double transformation_change = fabs ((this->transformation_ - this->previous_transformation_).sum ());

          this->nr_iterations_++;
          // Check for convergence
          if (this->nr_iterations_ >= this->max_iterations_ ||
              transformation_change < this->transformation_epsilon_)
          {
            this->converged_ = true;
            ROS_INFO ("[pcl::%s::computeTransformation] Convergence reached. Number of iterations: %d out of %d. Transformation difference: %g",
                      getClassName().c_str(), this->nr_iterations_, this->max_iterations_, fabs ((this->transformation_ - this->previous_transformation_).sum ()));
          }
        }
      }

    public:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using SVD.
        * \param cloud_src the source point cloud dataset
        * \param cloud_tgt the target point cloud dataset
        * \param transformation_matrix the resultant transformation matrix
        */

      void estimateRigidTransformationSVD (const PointCloudSource &cloud_src, const PointCloudTarget &cloud_tgt, Eigen::Matrix4f &transformation_matrix)
      {
        ROS_ASSERT (cloud_src.points.size () == cloud_tgt.points.size ());

        // <cloud_src,cloud_src> is the source dataset
        transformation_matrix.setIdentity ();

        Eigen::Vector4f centroid_src, centroid_tgt;
        // Estimate the centroids of source, target
        compute3DCentroid (cloud_src, centroid_src);
        compute3DCentroid (cloud_tgt, centroid_tgt);

        // Subtract the centroids from source, target
        Eigen::MatrixXf cloud_src_demean;
        demeanPointCloud (cloud_src, centroid_src, cloud_src_demean);


        Eigen::MatrixXf cloud_tgt_demean;
        demeanPointCloud (cloud_tgt, centroid_tgt, cloud_tgt_demean);

        // Assemble the correlation matrix H = source * target'
        Eigen::Matrix3f H = (cloud_src_demean * cloud_tgt_demean.transpose ()).corner<3, 3>(Eigen::TopLeft);

        // Compute the Singular Value Decomposition
        Eigen::SVD<Eigen::Matrix3f> svd (H);
        Eigen::Matrix3f u = svd.matrixU ();
        Eigen::Matrix3f v = svd.matrixV ();

        // Compute R = V * U'
        //if (u.determinant () * v.determinant () < 0)
        //{
        //  ROS_WARN ("[pcl::estimateRigidTransformationSVD] Determinant < 0!");
        //  for (int x = 0; x < 3; ++x)
        //    v (x, 2) *= -1;
        //}

        Eigen::Matrix3f R = v * u.transpose ();

        // Return the correct transformation
        transformation_matrix.corner<3, 3> (Eigen::TopLeft) = R;
        Eigen::Vector3f Rc = R * centroid_src.start<3> ();
        transformation_matrix.block <3, 1> (0, 3) = centroid_tgt.start<3> () - Rc;

        // Assemble the correlation matrix H = source' * target
        //Eigen::Matrix3f H = (cloud_src_demean.transpose () * cloud_tgt_demean).corner<3, 3>(Eigen::TopLeft);

        // Compute the Singular Value Decomposition
        //Eigen::SVD<Eigen::Matrix3f> svd (H);
        //Eigen::Matrix3f u = svd.matrixU ();
        //Eigen::Matrix3f v = svd.matrixV ();

        // Compute R = V * U'
        //if (u.determinant () * v.determinant () < 0)
          //for (int x = 0; x < 3; ++x)
            //v (x, 2) *= -1;

        //Eigen::Matrix3f R = u * v.transpose ();
        //Eigen::Matrix3f Rinv;
        //R.computeInverse (&Rinv);
        // Return the correct transformation
        //transformation_matrix.corner<3, 3> (Eigen::TopLeft) = Rinv;
        //Eigen::Vector3f Rc = Rinv * centroid_src.start<3> ();
        //transformation_matrix.block <3, 1> (0, 3) = centroid_tgt.start<3> () - Rc;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief A simple method to set the parameters */
      void setParameters( double radius, int max_nn, double epsilon_z, double epsilon_curvature, bool curvature_check)
      {
        radius_ = radius;
        max_nn_ = max_nn;
        epsilon_z_ = epsilon_z;
        epsilon_curvature_ = epsilon_curvature;
        curvature_check_ = curvature_check;
      }
  };
}

#endif  //#ifndef _ICP_CORRESPONDENCES_CHECK_H_

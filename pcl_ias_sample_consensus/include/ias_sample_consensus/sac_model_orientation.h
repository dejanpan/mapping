/* 
 * Copyright (c) 2010, Zoltan Csaba Marton <marton@cs.tum.edu>
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

#ifndef IAS_SAMPLE_CONSENSUS_SACMODELORIENTATION_H_
#define IAS_SAMPLE_CONSENSUS_SACMODELORIENTATION_H_

// SaC
#include <point_cloud_mapping/sample_consensus/sac_model.h>

// Kd Tree
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

// Eigen
#include <Eigen/Array>
#include <Eigen/Geometry>

namespace ias_sample_consensus
{
  /** \brief Given a point/vector, an axis and an angle, it returns the rotated input
   * \note if it needs to be done multiple times, computing the rotation matrix and applying it is faster! -- see: cloud_geometry::transforms::convertAxisAngleToRotationMatrix ()
   * \param r the point or vector to be rotated
   * \param axis the rotation axis
   * \param angle the rotation angle in radians
   */
  inline Eigen::Vector3f rotateAroundAxis (Eigen::Vector3f r, Eigen::Vector3f axis, float angle)
  {
    // Rodrigues' rotation formula
    return r * cos(angle) + (axis.cross(r)) * sin(angle) + axis * axis.dot(r) * (1-cos(angle));
  }

  /** \brief A Sample Consensus Model class for determining the 2 perpendicular directions to which most normals align.
   */
  class SACModelOrientation : public sample_consensus::SACModel
  {
    public:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief The fixed axis. */
      Eigen::Vector3f axis_;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief The inlier threshold for normals, in radians. */
      //float eps_angle_;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor. */
      SACModelOrientation () : sample_consensus::SACModel ()
      { 
        // nx needs to be set
        nx_idx_ = -1;
        axis_[0] = 0;
        axis_[1] = 0;
        axis_[2] = 1;
        //eps_angle_ = 0.1;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Destructor for SACModelOrientation. */
      //~SACModelOrientation () {}

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the dataset -- and initialize internal variables
       * \param cloud the data set to be used */
      inline void
        setDataSet (sensor_msgs::PointCloud *cloud)
      {
        SACModel::setDataSet (cloud);
        init (cloud);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the dataset and indices -- and initialize internal variables
       * \param cloud the data set to be used
       * \param indices the point indices used
       */
      inline void
        setDataSet (sensor_msgs::PointCloud *cloud, std::vector<int> indices)
      {
        SACModel::setDataSet (cloud, indices);
        init (cloud);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get the minimum and maximum distances along the axes and the corresponding index in the indices_ vector.
        * \param model_coefficients the model coefficients that should be used for getting the axes
        * \param inliers the data inliers found as supporting the model (each element is an index of indices_)
        */
      virtual void getMinAndMax (std::vector<double> *model_coefficients, std::vector<int> *inliers, std::vector<int> &min_max_indices, std::vector<float> &min_max_distances);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get a random point and return its index.
        * \param iterations the internal number of iterations used by SAC methods (incremented at samplings that can not produce a model) -- Not needed.
        * \param samples the resultant model samples
        */
      virtual void getSamples (int &iterations, std::vector<int> &samples);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Test whether the given model coefficients are valid given the input point cloud data.
        * @note Since the model coefficient is a point with a normal, it comes down to accepting the point as "source" or not.
        * \param model_coefficients the model coefficients that need to be tested
        */
      virtual bool testModelCoefficients (const std::vector<double> &model_coefficients);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Check whether the given index samples can form a valid model, compute the model coefficients from
        * these samples and store them internally in model_coefficients_.
        * @note Model coefficients are the normal of the sample point (and it's index for testModelCoefficients)
        * \param samples the point indices found as possible good candidates for creating a valid model
        */
      virtual bool computeModelCoefficients (const std::vector<int> &samples);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Recompute the model coefficients using the given inlier set and return them to the user. Pure virtual.
        * @note: These are the coefficients of the model after refinement (eg. after a least-squares optimization)
        * \param inliers the data inliers found as supporting the model (each element is an index of indices_)
        * \param refit_coefficients the resultant recomputed coefficients (has -1 as source index in refit_coefficients[3] at failures)
        */
      virtual void refitModel (const std::vector<int> &inliers, std::vector<double> &refit_coefficients);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Compute all distances from the cloud data to a given model. -- Not needed.
        * \param model_coefficients the coefficients of a model that we need to compute distances to
        * \param distances the resultant estimated distances
        */
      virtual void getDistancesToModel (const std::vector<double> &model_coefficients, std::vector<double> &distances) {}

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Select all the points which respect the given model coefficients as inliers.
        * \param model_coefficients the coefficients of a model that we need to compute distances to
        * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
        * \param inliers the resultant model inliers (each element is an index of indices_)
        * @note: To get the refined inliers of a model, use:
        *        refined_coeff = refitModel (...); selectWithinDistance (refined_coeff, threshold);
        */
      virtual void selectWithinDistance (const std::vector<double> &model_coefficients, double threshold, std::vector<int> &inliers);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Create a new point cloud with inliers projected onto the model. -- Not needed.
        * \param inliers the data inliers that we want to project on the model (each element is an index of indices_)
        * \param model_coefficients the coefficients of a model
        * \param projected_points the resultant projected points
        */
      virtual void projectPoints (const std::vector<int> &inliers, const std::vector<double> &model_coefficients, sensor_msgs::PointCloud &projected_points) {}

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Project inliers (in place) onto the given model. -- Not needed.
        * \param inliers the data inliers that we want to project on the model (each element is an index of indices_)
        * \param model_coefficients the coefficients of a model
        */
      virtual void projectPointsInPlace (const std::vector<int> &inliers, const std::vector<double> &model_coefficients) {}

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Verify whether a subset of indices verifies the internal model coefficients. -- Not needed.
        * \param indices the data indices that need to be tested against the model (each element is an index of indices_)
        * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
        */
      virtual bool doSamplesVerifyModel (const std::set<int> &indices, double threshold) { return true; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return an unique id for each type of model employed. */
      virtual int getModelType () { return 1001; }

    protected:

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Creates a kd-tree for searching the normals and locates the nx channel.
       * @note We assume that if nx exists then ny and nz are the channels following it.
       * \param cloud the data set to be used
       */
       void init (sensor_msgs::PointCloud *cloud);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Index of the nx channel (the x component of the normal vector). The next indices are assumed to be that of ny and nz! */
      int nx_idx_;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Kd-Tree for searching in normal space. */
      cloud_kdtree::KdTree *kdtree_;
      std::vector<int> front_indices_;
      std::vector<int>  back_indices_;
      std::vector<int>  left_indices_;
      std::vector<int> right_indices_;
      std::vector<float> points_sqr_distances_;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Normals copied as x y and z values for creating the Kd-Tree. */
      sensor_msgs::PointCloud normals_;
  };
}

#endif

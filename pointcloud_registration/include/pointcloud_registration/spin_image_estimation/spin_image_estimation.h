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
 * Authors: Zoltan-Csaba Marton, Hozefa Indorewala
 */

#ifndef _SPIN_IMAGE_ESTIMATION_H_
#define _SPIN_IMAGE_ESTIMATION_H_

#include <iostream>
#include <pcl/features/feature.h>

namespace pcl
{
  template <typename PointInT, typename PointNT, typename PointOutT>
  class SpinImageEstimation: public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Feature<PointInT, PointOutT>::PointCloudIn  PointCloudIn;

    public:
      //empty contructor
      SpinImageEstimation() {};

      //empty virtual destructor
      virtual ~SpinImageEstimation(){};

      inline bool
        computeAlphaBeta (const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals,
                             int p_idx, int q_idx, float &alpha, float &beta)
      {
        if( p_idx == q_idx)
          return(false);
        Eigen::Vector3f delta_vector, u;

        // Compute the Cartesian difference between the two points i.e. (p_idx - q_idx)
        delta_vector[0] = cloud.points[p_idx].x - cloud.points[q_idx].x;
        delta_vector[1] = cloud.points[p_idx].y - cloud.points[q_idx].y;
        delta_vector[2] = cloud.points[p_idx].z - cloud.points[q_idx].z;

        if (delta_vector.squaredNorm() == 0.0 )
        {
          ROS_ERROR ("Euclidean distance between points %d and %d is 0!", p_idx, q_idx);
          return (false);
        }

        //Eigen::Vector4f u = Eigen::Vector4f::Map (normals.points[q_idx].normal[0], 3);
        u[0] = normals.points[q_idx].normal[0];
        u[1] = normals.points[q_idx].normal[1];
        u[2] = normals.points[q_idx].normal[2];

        // Estimate beta_vector= u * delta
        beta = fabs(delta_vector.dot(u));

        // Estimate alpha_vector = (- delta_vecto)r x (- u)
        alpha = ( ( -delta_vector ).cross( -u ) ).norm();

        return (true);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the number of subdivisions for each angular feature interval.
        * \param nr_subdiv the number of subdivisions
        */
      inline void
        setNrSubdivisions (int nr_subdiv)
      {
        nr_subdiv_ = nr_subdiv;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get the number of subdivisions for the feature interval.
        * \param nr_subdiv the resultant number of subdivisions as set by the user
        */
      inline void
        getNrSubdivisions (int &nr_subdiv)
      {
        nr_subdiv = nr_subdiv_;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the radius of the neigborhood
        * \param radius the radius of the neighborhood
        */
      inline void
        setRadius (double radius)
      {
        radius_ = radius;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get the radius of the neigborhood
        * \param radius the resultant radius of the neigborhood as set by the user
        */
      inline void
        getRadius (double &radius)
      {
        radius = radius_;
      }

    private:

      inline void
        computeFeature (PointCloudOut &output)
      {
        // Check if input was set
        if (!this->normals_)
        {
          ROS_ERROR ("[pcl::%s::computeFeature] No input dataset containing normals was given!", getName ().c_str ());
          return;
        }
        if (this->normals_->points.size () != this->surface_->points.size ())
        {
          ROS_ERROR ("[pcl::%s::computeFeature] The number of points in the input dataset differs from the number of points in the dataset containing the normals!", getName ().c_str ());
          return;
        }

        spin_image_histogram_.setZero (nr_subdiv_ * nr_subdiv_);

        // Allocate enough space to hold the results
        // \note This resize is irrelevant for a radiusSearch ().
        std::vector<int> nn_indices (this->k_);
        std::vector<float> nn_dists (this->k_);

        double step_size = radius_ / nr_subdiv_;

        // Iterating over the entire index vector
        for (size_t idx = 0; idx < this->indices_->size (); ++idx)
        {
          searchForNeighbors ((*this->indices_)[idx], this->search_parameter_, nn_indices, nn_dists);

          //select only those neigbors that lie within the radius
          for(size_t i = 0 ; i < nn_indices.size(); i++)
          {
            if(sqrt(nn_dists[i]) > radius_)
              continue;
            else
            {
              float alpha, beta;
              if( computeAlphaBeta(*this->surface_, *this->normals_, (*this->indices_)[idx], nn_indices[i], alpha, beta) )
              {
                int alpha_idx = (int) floor( alpha / step_size);
                int beta_idx = (int)  floor( beta / step_size);
                spin_image_histogram_[ alpha_idx + beta_idx * nr_subdiv_] ++ ;
              }
            }
          }

          // Copy into the resultant cloud
          for (int d = 0; d < spin_image_histogram_.size (); ++d)
            output.points[idx].histogram[d] = spin_image_histogram_[d];

          // set the histogram values back to zero
          spin_image_histogram_.setZero();
        }

      }

    private:
      /** \brief The radius for the search. */
      double radius_;

      /** \brief The number of subdivisions. */
      int nr_subdiv_;

      /** \brief Placeholder for a point's Spin Image signature. */
      Eigen::VectorXf spin_image_histogram_;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get a string representation of the name of this class. */
      std::string getName () const { return ("SpinImageEstimation"); }

  }; //class SpinImageEstimation
} // end namespace pcl

#endif /* #ifndef _SPIN_IMAGE_ESTIMATION_H_ */

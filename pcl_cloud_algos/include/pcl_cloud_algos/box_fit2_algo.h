/*
 * Copyright (c) 2010, Zoltan-Csaba Marton <marton@cs.tum.edu>
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

#ifndef PCL_CLOUD_ALGOS_ROBUST_BOX_ESTIMATION_H
#define PCL_CLOUD_ALGOS_ROBUST_BOX_ESTIMATION_H

//cloud_algos
#include <pcl_cloud_algos/box_fit_algo.h>

#include <pcl_ias_sample_consensus/pcl_sac_model_orientation.h>
namespace pcl_cloud_algos
{

class RobustBoxEstimation : public BoxEstimation
{
 public:

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Inlier threshold for normals in radians
   */
  double eps_angle_;

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief If it is not in the (0,1) interval, exhaustive search will be done, otherwise
   *  it will be the probability to be set for RANSAC (tradeoff between speed and accuracy).
   */
  double success_probability_;

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Constructor and destructor
   */
  RobustBoxEstimation ()
  {
    eps_angle_ = 0.1; // approximately 6 degrees
    success_probability_ = 1; // for using SAC replace with 0.99
  };

  void getMinAndMax(Eigen::VectorXf model_coefficients, boost::shared_ptr<pcl::SACModelOrientation<pcl::PointXYZINormal> > model, std::vector<int> &min_max_indices, std::vector<float> &min_max_distances);

  // Overwritten Cloud Algo stuff
  void pre  ();
  std::vector<std::string> requires ();

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Function for actual SaC-based model fitting
   * \param cloud de-noisified input point cloud message with normals
   * \param coeff box to-be-filled-in coefficients (15 elements):
   * box center: cx, cy, cz,
   * box dimensions: dx, dy, dz,
   * box robust axes: e1_x, e1y, e1z, e2_x, e2y, e2z, e3_x, e3y, e3z
   */
  virtual bool find_model (boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZINormal> > cloud, std::vector<double> &coeff);

  ros::Publisher createPublisher (ros::NodeHandle& nh)
  {
    ros::Publisher p = nh.advertise<OutputType> (default_output_topic (), 5);
    return p;
  }
};

}
#endif



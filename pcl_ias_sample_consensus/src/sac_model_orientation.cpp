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

#include <ias_sample_consensus/sac_model_orientation.h>
#include <float.h>

namespace ias_sample_consensus
{
  void
    SACModelOrientation::init (sensor_msgs::PointCloud *cloud)
  {
    // Locate channels with normals
    for (unsigned int d = 0; d < cloud->channels.size (); d++)
      if (cloud->channels[d].name == "nx")
      {
        nx_idx_ = d;
        break;
      }
    if (nx_idx_ == -1)
    {
      ROS_ERROR ("[SACModelOrientation] Provided point cloud does not have normals!");
      return;
    }

    // Copy normals of points into a PCD -- not the most optimal way of creating the kd-tree, but the only one easily available as of this writing :)
    //normals_ = new sensor_msgs::PointCloud;
    normals_.points.resize (indices_.size ()); // we don't care about header and channels
    for (unsigned i = 0; i < normals_.points.size (); i++)
    {
      normals_.points[i].x = cloud->channels[nx_idx_+0].values[indices_[i]];
      normals_.points[i].y = cloud->channels[nx_idx_+1].values[indices_[i]];
      normals_.points[i].z = cloud->channels[nx_idx_+2].values[indices_[i]];
    }

    // Create kd-tree
    kdtree_ = new cloud_kdtree::KdTreeANN (normals_);
  }

  void
    SACModelOrientation::getMinAndMax (std::vector<double> *model_coefficients, std::vector<int> *inliers, std::vector<int> &min_max_indices, std::vector<float> &min_max_distances)
  {
    // Initialize result vectors
    min_max_indices.resize (6);
    min_max_distances.resize (6);
    min_max_distances[0] = min_max_distances[2] = min_max_distances[4] = +DBL_MAX;
    min_max_distances[1] = min_max_distances[3] = min_max_distances[5] = -DBL_MAX;

    // The 3 coordinate axes are nm, nc and axis_
    Eigen::Vector3f nm = Eigen::Vector3d::Map(&(*model_coefficients)[0]).cast<float> ();
    Eigen::Vector3f nc = axis_.cross (nm);

    // Find minimum and maximum distances from origin along the three axes
    for (std::vector<int>::iterator it = inliers->begin (); it != inliers->end (); it++)
    //for (unsigned i = 0; i < inliers.size (); i++)
    {
      /// @NOTE inliers is a list of indices of the indices_ array!
      Eigen::Vector3f point (cloud_->points[indices_[*it]].x, cloud_->points[indices_[*it]].y, cloud_->points[indices_[*it]].z);
      //Eigen::Vector3f point (cloud_->points[indices_[*it]].x - center.x, cloud_->points[indices_[*it]].y - center.y, cloud_->points[indices_[*it]].z - center.z);
      //Eigen::Vector3f point (cloud_->points[indices_[inliers[i]]].x, cloud_->points[indices_[inliers[i]]].y, cloud_->points[indices_[inliers[i]]].z);
      double dists[3];
      dists[0] = nm.dot(point);
      dists[1] = nc.dot(point);
      dists[2] = axis_.dot(point);
      for (int d=0; d<3; d++)
      {
        if (min_max_distances[2*d+0] > dists[d]) { min_max_distances[2*d+0] = dists[d]; min_max_indices[2*d+0] = *it; }
        if (min_max_distances[2*d+1] < dists[d]) { min_max_distances[2*d+1] = dists[d]; min_max_indices[2*d+1] = *it; }
      }
    }
  }

  void
    SACModelOrientation::getSamples (int &iterations, std::vector<int> &samples)
  {
    /// @NOTE: we get an index of the indices_ vector because normals_ has only those elements!

    // Get a random number between 0 and indices_.size ()
    samples.resize (1);

    // I don't care, but: http://www.thinkage.ca/english/gcos/expl/c/lib/rand.html
    while (indices_.size () <= (unsigned)(samples[0] = rand () / (RAND_MAX/indices_.size ())));
    //unsigned idx;// = (int)(rand () * indices_.size () / (RAND_MAX + 1.0));
    //while (indices_.size () <= (idx = rand () / (RAND_MAX/indices_.size ())));
    // Get the index
    //samples[0] = indices_.at (idx);

    // TODO: repeat this and test coefficients until ok? increase iterations?
  }

  bool
    SACModelOrientation::testModelCoefficients (const std::vector<double> &model_coefficients)
  {
    // TODO: check radius and/or curvature?
    return true;
  }

  bool
    SACModelOrientation::computeModelCoefficients (const std::vector<int> &samples)
  {
    // Check whether the given index samples can form a valid model
    // compute the model coefficients from these samples
    // and store them internally in model_coefficients_
    model_coefficients_.resize (4);
    //model_coefficients_[0] = cloud_->channels[nx_idx_+0].values[samples[0]];
    //model_coefficients_[1] = cloud_->channels[nx_idx_+1].values[samples[0]];
    //model_coefficients_[2] = cloud_->channels[nx_idx_+2].values[samples[0]];
    model_coefficients_[0] = normals_.points[samples[0]].x;
    model_coefficients_[1] = normals_.points[samples[0]].y;
    model_coefficients_[2] = normals_.points[samples[0]].z;
    model_coefficients_[3] = samples[0];
    return true; // return value not used anyways
  }

  void
    SACModelOrientation::refitModel (const std::vector<int> &inliers, std::vector<double> &refit_coefficients)
  {
    // Compute average direction
    refit_coefficients.resize (4);

    // Coefficient[3] will hold the the source point's index
    if (inliers.size () == 0)
    {
      ROS_ERROR ("[SACModelOrientation] Can not re-fit 0 points! Returned -1 as source index (in refit_coefficients[3]).");
      refit_coefficients[3] = -1;
      return;
    }
    refit_coefficients[3] = inliers.at (0);

    //*

    // Get source and acceptable distance from it
    Eigen::Vector3f nm (normals_.points[refit_coefficients[3]].x, normals_.points[refit_coefficients[3]].y, normals_.points[refit_coefficients[3]].z);
    double eps_angle = M_PI/6; // or use a user defined threshold (eps_angle_)?
    double radius = 2 * sin (eps_angle/2);
    double sqr_radius = radius*radius;

    // get local coordinate system from the axis
    //Eigen::Vector3f v = axis_.unitOrthogonal ();
    //Eigen::Vector3f u = axis_.cross (v);

    //Eigen::MatrixXf rotated_normals(inliers.size (), 3);
    Eigen::Vector3f sum_rotated_normals = Eigen::VectorXf::Zero(3);

    // Rotate all inliers onto the first direction and sum them up
    for (unsigned it = 0; it < inliers.size (); it++)
    //for (std::vector<int>::iterator it = inliers.begin (); it != inliers.end (); it++)
    {
      /// @NOTE inliers is a list of indices of the indices_ array!
      /// @NOTE normal_ contains only the elements listed in the indices_ array!
      Eigen::Vector3f ni (normals_.points[inliers[it]].x, normals_.points[inliers[it]].y, normals_.points[inliers[it]].z);
      //std::cerr << "inlier " << it << "/" << inliers[it] << ": " << ni.transpose () << std::endl;
      //Eigen::Vector3f ni2 = ni.cwise.square ();
      for (int i=0; i<4; i++)
      {
        // Find the orientation that is pointing in the same direction as the model
        if ((nm-ni).squaredNorm () < sqr_radius)
          break;
        ni = rotateAroundAxis (ni, axis_, M_PI/2);
        //std::cerr << i << ": " << ni.transpose () << std::endl;
      }
      sum_rotated_normals += ni;
    }

    // Compute average and normalize to get the final model
    sum_rotated_normals /= inliers.size ();
    sum_rotated_normals.normalize();
    refit_coefficients[0] = sum_rotated_normals[0];
    refit_coefficients[1] = sum_rotated_normals[1];
    refit_coefficients[2] = sum_rotated_normals[2];

    //*/
  }

  void
    SACModelOrientation::selectWithinDistance (const std::vector<double> &model_coefficients, double threshold, std::vector<int> &inliers)
  {
    //std::cerr << model_coefficients[3] << ": " << model_coefficients[0] << " " << model_coefficients[1] << " " << model_coefficients[2] << " ";

    // accept only if model direction is perpendicular to axis_
    double cosine = axis_[0]*model_coefficients[0] + axis_[1]*model_coefficients[1] + axis_[2]*model_coefficients[2];
    if (cosine > 1) cosine = 1;
    if (cosine < -1) cosine = -1;
    //std::cerr << "angle difference = " << acos (cosine);
    if (fabs (acos (cosine) - M_PI/2) > threshold)
    {
      // this will make sure the model is not accepted
      inliers.resize (0);
      //std::cerr << " DISMISSED" << std::endl;
      return;
      // TODO: this will increase the iteration count, thus reduces robustness if many points are on the top! do it in getSamples ()!
    }
    //std::cerr << " ACCEPTED" << std::endl;


    // Distance in normal space - TODO: make setter for eps_angle_ and set these there
    double radius = 2 * sin (threshold/2);
    //double sqr_radius = radius*radius;

    // TODO: radius search should return number of points found

    // List of points matching the model directly (parallel normal to the sample)
    kdtree_->radiusSearch (model_coefficients[3], radius, front_indices_, points_sqr_distances_, normals_.points.size ());

    // TODO: pre-check?

    // List of points matching the model's back side (opposing normal to the sample)
    geometry_msgs::Point32 tmp;
    tmp.x = -normals_.points[model_coefficients[3]].x;
    tmp.y = -normals_.points[model_coefficients[3]].y;
    tmp.z = -normals_.points[model_coefficients[3]].z;
    kdtree_->radiusSearch (tmp, radius, back_indices_, points_sqr_distances_, normals_.points.size ());

    // Number of points matching the model's left side (perpendicular normal to the sample)
    geometry_msgs::Point32 zXn; // cross product of axis and original normal (==-tmp)
    zXn.x = axis_[1]*(-tmp.z) - axis_[2]*(-tmp.y);
    zXn.y = axis_[2]*(-tmp.x) - axis_[0]*(-tmp.z);
    zXn.z = axis_[0]*(-tmp.y) - axis_[1]*(-tmp.x);
    kdtree_->radiusSearch (zXn, radius, left_indices_, points_sqr_distances_, normals_.points.size ());

    // Number of points matching the model's right side (perpendicular normal to the sample)
    tmp.x = -zXn.x;
    tmp.y = -zXn.y;
    tmp.z = -zXn.z;
    kdtree_->radiusSearch (tmp, radius, right_indices_, points_sqr_distances_, normals_.points.size ());

    // Concatenate results
    inliers = front_indices_;
    inliers.insert (inliers.end (), back_indices_.begin (), back_indices_.end ());
    inliers.insert (inliers.end (), left_indices_.begin (), left_indices_.end ());
    inliers.insert (inliers.end (), right_indices_.begin (), right_indices_.end ());

    /// @NOTE: inliers are actually indexes in the indices_ array!
  }
}


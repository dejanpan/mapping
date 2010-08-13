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

#include <pcl_cloud_algos/box_fit2_algo.h>

// Sample Consensus
//#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <pcl/sample_consensus/ransac.h>
// For computeCentroid
//#include <point_cloud_mapping/geometry/nearest.h>

using namespace std;
using namespace pcl_cloud_algos;
using namespace pcl;
//using namespace sample_consensus;

void RobustBoxEstimation::getMinAndMax(Eigen::VectorXf *model_coefficients, boost::shared_ptr<SACModelOrientation<Normal> > model, std::vector<int> &min_max_indices, std::vector<float> &min_max_distances)
{

  boost::shared_ptr<vector<int> > inliers = model->getIndices();
  boost::shared_ptr<vector<int> > indices = model->getIndices();

  // Initialize result vectors
  min_max_indices.resize (6);
  min_max_distances.resize (6);
  min_max_distances[0] = min_max_distances[2] = min_max_distances[4] = +DBL_MAX;
  min_max_distances[1] = min_max_distances[3] = min_max_distances[5] = -DBL_MAX;

  // The 3 coordinate axes are nm, nc and axis_
  Eigen::Vector3f nm = *model_coefficients;
  //Eigen::Vector3f nm = Eigen::Vector3d::Map(&(*model_coefficients)[0]).cast<float> ();
  Eigen::Vector3f nc = model->axis_.cross (nm);

  // Find minimum and maximum distances from origin along the three axes
  for (std::vector<int>::iterator it = inliers->begin (); it != inliers->end (); it++)
  //for (unsigned i = 0; i < inliers.size (); i++)
  {
    // @NOTE inliers is a list of indices of the indices_ array!
    Eigen::Vector3f point (cloud_->points[(*indices)[*it]].x, cloud_->points[(*indices)[*it]].y, cloud_->points[(*indices)[*it]].z);
    //Eigen::Vector3f point (cloud_->points[indices_[*it]].x - center.x, cloud_->points[indices_[*it]].y - center.y, cloud_->points[indices_[*it]].z - center.z);
    //Eigen::Vector3f point (cloud_->points[indices_[inliers[i]]].x, cloud_->points[indices_[inliers[i]]].y, cloud_->points[indices_[inliers[i]]].z);
    double dists[3];
    dists[0] = nm.dot(point);
    dists[1] = nc.dot(point);
    dists[2] = model->axis_.dot(point);
    for (int d=0; d<3; d++)
    {
      if (min_max_distances[2*d+0] > dists[d]) { min_max_distances[2*d+0] = dists[d]; min_max_indices[2*d+0] = *it; }
      if (min_max_distances[2*d+1] < dists[d]) { min_max_distances[2*d+1] = dists[d]; min_max_indices[2*d+1] = *it; }
    }
  }

}
std::vector<std::string> RobustBoxEstimation::requires ()
{
  std::vector<std::string> requires;
  // requires 3D coordinates
  requires.push_back("x");
  requires.push_back("y");
  requires.push_back("z");
  // requires normals
  requires.push_back("nx");
  requires.push_back("ny");
  requires.push_back("nz");
  return requires;
}

void RobustBoxEstimation::pre ()
{
  BoxEstimation::pre ();
  nh_.param("eps_angle", eps_angle_, eps_angle_);
  nh_.param("success_probability", success_probability_, success_probability_);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * actual model fitting happens here
 */
bool RobustBoxEstimation::find_model(boost::shared_ptr<const pcl::PointCloud <pcl::PointXYZINormalScanLine> > cloud, std::vector<double> &coeff)
{
  if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Looking for box in a cluster of %u points", (unsigned)cloud->points.size ());

  // Compute center point
  //cloud_geometry::nearest::computeCentroid (*cloud, box_centroid_);
  PointCloud<Normal> nrmls ;
  nrmls.header = cloud->header;
  nrmls.points.resize(cloud->points.size());
  for(size_t i = 0 ; i < cloud->points.size(); i++)
  {
    nrmls.points[i].normal[0] = cloud->points[i].normal[0];
    nrmls.points[i].normal[1] = cloud->points[i].normal[1];
    nrmls.points[i].normal[2] = cloud->points[i].normal[2];
  }

  // Create model
  SACModelOrientation<Normal>::Ptr model = boost::make_shared<SACModelOrientation<Normal> >(boost::make_shared<pcl::PointCloud<pcl::Normal> > (nrmls));
  // SACModelOrientation<Normal> model(nrmls);

  model->axis_[0] = 0 ;
  model->axis_[1] = 0 ;
  model->axis_[2] = 1 ;

  //model->setDataSet ((sensor_msgs::PointCloud*)(cloud.get())); // TODO: this is nasty :)
  if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Axis is (%g,%g,%g) and maximum angular difference %g",
      model->axis_[0], model->axis_[1], model->axis_[2], eps_angle_);

  // Check probability of success and decide on method
  Eigen::VectorXf refined;
  vector<int> inliers;
  /// @NOTE: inliers are actually indexes in the indices_ array, but that is not set (by default it has all the points in the correct order)
  if (success_probability_ > 0 && success_probability_ < 1)
  {
    if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Using RANSAC with stop probability of %g and model refinement", success_probability_);

    // Fit model using RANSAC
    RandomSampleConsensus<Normal> *sac = new RandomSampleConsensus<Normal> (model, eps_angle_);
    sac->setProbability (success_probability_);
    if (!sac->computeModel ())
    {
      if (verbosity_level_ > -2) ROS_ERROR ("[RobustBoxEstimation] No model found using the angular threshold of %g!", eps_angle_);
      return false;
    }

    // Get inliers and refine result
    sac->getInliers(inliers);
    if (verbosity_level_ > 1) cerr << "number of inliers: " << inliers.size () << endl;
    /// @NOTE best_model_ contains actually the samples used to find the best model!
    //model->computeModelCoefficients(model->getBestModel ());
    //Eigen::VectorXf original;
    //model->getModelCoefficients (original);
    //if (verbosity_level_ > 1) cerr << "original direction: " << original[0] << " " << original[1] << " " << original[2] << ", found at point nr " << original[3] << endl;
    //sac->refineCoefficients(refined);
   // if (verbosity_level_ > 1) cerr << "refitted direction: " << refined.at (0) << " " << refined.at (1) << " " << refined.at (2) << ", initiated from point nr " << refined.at (3) << endl;
   // if (refined[3] == -1)
   //   refined = original;
  }
  else
  {
    if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Using exhaustive search in %ld points", cloud->points.size ());

    // Exhaustive search for best model
    std::vector<int> best_sample;
    std::vector<int> best_inliers;
    Eigen::VectorXf model_coefficients;
    for (unsigned i = 0; i < cloud->points.size (); i++)
    {
      std::vector<int> selection (1);
      selection[0] = i;
      model->computeModelCoefficients (selection, model_coefficients);

      model->selectWithinDistance (model_coefficients, eps_angle_, inliers);
      if (best_inliers.size () < inliers.size ())
      {
        best_inliers = inliers;
        best_sample = selection;
      }
    }

    // Check if successful and save results
    if (best_inliers.size () > 0)
    {
      model->computeModelCoefficients (best_sample, refined);
      //model->getModelCoefficients (refined);
      /// @NOTE: making things transparent for the outside... not really needed
      inliers = best_inliers;
      //model->setBestModel (best_sample);
      //model->setBestInliers (best_inliers);
      // refine results: needs inliers to be set!
      // sac->refineCoefficients(refined);
    }
    else
    {
      if (verbosity_level_ > -2) ROS_ERROR ("[RobustBoxEstimation] No model found using the angular threshold of %g!", eps_angle_);
      return false;
    }
  }

  // Save fixed axis
  coeff[12+0] = model->axis_[0];
  coeff[12+1] = model->axis_[1];
  coeff[12+2] = model->axis_[2];

  // Save complementary axis (cross product)
  coeff[9+0] = model->axis_[1]*refined[2] - model->axis_[2]*refined[1];
  coeff[9+1] = model->axis_[2]*refined[0] - model->axis_[0]*refined[2];
  coeff[9+2] = model->axis_[0]*refined[1] - model->axis_[1]*refined[0];

  // Save principle axis (corrected)
  refined[0] = - (model->axis_[1]*coeff[9+2] - model->axis_[2]*coeff[9+1]);
  refined[1] = - (model->axis_[2]*coeff[9+0] - model->axis_[0]*coeff[9+2]);
  refined[2] = - (model->axis_[0]*coeff[9+1] - model->axis_[1]*coeff[9+0]);
  coeff[6+0] = refined[0];
  coeff[6+1] = refined[1];
  coeff[6+2] = refined[2];

  /*// Save complementary axis (AGIAN, JUST TO MAKE SURE)
  coeff[9+0] = model->axis_[1]*refined[2] - model->axis_[2]*refined[1];
  coeff[9+1] = model->axis_[2]*refined[0] - model->axis_[0]*refined[2];
  coeff[9+2] = model->axis_[0]*refined[1] - model->axis_[1]*refined[0];*/

  // Compute minimum and maximum along each dimension for the whole cluster
  vector<int> min_max_indices;
  vector<float> min_max_distances;
  //boost::shared_ptr<vector<int> > indices (new vector<int>);
  //indices = model->getIndices();

  //model->getMinAndMax (&refined, &inliers, min_max_indices, min_max_distances);
  //getMinAndMax (&refined, model->getIndices (), min_max_indices, min_max_distances);
  getMinAndMax (&refined, model, min_max_indices, min_max_distances);
  //vector<int> min_max_indices = model->getMinAndMaxIndices (refined);

  //cerr << min_max_distances.at (1) << " " << min_max_distances.at (0) << endl;
  //cerr << min_max_distances.at (3) << " " << min_max_distances.at (2) << endl;
  //cerr << min_max_distances.at (5) << " " << min_max_distances.at (4) << endl;

  // Save dimensions
  coeff[3+0] = min_max_distances.at (1) - min_max_distances.at (0);
  coeff[3+1] = min_max_distances.at (3) - min_max_distances.at (2);
  coeff[3+2] = min_max_distances.at (5) - min_max_distances.at (4);

  // Distance of box's geometric center relative to origin along orientation axes
  double dist[3];
  dist[0] = min_max_distances[0] + coeff[3+0] / 2;
  dist[1] = min_max_distances[2] + coeff[3+1] / 2;
  dist[2] = min_max_distances[4] + coeff[3+2] / 2;

  // Compute position of the box's geometric center in XYZ
  coeff[0] = dist[0]*coeff[6+0] + dist[1]*coeff[9+0] + dist[2]*coeff[12+0];
  coeff[1] = dist[0]*coeff[6+1] + dist[1]*coeff[9+1] + dist[2]*coeff[12+1];
  coeff[2] = dist[0]*coeff[6+2] + dist[1]*coeff[9+2] + dist[2]*coeff[12+2];
  //coeff[0] = box_centroid_.x + dist[0]*coeff[6+0] + dist[1]*coeff[9+0] + dist[2]*coeff[12+0];
  //coeff[1] = box_centroid_.y + dist[0]*coeff[6+1] + dist[1]*coeff[9+1] + dist[2]*coeff[12+1];
  //coeff[2] = box_centroid_.z + dist[0]*coeff[6+2] + dist[1]*coeff[9+2] + dist[2]*coeff[12+2];
  if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Cluster center x: %g, y: %g, z: %g", coeff[0], coeff[1], coeff[2]);

  // Print info
  if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Dimensions x: %g, y: %g, z: %g",
      coeff[3+0], coeff[3+1], coeff[3+2]);
  if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Direction vectors: \n\t%g %g %g \n\t%g %g %g \n\t%g %g %g",
      coeff[3+3], coeff[3+4], coeff[3+5],
      coeff[3+6], coeff[3+7], coeff[3+8],
      coeff[3+9], coeff[3+10],coeff[3+11]);

  return true;
}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <RobustBoxEstimation> (argc, argv);
}
#endif


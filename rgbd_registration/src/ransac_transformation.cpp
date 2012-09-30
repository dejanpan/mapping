/*
 * RansacTransformation.cpp
 *
 *  Created on: 13.07.2012
 *      Author: ross
 */

#include "rgbd_registration/ransac_transformation.h"
#include "rgbd_registration/parameter_server.h"

#include <ros/console.h>

//pcl
#include <Eigen/Geometry>
#include <pcl/common/transformation_from_correspondences.h>
#include <iostream>

RansacTransformation::RansacTransformation() {
	// TODO Auto-generated constructor stub

}

RansacTransformation::~RansacTransformation() {
	// TODO Auto-generated destructor stub
}


void RansacTransformation::computeInliersAndError(const std::vector<cv::DMatch>& matches,
                                  const Eigen::Matrix4f& transformation,
                                  //const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
                                  //const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& earlier,
                                  const std::vector<Eigen::Vector4f>& origins,
                                  const std::vector<Eigen::Vector4f>& earlier,
                                  std::vector<cv::DMatch>& inliers, //output var
                                  double& mean_error,
                                  std::vector<double>& errors,
                                  double squaredMaxInlierDistInM) { //output var


  inliers.clear();
  errors.clear();

  std::vector<std::pair<float,int> > dists;
  std::vector<cv::DMatch> inliers_temp;

  assert(matches.size() > 0);
  mean_error = 0.0;
  for (unsigned int j = 0; j < matches.size(); j++){ //compute new error and inliers

    unsigned int this_id = matches[j].queryIdx;
    unsigned int earlier_id = matches[j].trainIdx;

    Eigen::Vector4f vec = (transformation * origins[this_id]) - earlier[earlier_id];

    double error = vec.dot(vec);

    if(error > squaredMaxInlierDistInM)
      continue; //ignore outliers
    if(!(error >= 0.0)){
      ROS_DEBUG_STREAM("Transformation for error !> 0: " << transformation);
      ROS_DEBUG_STREAM(error << " " << matches.size());
    }
    error = sqrt(error);
    dists.push_back(std::pair<float,int>(error,j));
    inliers_temp.push_back(matches[j]); //include inlier

    mean_error += error;
    errors.push_back(error);
  }

  if (inliers_temp.size()<3){ //at least the samples should be inliers
    ROS_WARN_COND(inliers_temp.size() > 3, "No inliers at all in %d matches!", (int)matches.size()); // only warn if this checks for all initial matches
    mean_error = 1e9;
  } else {
    mean_error /= inliers_temp.size();

    // sort inlier ascending according to their error
    sort(dists.begin(),dists.end());

    inliers.resize(inliers_temp.size());
    for (unsigned int i=0; i<inliers_temp.size(); i++){
      inliers[i] = matches[dists[i].second];
    }
  }
  if(!(mean_error>0)) ROS_DEBUG_STREAM("Transformation for mean error !> 0: " << transformation);
  if(!(mean_error>0)) ROS_DEBUG_STREAM(mean_error << " " << inliers_temp.size());

}

template<class InputIterator>
Eigen::Matrix4f RansacTransformation::getTransformFromMatches(
                const std::vector<Eigen::Vector4f> & source_feature_locations_3d,
								const std::vector<Eigen::Vector4f> & target_feature_locations_3d,
                InputIterator iter_begin,
                InputIterator iter_end,
                bool& valid,
                float max_dist_m)
{
  pcl::TransformationFromCorrespondences tfc;
  valid = true;
  std::vector<Eigen::Vector3f> t, f;

  for ( ;iter_begin!=iter_end; iter_begin++) {
    int this_id    = iter_begin->queryIdx;
    int earlier_id = iter_begin->trainIdx;

    Eigen::Vector3f from(source_feature_locations_3d[this_id][0],
						source_feature_locations_3d[this_id][1],
						source_feature_locations_3d[this_id][2]);
    Eigen::Vector3f  to (target_feature_locations_3d[earlier_id][0],
						target_feature_locations_3d[earlier_id][1],
						target_feature_locations_3d[earlier_id][2]);
    if (max_dist_m > 0) {  //storing is only necessary, if max_dist is given
      f.push_back(from);
      t.push_back(to);
    }
    tfc.add(from, to, 1.0/to(2));//the further, the less weight b/c of accuracy decay
  }


  // find smallest distance between a point and its neighbour in the same cloud
  // je groesser das dreieck aufgespannt ist, desto weniger fallen kleine positionsfehler der einzelnen
  // Punkte ist Gewicht!

  if (max_dist_m > 0)
  {
    //float min_neighbour_dist = 1e6;
    Eigen::Matrix4f foo;

    valid = true;
    for (uint i=0; i<f.size(); i++)
    {
      float d_f = (f.at((i+1)%f.size())-f.at(i)).norm();
      float d_t = (t.at((i+1)%t.size())-t.at(i)).norm();

      if ( abs(d_f-d_t) > max_dist_m ) {
        valid = false;
        return Eigen::Matrix4f();
      }
    }
    //here one could signal that some samples are very close, but as the transformation is validated elsewhere we don't
    //if (min_neighbour_dist < 0.5) { ROS_INFO...}
  }
  // get relative movement from samples
  return tfc.getTransformation().matrix();
}


///Find transformation with largest support, RANSAC style.
///Return false if no transformation can be found
bool RansacTransformation::getRelativeTransformationTo(const std::vector<Eigen::Vector4f> & source_feature_locations_3d,
													   const std::vector<Eigen::Vector4f> & target_feature_locations_3d,
                                       std::vector<cv::DMatch>* initial_matches,
                                       Eigen::Matrix4f& resulting_transformation,
                                       float& rmse,
                                       std::vector<cv::DMatch>& matches,
                                       uint min_matches)
{

  assert(initial_matches != NULL);
  matches.clear();

  if(initial_matches->size() <= min_matches) {
	  ROS_INFO("[RansacTransformation] Only %d feature matches between source and target (minimal: %i)",(int)initial_matches->size() , min_matches);
    return false;
  }
  else
  {
	  ROS_INFO("[RansacTransformation] %d feature matches between source and target (minimal: %i)",(int)initial_matches->size() , min_matches);
  }

  //unsigned int min_inlier_threshold = int(initial_matches->size()*0.2);
  unsigned int min_inlier_threshold = (unsigned int) min_matches;
  std::vector<cv::DMatch> inlier; //holds those feature correspondences that support the transformation
  double inlier_error; //all squared errors
  srand((long)std::clock());

  // a point is an inlier if it's no more than max_dist_m m from its partner apart
  const float max_dist_m = ParameterServer::instance()->get<double>("max_dist_for_inliers");
  const int ransac_iterations = ParameterServer::instance()->get<int>("ransac_iterations");
  std::vector<double> dummy;

  // best values of all iterations (including invalids)
  double best_error = 1e6, best_error_invalid = 1e6;
  unsigned int best_inlier_invalid = 0, best_inlier_cnt = 0, valid_iterations = 0;

  Eigen::Matrix4f transformation;

  const unsigned int sample_size = 3;// chose this many randomly from the correspondences:
  for (int n_iter = 0; n_iter < ransac_iterations; n_iter++) {
    //generate a map of samples. Using a map solves the problem of drawing a sample more than once
    std::set<cv::DMatch> sample_matches;
    std::vector<cv::DMatch> sample_matches_vector;
    while(sample_matches.size() < sample_size){
      int id = rand() % initial_matches->size();
      sample_matches.insert(initial_matches->at(id));
      sample_matches_vector.push_back(initial_matches->at(id));
    }

    bool valid; // valid is false iff the sampled points clearly aren't inliers themself
    transformation = getTransformFromMatches(source_feature_locations_3d, target_feature_locations_3d,
        sample_matches.begin(), sample_matches.end(),valid,max_dist_m);
    if (!valid) continue; // valid is false iff the sampled points aren't inliers themself
    if(transformation!=transformation) continue; //Contains NaN

    //test whether samples are inliers (more strict than before)
    computeInliersAndError(sample_matches_vector, transformation, source_feature_locations_3d,
                           target_feature_locations_3d, inlier, inlier_error,  /*output*/
                           dummy, max_dist_m*max_dist_m);
    ROS_DEBUG("Transformation from and for %u samples results in an error of %f and %i inliers.", sample_size, inlier_error, (int)inlier.size());
    if(inlier_error > 1000) continue; //most possibly a false match in the samples
    computeInliersAndError(*initial_matches, transformation, source_feature_locations_3d,
                           target_feature_locations_3d, inlier, inlier_error,  /*output*/
                           dummy, max_dist_m*max_dist_m);
    ROS_DEBUG("Transformation from %u samples results in an error of %f and %i inliers for all matches (%i).", sample_size, inlier_error, (int)inlier.size(), (int)initial_matches->size());

    // check also invalid iterations
    if (inlier.size() > best_inlier_invalid) {
      best_inlier_invalid = inlier.size();
      best_error_invalid = inlier_error;
    }
    ROS_DEBUG("iteration %d  cnt: %lu, best: %u,  error: %.2f",n_iter, inlier.size(), best_inlier_cnt, inlier_error*100);

    if(inlier.size() < min_inlier_threshold || inlier_error > max_dist_m){
      ROS_DEBUG("Skipped iteration: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", (int)inlier.size(), (int) min_inlier_threshold,  inlier_error*100, max_dist_m*100);
      continue;
    }
    // ROS_INFO("[RansacTransformation] Refining iteration from %i samples: all matches: %i, inliers: %i, inlier_error: %f", (int)sample_size, (int)initial_matches->size(), (int)inlier.size(), inlier_error);
    valid_iterations++;
    //if (inlier_error > 0) ROS_ERROR("size: %i", (int)dummy.size());
    assert(inlier_error>=0);

    //Performance hacks:
    ///Iterations with more than half of the initial_matches inlying, count twice
    if (inlier.size() > initial_matches->size()*0.5) n_iter+=10;
    ///Iterations with more than 80% of the initial_matches inlying, count threefold
    if (inlier.size() > initial_matches->size()*0.8) n_iter+=20;



    if (inlier_error < best_error) { //copy this to the result
      resulting_transformation = transformation;
      matches = inlier;
      assert(matches.size()>= min_inlier_threshold);
      best_inlier_cnt = inlier.size();
      //assert(matches.size()>= ((float)initial_matches->size())*min_inlier_ratio);
      rmse = inlier_error;
      best_error = inlier_error;
      // ROS_INFO("[RansacTransformation]   new best iteration %d  cnt: %d, best_inlier: %d,  error: %.4f, bestError: %.4f",n_iter, inlier.size(), best_inlier_cnt, inlier_error, best_error);

    }else
    {
      // ROS_INFO("[RansacTransformation] NO new best iteration %d  cnt: %d, best_inlier: %d,  error: %.4f, bestError: %.4f",n_iter, inlier.size(), best_inlier_cnt, inlier_error, best_error);
    }

    //int max_ndx = min((int) min_inlier_threshold,30); //? What is this 30?
    double new_inlier_error;

    transformation = getTransformFromMatches(source_feature_locations_3d, target_feature_locations_3d,
        matches.begin(), matches.end(), valid, max_dist_m); // compute new trafo from all inliers:
    if(transformation!=transformation) continue; //Contains NaN
    computeInliersAndError(*initial_matches, transformation,
                           source_feature_locations_3d, target_feature_locations_3d,
                           inlier, new_inlier_error, dummy, max_dist_m*max_dist_m);
    ROS_DEBUG("Refined Transformation from all matches (%i) results in an error of %f and %i inliers for all matches.", (int)initial_matches->size(), inlier_error, (int)inlier.size());
    // ROS_INFO("[RansacTransformation] asd recomputed: inliersize: %i, inlier error: %f", (int) inlier.size(),100*new_inlier_error);


    // check also invalid iterations
    if (inlier.size() > best_inlier_invalid)
    {
      best_inlier_invalid = inlier.size();
      best_error_invalid = inlier_error;
    }

    if(inlier.size() < min_inlier_threshold || new_inlier_error > max_dist_m){
      //inlier.size() < ((float)initial_matches->size())*min_inlier_ratio ||
      // ROS_INFO("[RansacTransformation] Skipped iteration: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", (int)inlier.size(), (int) min_inlier_threshold,  inlier_error*100, max_dist_m*100);
      continue;
    }
    // ROS_INFO("[RansacTransformation] Refined iteration from %i samples: all matches %i, inliers: %i, new_inlier_error: %f", (int)sample_size, (int)initial_matches->size(), (int)inlier.size(), new_inlier_error);

    assert(new_inlier_error>=0);

    if (new_inlier_error < best_error)
    {
      resulting_transformation = transformation;
      matches = inlier;
      assert(matches.size()>= min_inlier_threshold);
      //assert(matches.size()>= ((float)initial_matches->size())*min_inlier_ratio);
      rmse = new_inlier_error;
      best_error = new_inlier_error;
      // ROS_INFO("[RansacTransformation]   improved: new best iteration %d  cnt: %d, best_inlier: %d,  error: %.2f, bestError: %.2f",n_iter, inlier.size(), best_inlier_cnt, inlier_error*100, best_error*100);
    }else
    {
      // ROS_INFO("[RansacTransformation] improved: NO new best iteration %d  cnt: %d, best_inlier: %d,  error: %.2f, bestError: %.2f",n_iter, inlier.size(), best_inlier_cnt, inlier_error*100, best_error*100);
    }
  } //iterations
  ROS_INFO("[RansacTransformation] %i good iterations (from %i), inlier pct %i, inlier cnt: %i, error: %.2f cm",valid_iterations, ransac_iterations, (int) (matches.size()*1.0/initial_matches->size()*100),(int) matches.size(),rmse*100);
  ROS_DEBUG("best overall: inlier: %i, error: %.2f",best_inlier_invalid, best_error_invalid*100);

  return matches.size() >= min_inlier_threshold;
}

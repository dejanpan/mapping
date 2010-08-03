/*
 * Based on original code written by Radu Rusu
 * Modified by Hozefa Indorewala
 */

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Provide a pointer to the input model (e.g., the point cloud that we want to align the input source to)
  * \param cloud the input point cloud model
  */
template <typename PointSource, typename PointTarget> inline void
  pcl::RegistrationCorrespondencesCheck<PointSource, PointTarget>::setInputTarget (const PointCloudTargetConstPtr &cloud)
{
  if (cloud->points.empty ())
  {
    ROS_ERROR ("[pcl::%s::setInputTarget] Invalid or empty point cloud dataset given!", getClassName ().c_str ());
    return;
  }
  target_ = cloud;
  tree_->setInputCloud (target_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Obtain the fitness score (e.g., sum of squared distances from the source to the target).
  * \param max_range maximum allowable distance between a point and its correspondent neighbor in the target (default: <double>max)
  */
template <typename PointSource, typename PointTarget> inline double
  pcl::RegistrationCorrespondencesCheck<PointSource, PointTarget>::getFitnessScore (double max_range)
{
  double fitness_score = 0.0;

  // Transform the input dataset using the final transformation
  PointCloudSource input_transformed;
  transformPointCloud (*input_, input_transformed, final_transformation_);

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // For each point in the source dataset
  int nr = 0;
  for (size_t i = 0; i < input_transformed.points.size (); ++i)
  {
    Eigen::Vector4f p1 = Eigen::Vector4f (input_transformed.points[i].x,
                                          input_transformed.points[i].y,
                                          input_transformed.points[i].z, 0);
    // Find its nearest neighbor in the target
    tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);

    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] > max_range)
      continue;

    Eigen::Vector4f p2 = Eigen::Vector4f (target_->points[nn_indices[0]].x,
                                          target_->points[nn_indices[0]].y,
                                          target_->points[nn_indices[0]].z, 0);
    // Calculate the fitness score
    fitness_score += fabs ((p1-p2).squaredNorm ());
    nr++;
  }

  if (nr > 0)
    return (fitness_score / nr);
  else
    return (std::numeric_limits<double>::max ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Call the registration algorithm which estimates the transformation and returns the transformed source (input) as \a output.
  * \param output the resultant input transfomed point cloud dataset
  */
template <typename PointSource, typename PointTarget> inline void
  pcl::RegistrationCorrespondencesCheck<PointSource, PointTarget>::align (PointCloudSource &output)
{
  if (!initCompute ()) return;

  if (!target_)
  {
    ROS_WARN ("[pcl::%s::compute] No input target dataset was given!", getClassName ().c_str ());
    return;
  }

  // Resize the output dataset
  if (output.points.size () != indices_->size ())
    output.points.resize (indices_->size ());
  // Copy the header
  output.header   = input_->header;
  // Check if the output is dense or not
  if (indices_->size () != input_->points.size ())
  {
    output.width    = indices_->size ();
    output.height   = 1;
    output.is_dense = false;
  }
  else
  {
    output.width    = input_->width;
    output.height   = input_->height;
    output.is_dense = input_->is_dense;
  }

  // Copy the point data to output
  for (size_t i = 0; i < indices_->size (); ++i)
    output.points[i] = input_->points[(*indices_)[i]];

  // Perform the actual transformation computation
  converged_ = false;
  final_transformation_ = transformation_ = previous_transformation_ = Eigen::Matrix4f::Identity ();
  computeTransformation (output);

  deinitCompute ();
}


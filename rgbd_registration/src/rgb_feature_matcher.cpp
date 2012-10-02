/*
 * rgb_feature_matcher.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: kidson
 */

#include "rgbd_registration/rgb_feature_matcher.h"
#include "rgbd_registration/rgb_feature_detection.h"
#include "rgbd_registration/ransac_transformation.h"
#include "rgbd_registration/parameter_server.h"

#include "opencv2/highgui/highgui.hpp"

// converts rgb point clouds to images

RGBFeatureMatcher::RGBFeatureMatcher (PointCloudPtr source_cloud_ptr,
    PointCloudPtr target_cloud_ptr) :
  source_cloud_ptr_ (source_cloud_ptr), target_cloud_ptr_ (target_cloud_ptr)
{
  source_image_ = this->restoreCVMatFromPointCloud (source_cloud_ptr_);
  target_image_ = this->restoreCVMatFromPointCloud (target_cloud_ptr_);
}

// if the point cloud and images are separated

RGBFeatureMatcher::RGBFeatureMatcher (PointCloudPtr source_cloud_ptr,
    PointCloudPtr target_cloud_ptr, const cv::Mat& source_image, const cv::Mat& target_image) :
  source_cloud_ptr_ (source_cloud_ptr), target_cloud_ptr_ (target_cloud_ptr), source_image_ (
      source_image), target_image_ (target_image)
{
}

RGBFeatureMatcher::RGBFeatureMatcher ()
{
  // TODO Auto-generated destructor stub
}

RGBFeatureMatcher::~RGBFeatureMatcher ()
{
  // TODO Auto-generated destructor stub
}

void RGBFeatureMatcher::setSourceCloud (const PointCloudPtr source_cloud)
{
  source_cloud_ptr_ = source_cloud;
}

PointCloudConstPtr RGBFeatureMatcher::getSourceCloud ()
{
  return source_cloud_ptr_;
}

void RGBFeatureMatcher::setTargetCloud (const PointCloudPtr target_cloud)
{
  target_cloud_ptr_ = target_cloud;
}

PointCloudConstPtr RGBFeatureMatcher::getTargetCloud ()
{
  return target_cloud_ptr_;
}

void RGBFeatureMatcher::setSourceImage (const cv::Mat& source_image)
{
  source_image_ = source_image;
}
cv::Mat RGBFeatureMatcher::getSourceImage ()
{
  return source_image_;
}
void RGBFeatureMatcher::setTargetImage (const cv::Mat& target_image)
{
  target_image_ = target_image;
}
cv::Mat RGBFeatureMatcher::getTargetImage ()
{
  return target_image_;
}

cv::Mat RGBFeatureMatcher::restoreCVMatFromPointCloud (PointCloudConstPtr cloud_in)
{
  cv::Mat restored_image = cv::Mat (cloud_in->height, cloud_in->width, CV_8UC3);
  for (uint rows = 0; rows < cloud_in->height; rows++)
  {
    for (uint cols = 0; cols < cloud_in->width; ++cols)
    {
      //      restored_image.at<uint8_t>(rows, cols) = cloud_in->at(cols, rows).r;
      restored_image.at<cv::Vec3b> (rows, cols)[0] = cloud_in->at (cols, rows).b;
      restored_image.at<cv::Vec3b> (rows, cols)[1] = cloud_in->at (cols, rows).g;
      restored_image.at<cv::Vec3b> (rows, cols)[2] = cloud_in->at (cols, rows).r;
    }
  }
  return restored_image;
}

bool RGBFeatureMatcher::getMatches (std::vector<Eigen::Vector4f>& source_inlier_3d_locations,
    std::vector<Eigen::Vector4f>& target_inlier_3d_locations, Eigen::Matrix4f& ransac_trafo)
{
  if (source_image_.empty () || target_image_.empty ())
  {
    ROS_ERROR("Image not set for RGBFeatureMatcher");
    return 0;
  }
  // Extract RGB features and project into 3d
  RGBFeatureDetection RGB_feature_detector;
  std::vector<cv::KeyPoint> source_keypoints, target_keypoints;
  cv::Mat source_descriptors, target_descriptors;
  std::vector<Eigen::Vector4f> source_feature_3d_locations, target_feature_3d_locations;

  RGB_feature_detector.detectFeatures (source_image_, source_keypoints);
  RGB_feature_detector.detectFeatures (target_image_, target_keypoints);

  RGB_feature_detector.projectFeaturesTo3D (source_keypoints, source_feature_3d_locations,
      source_cloud_ptr_);
  ROS_INFO_STREAM("[RGBFeatureMatcher] Found " << source_keypoints.size() << " valid keypoints in source frame");
  RGB_feature_detector.projectFeaturesTo3D (target_keypoints, target_feature_3d_locations,
      target_cloud_ptr_);
  ROS_INFO_STREAM("[RGBFeatureMatcher] Found " << target_keypoints.size() << " valid keypoints in target frame");

  RGB_feature_detector.extractFeatures (source_image_, source_keypoints, source_descriptors);
  RGB_feature_detector.extractFeatures (target_image_, target_keypoints, target_descriptors);

  // Match features using opencv (doesn't consider depth info)
  std::vector<cv::DMatch> matches, good_matches;
  this->findMatches (source_descriptors, target_descriptors, matches);

  // Run Ransac to remove outliers and obtain a transformation between clouds
  RansacTransformation ransac_transformer;
  float rmse = 0.0;
  if (!ransac_transformer.getRelativeTransformationTo (source_feature_3d_locations,
      target_feature_3d_locations, &matches, ransac_trafo, rmse, good_matches,
      ParameterServer::instance ()->get<int> ("minimum_inliers")))
  {
    ROS_ERROR( "Not enough feature matches between frames.  Adjust 'minimum inliers parameter'");
    return false;
  }

  // Copy just the inliers to the output vectors
  for (std::vector<cv::DMatch>::iterator itr = good_matches.begin (); itr != good_matches.end (); ++itr)
  {
    source_inlier_3d_locations.push_back (source_feature_3d_locations.at (itr->queryIdx));
    target_inlier_3d_locations.push_back (target_feature_3d_locations.at (itr->trainIdx));
  }

  if (ParameterServer::instance ()->get<bool> ("show_feature_matching"))
  {
    cv::Mat img_matches;
    cv::drawMatches (source_image_, source_keypoints, target_image_, target_keypoints, matches,
        img_matches, cv::Scalar::all (-1), cv::Scalar::all (-1), std::vector<char> (),
        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::imshow ("Matches", img_matches);
    cv::moveWindow ("Matches", -10, 0);

    cv::drawMatches (source_image_, source_keypoints, target_image_, target_keypoints,
        good_matches, img_matches, cv::Scalar::all (-1), cv::Scalar::all (-1),
        std::vector<char> (), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::imshow ("Good Matches", img_matches);
    cv::moveWindow ("Good Matches", -10, 500);
    cv::waitKey (0);
    cv::destroyAllWindows ();
  }
  return true;
}

void RGBFeatureMatcher::findMatches (const cv::Mat& source_descriptors,
    const cv::Mat& target_descriptors, std::vector<cv::DMatch>& matches)
{
  cv::DescriptorMatcher* matcher;
  if (ParameterServer::instance ()->get<std::string> ("descriptor_matcher") == "FLANN")
    matcher = new cv::FlannBasedMatcher;
  else if (ParameterServer::instance ()->get<std::string> ("descriptor_matcher") == "Bruteforce")
    matcher = new cv::BFMatcher (cv::NORM_L1, false);
  else
  {
    ROS_WARN ("descriptor_matcher parameter not correctly set, defaulting to FLANN");
    matcher = new cv::FlannBasedMatcher;
  }
  matcher->match (source_descriptors, target_descriptors, matches);
}

// crude outlier removal implementation.  RANSAC is preferred to find outliers

void RGBFeatureMatcher::OutlierRemoval (const std::vector<cv::DMatch>& matches, std::vector<
    cv::DMatch>& good_matches)
{
  // Outlier detection
  double max_dist = 0;
  double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for (uint i = 0; i < matches.size (); i++)
  {
    double dist = matches[i].distance;
    if (dist < min_dist)
      min_dist = dist;
    if (dist > max_dist)
      max_dist = dist;
  }

  printf ("-- Max dist : %f \n", max_dist);
  printf ("-- Min dist : %f \n", min_dist);

  //-- Find only "good" matches (i.e. whose distance is less than 2*min_dist )
  //-- PS.- radiusMatch can also be used here.
  for (uint i = 0; i < matches.size (); i++)
  {
    if (matches[i].distance < 4 * min_dist)
      good_matches.push_back (matches[i]);
  }
  for (uint i = 0; i < good_matches.size (); i++)
  {
    printf ("-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i,
        good_matches[i].queryIdx, good_matches[i].trainIdx);
  }
}

/*
 * rgb_feature_detection.cpp
 *
 *  Created on: Sep 25, 2012
 *      Author: Ross kidson
 */

#include "rgbd_registration/rgb_feature_detection.h"
//#include "rgbd_registration/sift_gpu_wrapper.h"
#include "rgbd_registration/parameter_server.h"

//opencv
#include "opencv2/core/core.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv.h>

#include <ros/console.h>

RGBFeatureDetection::RGBFeatureDetection ():
image_counter_(0)
{
}

RGBFeatureDetection::~RGBFeatureDetection ()
{
}

// projectFeaturesTo3D
//
// Takes a RGB feature pixel location and uses depth information to make it a 3d coordiant
// this also removes keypoints that have NaN depths

void RGBFeatureDetection::projectFeaturesTo3D (std::vector<cv::KeyPoint>& feature_locations_2d,
    std::vector<Eigen::Vector4f> & feature_locations_3d, const PointCloudConstPtr point_cloud)
{
  int index = -1;
  for (unsigned int i = 0; i < feature_locations_2d.size (); /*increment at end of loop*/)
  {
    ++index;

    cv::Point2f p2d = feature_locations_2d[i].pt;
    PointType p3d = point_cloud->at ((int) p2d.x, (int) p2d.y);

    // Check for invalid measurements
    if (isnan (p3d.x) || isnan (p3d.y) || isnan (p3d.z))
    {
      ROS_DEBUG ("Feature %d has been extracted at NaN depth. Omitting", i);
      feature_locations_2d.erase (feature_locations_2d.begin () + i);
      continue;
    }

    feature_locations_3d.push_back (Eigen::Vector4f (p3d.x, p3d.y, p3d.z, 1.0));
    //featuresUsed.push_back(index);  //save id for constructing the descriptor matrix
    i++; //Only increment if no element is removed from vector
  }
}

void RGBFeatureDetection::detectFeatures(const cv::Mat& input_image,
    std::vector<cv::KeyPoint>& keypoints)
{
  // convert to black and white
  cv::Mat image_greyscale;
  cvtColor (input_image, image_greyscale, CV_RGB2GRAY);

  //detect features
  cv::FeatureDetector* detector;
  if (ParameterServer::instance ()->get<std::string> ("feature_extractor") == "SIFT")
    detector = new cv::SiftFeatureDetector;
  else
    detector = new cv::SurfFeatureDetector (400);
  detector->detect (image_greyscale, keypoints);
}

void RGBFeatureDetection::extractFeatures(const cv::Mat& input_image,
    std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
  // convert to black and white
  cv::Mat image_greyscale;
  cvtColor (input_image, image_greyscale, CV_RGB2GRAY);

  //extract features
  cv::DescriptorExtractor* extractor;

  if (ParameterServer::instance ()->get<std::string> ("feature_descriptor") == "SIFT")
    extractor = new cv::SiftDescriptorExtractor;
  else
    extractor = new cv::SurfDescriptorExtractor;
  extractor->compute (image_greyscale, keypoints, descriptors);

  if (ParameterServer::instance ()->get<bool> ("save_features_image"))
  {
    cv::Mat output;
    cv::drawKeypoints (image_greyscale, keypoints, output);
    std::stringstream result;
    result << "sift_result" << image_counter_++ << ".jpg";
    cv::imwrite (result.str (), output);
  }
}


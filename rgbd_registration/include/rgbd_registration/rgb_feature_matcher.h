/*
 * RGB_feature_matcher.h
 *
 *  Created on: Sep 26, 2012
 *      Author: kidson
 */

#ifndef RGB_FEATURE_MATCHER_H_
#define RGB_FEATURE_MATCHER_H_

#include "rgbd_registration/typedefs.h"
#include <cv.h>

class RGBFeatureMatcher
{
  public:
    RGBFeatureMatcher ();

    RGBFeatureMatcher (PointCloudPtr source_cloud_ptr, PointCloudPtr target_cloud_ptr);

    RGBFeatureMatcher (PointCloudPtr source_cloud_ptr, PointCloudPtr target_cloud_ptr,
        const cv::Mat& source_image, const cv::Mat& target_image);

    virtual ~RGBFeatureMatcher ();

    void setSourceCloud (const PointCloudPtr source_cloud);
    PointCloudConstPtr getSourceCloud ();
    void setTargetCloud (const PointCloudPtr target_cloud);
    PointCloudConstPtr getTargetCloud ();
    void setSourceImage (const cv::Mat& source_image);
    cv::Mat getSourceImage ();
    void setTargetImage (const cv::Mat& target_image);
    cv::Mat getTargetImage ();

    cv::Mat restoreCVMatFromPointCloud (PointCloudConstPtr cloud_in);

    void findMatches (const cv::Mat& source_descriptors, const cv::Mat& target_descriptors,
        std::vector<cv::DMatch>& matches);

    void OutlierRemoval (const std::vector<cv::DMatch>& matches,
        std::vector<cv::DMatch>& good_matches);

    bool
        getMatches (
            std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& source_inlier_3d_locations,
            std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& target_inlier_3d_locations,
            Eigen::Matrix4f& ransac_trafo);

  private:
    PointCloudConstPtr source_cloud_ptr_, target_cloud_ptr_;
    cv::Mat source_image_, target_image_;
};

#endif /* RGB_FEATURE_MATCHER_H_ */

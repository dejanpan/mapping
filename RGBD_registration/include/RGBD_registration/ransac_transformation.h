/*
 * RansacTransformation.h
 *
 *  Created on: 13.07.2012
 *      Author: ross
 */

#ifndef RANSACTRANSFORMATION_H_
#define RANSACTRANSFORMATION_H_

//opencv
#include <opencv/cv.h>

#include <Eigen/Core>

class RansacTransformation
{
  public:
    RansacTransformation ();
    virtual ~RansacTransformation ();

    template<class InputIterator>
    Eigen::Matrix4f getTransformFromMatches (
        const std::vector<Eigen::Vector4f> & source_feature_locations_3d,
        const std::vector<Eigen::Vector4f> & target_feature_locations_3d,
        InputIterator iter_begin, InputIterator iter_end, bool& valid, float max_dist_m);

    void computeInliersAndError (const std::vector<cv::DMatch>& matches,
        const Eigen::Matrix4f& transformation,
        //const std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
        //const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& earlier,
        const std::vector<Eigen::Vector4f >& origins,
        const std::vector<Eigen::Vector4f >& earlier,
        std::vector<cv::DMatch>& inliers, //output var
        double& mean_error, std::vector<double>& errors, double squaredMaxInlierDistInM);

    bool
        getRelativeTransformationTo (
            const std::vector<Eigen::Vector4f> & source_feature_locations_3d,
            const std::vector<Eigen::Vector4f> & target_feature_locations_3d,
            std::vector<cv::DMatch>* initial_matches,
            Eigen::Matrix4f& resulting_transformation,
            float& rmse,
            std::vector<cv::DMatch>& matches, uint min_matches);
};

#endif /* RANSACTRANSFORMATION_H_ */

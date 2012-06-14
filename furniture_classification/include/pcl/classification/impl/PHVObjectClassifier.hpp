/*
 * PHVObjectClassifier.hpp
 *
 *  Created on: Jun 9, 2012
 *      Author: vsu
 */

#ifndef PHVOBJECTCLASSIFIER_HPP_
#define PHVOBJECTCLASSIFIER_HPP_

#include <pcl/classification/PHVObjectClassifier.h>
#include <opencv2/core/core.hpp>

template<class FeatureT>
  cv::Mat transform_to_mat(const std::vector<FeatureT> & features)
  {
    int featureLength = sizeof(features[0].histogram) / sizeof(float);

    cv::Mat res(features.size(), featureLength, CV_32F);
    for (size_t i = 0; i < features.size(); i++)
    {
      for (int j = 0; j < featureLength; j++)
      {
        res.at<float> (i, j) = features[i].histogram[j];
      }
    }

    return res;
  }

template<class FeatureT>
  void transform_to_features(const cv::Mat & mat, std::vector<FeatureT> & features)
  {
    features.clear();
    for (int i = 0; i < mat.rows; i++)
    {
      FeatureT f;
      for (int j = 0; j < mat.cols; j++)
      {
        f.histogram[j] = mat.at<float> (i, j);
      }
      features.push_back(f);
    };
  }

template<class PointT, class PointNormalT, class FeatureT>
  void pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::clusterFeatures(vector<FeatureT> & cluster_centers,
                                                                                 vector<int> & cluster_labels)
  {
    int featureLength = sizeof(features_[0].histogram) / sizeof(float);

    cv::Mat feature_vectors = transform_to_mat(features_);
    cv::Mat centers(num_clusters_, featureLength, feature_vectors.type()), labels;

    cv::kmeans(feature_vectors, num_clusters_, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0),
               3, cv::KMEANS_PP_CENTERS, centers);

    transform_to_features(centers, cluster_centers);
    cluster_labels = labels;
  }

template<class PointT, class PointNormalT, class FeatureT>
  void pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::saveToFile()
  {

    // Delete old and create new directory sturcture for output
    boost::filesystem::path output_path(database_dir_);
    boost::filesystem::path output_models_path(database_dir_+"models/");
    if (boost::filesystem::exists(output_path))
    {
      boost::filesystem::remove_all(output_path);
    }

    boost::filesystem::create_directories(output_path);
    boost::filesystem::create_directories(output_models_path);

    YAML::Emitter out;

    out << *this;

  std::ofstream f;
  f.open((database_dir_ + "database.yaml").c_str());
  f << out.c_str();
  f.close();

}

template<class PointT, class PointNormalT, class FeatureT>
  void pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::loadFromFile()
  {

    std::ifstream fin((database_dir_ + "database.yaml").c_str());
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);

    doc >> *this;

  }

#endif /* PHVOBJECTCLASSIFIER_HPP_ */

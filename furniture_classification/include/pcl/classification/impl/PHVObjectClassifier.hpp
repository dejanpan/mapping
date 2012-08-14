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
    boost::filesystem::path output_models_path(database_dir_ + "models/");
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

template<class PointT, class PointNormalT, class FeatureT>
  void pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::computeClassifier()
  {

    BOOST_FOREACH(ModelMapValueType &v, class_name_to_partial_views_map_)
{    int view_num_counter = 0;

    for(size_t i=0; i<v.second.size(); i++)
    {
      appendFeaturesFromCloud(v.second[i], v.first, view_num_counter);
      view_num_counter++;

    }

  }

  // Transform to model centroinds in local coordinate frame of the segment
  centroids_.getMatrixXfMap() *= -1;

  normalizeFeatures(features_);

  if(debug_)
  {

    for(int i=0; i<num_clusters_; i++)
    {
      std::stringstream ss;
      ss << debug_folder_ << "Cluster" << i << "/";

      boost::filesystem::path output_path(ss.str());
      if (boost::filesystem::exists(output_path))
      {
        boost::filesystem::remove_all(output_path);
      }

      boost::filesystem::create_directories(output_path);
    }

    std::ofstream f((debug_folder_+"features.txt").c_str());

    size_t N = sizeof(features_[0].histogram)/sizeof(float);

    for(size_t i=0; i<features_.size(); i++)
    {
      for(size_t j=0; j<N; j++)
      {
        f << features_[i].histogram[j] << " ";
      }

      f << "\n";

    }

    f.close();

    std::ofstream f2((debug_folder_+"classnames.txt").c_str());

    for(size_t i=0; i<classes_.size(); i++)
    {

      f2 << classes_[i] << "\n";

    }

    f2.close();

    std::ofstream f3((debug_folder_+"centroids.txt").c_str());

    for(size_t i=0; i<centroids_.size(); i++)
    {

      f3 << centroids_.points[i].x << " " << centroids_.points[i].y << " " << centroids_.points[i].z << "\n";

    }

    f3.close();

  }

  vector<FeatureT> cluster_centers;
  vector<int> cluster_labels;

  clusterFeatures(cluster_centers, cluster_labels);

  database_.clear();

  for (size_t i = 0; i < cluster_labels.size(); i++)
  {

    FeatureT cluster_center = cluster_centers[cluster_labels[i]];
    std::string classname = classes_[i];

    database_[cluster_center][classname].points.push_back(centroids_[i]);
    database_[cluster_center][classname].width
    = database_[cluster_center][classname].points.size();
    database_[cluster_center][classname].height = 1;
    database_[cluster_center][classname].is_dense = true;

    ransac_result_threshold_[classname] = 0.5;

    if(debug_)
    {
      std::stringstream ss;
      ss << debug_folder_ << "Cluster" << cluster_labels[i] << "/Segment" << i << ".pcd";
      pcl::io::savePCDFileASCII(ss.str(), *segment_pointclouds_[i]);

    }

  }

}

template<class PointT, class PointNormalT, class FeatureT>
  void pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::computeExternalClassifier(const std::string & labels)
  {

    BOOST_FOREACH(ModelMapValueType &v, class_name_to_partial_views_map_)
{    int view_num_counter = 0;

    for(size_t i=0; i<v.second.size(); i++)
    {
      appendFeaturesFromCloud(v.second[i], v.first, view_num_counter);
      view_num_counter++;

    }

  }

  // Transform to model centroinds in local coordinate frame of the segment
  centroids_.getMatrixXfMap() *= -1;

  normalizeFeatures(features_);

  for(int i=0; i<num_clusters_; i++)
  {
    std::stringstream ss;
    ss << debug_folder_ << "Cluster" << i << "/";

    boost::filesystem::path output_path(ss.str());
    if (boost::filesystem::exists(output_path))
    {
      boost::filesystem::remove_all(output_path);
    }

    boost::filesystem::create_directories(output_path);
  }

  std::ofstream f((debug_folder_+"features.txt").c_str());

  size_t N = sizeof(features_[0].histogram)/sizeof(float);

  for(size_t i=0; i<features_.size(); i++)
  {
    for(size_t j=0; j<N; j++)
    {
      f << features_[i].histogram[j] << " ";
    }

    f << "\n";

  }

  f.close();

  std::ofstream f2((debug_folder_+"classnames.txt").c_str());

  for(size_t i=0; i<classes_.size(); i++)
  {

    f2 << classes_[i] << "\n";

  }

  f2.close();

  std::ofstream f3((debug_folder_+"centroids.txt").c_str());

  for(size_t i=0; i<centroids_.size(); i++)
  {

    f3 << centroids_.points[i].x << " " << centroids_.points[i].y << " " << centroids_.points[i].z << "\n";

  }

  f3.close();

  vector<int> cluster_labels;
  set<int> unique_cluster_labels;
  vector<FeatureT> cluster_centers();

  std::ifstream f4(labels.c_str());

  for(size_t i=0; i<features_.size(); i++)
  {
    int l;
    f4 >> l;
    cluster_labels.push_back(l);
    unique_cluster_labels.insert(l);
  }

  for(set<int>::iterator it=unique_cluster_labels.begin(); it != unique_cluster_labels.end(); it++)
  {
    int N = sizeof(FeatureT)/sizeof(float);
    FeatureT c;
    int num_clusters = 0;

    map<string, PointCloud> map;

    for(int j=0; j<N; j++)
    {
      c.histogram[j] = 0;
    }

    for(size_t i=0; i<features_.size(); i++)
    {
      if(cluster_labels[i] == *it)
      {
        for(int j=0; j<N; j++)
        {
          c.histogram[j] += features_[i].histogram[j];

        }

        map[classes_[i]].points.push_back(centroids_.points[i]);
        map[classes_[i]].width
        = map[classes_[i]].points.size();
        map[classes_[i]].height = 1;
        map[classes_[i]].is_dense = true;

        ransac_result_threshold_[classes_[i]] = 0.5;
        num_clusters ++;
      }
    }

    for(int j=0; j<N; j++)
    {
      c.histogram[j] = c.histogram[j]/num_clusters;
    }

    database_[c] = map;

  }

}

template<class PointT, class PointNormalT, class FeatureT>
  void pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::classify()
  {
    appendFeaturesFromCloud(scene_, "Scene", 0);
    normalizeFeaturesWithCurrentMinMax(features_);
    vote();

    RandomSampleConsensusSimple<PointNormalT> ransac(subsampling_resolution_ * 2);

    ransac.setScene(scene_);
    ransac.setMaxIterations(ransac_num_iter_);
    ransac.setWeight(ransac_vis_score_weight_);

    for (std::map<std::string, pcl::PointCloud<pcl::PointXYZI> >::const_iterator it = votes_.begin(); it
        != votes_.end(); it++)
    {

      std::cerr << "Checking for " << it->first << std::endl;
      pcl::io::savePCDFileASCII(debug_folder_ + it->first + "_votes.pcd", it->second);

      int grid_center_x, grid_center_y;
      Eigen::MatrixXf grid = projectVotesToGrid(it->second, grid_center_x, grid_center_y);

      if (debug_)
      {
        Eigen::MatrixXi img = (grid * 255.0 / grid.maxCoeff()).cast<int> ();

        std::stringstream filename;
        filename << debug_folder_ << it->first << "_center_" << grid_center_x << "_" << grid_center_y << ".pgm";
        std::ofstream f(filename.str().c_str());
        f << "P2\n" << grid.cols() << " " << grid.rows() << "\n255\n";
        f << img;
      }

      vector<PointNormalCloudPtr> result;
      vector<float> scores;

      BOOST_FOREACH(PointNormalCloudPtr & full_model, class_name_to_full_models_map_[it->first])
{      PointNormalT minp, maxp;
      pcl::getMinMax3D<PointNormalT>(*full_model, minp, maxp);
      float window_size = std::max((maxp.x - minp.x),(maxp.y - minp.y))/2;

      std::cerr << "Window size " << window_size << std::endl;

      ransac.setModel(full_model);

      PointCloudPtr local_maxima_ = findLocalMaximaInGrid(grid, window_size);

      if (debug_ && !local_maxima_->empty())
      pcl::io::savePCDFileASCII(debug_folder_ + it->first + "_local_maxima.pcd", *local_maxima_);

      vector<boost::shared_ptr<std::vector<int> > > voted_segments;
      voted_segments = findVotedSegments(local_maxima_, it->first, window_size);

      fitModelsWithRansac(voted_segments, it->first, ransac, result, scores);

    }

    //generateVisibilityScore(result, scores);
    result = removeIntersecting(result, scores);

    found_objects_[it->first] = result;

  }

}

template<class PointT, class PointNormalT, class FeatureT>
  void pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::eval_clustering(const std::string & classname,
                                                                                 const float search_radius, double &tp,
                                                                                 double &fn, double &fp)
  {
    appendFeaturesFromCloud(scene_, "Scene", 0);
    normalizeFeaturesWithCurrentMinMax(features_);
    vote();

    for (std::map<std::string, pcl::PointCloud<pcl::PointXYZI> >::const_iterator it = votes_.begin(); it
        != votes_.end(); it++)
    {

      //std::cerr << "Checking for " << it->first << std::endl;
      //pcl::io::savePCDFileASCII(debug_folder_ + it->first + "_votes.pcd", it->second);

      int grid_center_x, grid_center_y;
      Eigen::MatrixXf grid = projectVotesToGrid(it->second, grid_center_x, grid_center_y);

      BOOST_FOREACH(PointNormalCloudPtr & full_model, class_name_to_full_models_map_[it->first])
{      PointNormalT minp, maxp;
      pcl::getMinMax3D<PointNormalT>(*full_model, minp, maxp);
      float window_size = std::max((maxp.x - minp.x),(maxp.y - minp.y))/2;

      Eigen::ArrayXXi local_max = getLocalMaximaGrid(grid, window_size);

      int ctp, cfp, cfn;
      int search_radius_pixels = search_radius/cell_size_;

      if (it->first == classname)
      {

        Eigen::ArrayXXi true_region = local_max.block(grid_center_x - search_radius_pixels, grid_center_y - search_radius_pixels, 2 * search_radius_pixels
            + 1, 2 * search_radius_pixels + 1);

        ctp = (true_region == 1).count();
        cfn = (ctp == 0);
        cfp = (local_max == 1).count() - ctp;

      }
      else
      {
        cfp = (local_max == 1).count();
        ctp = 0;
        cfn = 0;
      }

      tp += ctp;
      fp += cfp;
      fn += cfn;

    }

  }

}

template<class PointT, class PointNormalT, class FeatureT>
  typename pcl::PointCloud<PointNormalT>::Ptr pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::estimateNormalsAndSubsample(
                                                                                                                                    typename pcl::PointCloud<
                                                                                                                                        PointT>::ConstPtr cloud_orig)
  {

    //std::vector<int> idx;
    //PointCloudPtr cloud(new PointCloud);
    //pcl::removeNaNFromPointCloud(*cloud_orig, *cloud, idx);

    PointNormalCloudPtr cloud_with_normals(new PointNormalCloud);
    PointNormalCloudPtr cloud_downsampled(new PointNormalCloud);

    //    pcl::VoxelGrid<PointT> grid;
    //    grid.setInputCloud(cloud_orig);
    //    grid.setLeafSize(subsampling_resolution_, subsampling_resolution_, subsampling_resolution_);
    //    grid.filter(*cloud_downsampled);

    PointTreePtr tree(new PointTree);

    mls_->setComputeNormals(true);

    mls_->setInputCloud(cloud_orig);
    mls_->setPolynomialFit(mls_polynomial_fit_);
    mls_->setPolynomialOrder(mls_polynomial_order_);
    mls_->setSearchMethod(tree);
    mls_->setSearchRadius(mls_search_radius_);

    this->mls_->process(*cloud_with_normals);

    pcl::VoxelGrid<PointNormalT> grid;
    grid.setInputCloud(cloud_with_normals);
    grid.setLeafSize(subsampling_resolution_, subsampling_resolution_, subsampling_resolution_);
    grid.filter(*cloud_downsampled);

    //cloud_downsampled->is_dense = false;
    //pcl::removeNaNFromPointCloud(*cloud_downsampled, *cloud_downsampled, idx);

    cloud_downsampled->sensor_origin_ = cloud_orig->sensor_origin_;
    cloud_downsampled->sensor_orientation_ = cloud_orig->sensor_orientation_;

    return cloud_downsampled;

  }

template<class PointT, class PointNormalT, class FeatureT>
  void pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::getSegmentsFromCloud(
                                                                                      PointNormalCloudPtr cloud_with_normals,
                                                                                      vector<boost::shared_ptr<vector<
                                                                                          int> > > & segment_indices,
                                                                                      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & colored_segments)
  {

    segment_indices.clear();

    PointCloudPtr cloud(new PointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::copyPointCloud(*cloud_with_normals, *cloud);
    pcl::copyPointCloud(*cloud_with_normals, *normals);

    PointTreePtr tree(new PointTree);

    vector<vector<int> > segment_indices_all;

    pcl::RegionGrowing<PointT> region_growing;
    region_growing.setCloud(cloud);
    region_growing.setNormals(normals);
    region_growing.setNeighbourSearchMethod(tree);
    region_growing.setResidualTest(true);
    region_growing.setResidualThreshold(rg_residual_threshold_);
    region_growing.setCurvatureTest(false);
    region_growing.setSmoothMode(false);
    region_growing.setSmoothnessThreshold(rg_smoothness_threshold_);
    region_growing.segmentPoints();
    segment_indices_all = region_growing.getSegments();

    BOOST_FOREACH(vector<int> & i, segment_indices_all)
{    if((int)i.size() > min_points_in_segment_)
    {
      boost::shared_ptr<vector<int> > indices(new vector<int>);
      *indices = i;
      segment_indices.push_back(indices);
    }
  }

  if(debug_)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_segments_all;
    colored_segments_all = region_growing.getColoredCloud();

    vector<int> valid_segment_indices;

    BOOST_FOREACH(boost::shared_ptr<vector<int> > & i, segment_indices)
    {
      valid_segment_indices.insert(valid_segment_indices.begin(), i->begin(), i->end());
    }

    colored_segments.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    for(size_t i=0; i<valid_segment_indices.size(); i++){
      pcl::PointXYZRGBNormal p;
      p.x = colored_segments_all->points[i].x;
      p.y = colored_segments_all->points[i].y;
      p.z = colored_segments_all->points[i].z;
      p.normal_x = normals->points[i].normal_x;
      p.normal_y = normals->points[i].normal_y;
      p.normal_z = normals->points[i].normal_z;
      p.rgb = colored_segments_all->points[i].rgb;
      colored_segments->push_back(p);
    }


  }

}

template<class PointT, class PointNormalT, class FeatureT>
  void pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::appendFeaturesFromCloud(PointNormalCloudPtr & cloud,
                                                                                         const string & class_name,
                                                                                         const int i)
  {
    if (debug_)
    {
      std::stringstream ss;
      ss << debug_folder_ << class_name << i << ".pcd";
      std::cerr << "Writing to file " << ss.str() << std::endl;
      pcl::io::savePCDFileASCII(ss.str(), *cloud);

    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colored_segments;
    vector<boost::shared_ptr<vector<int> > > segment_indices;
    getSegmentsFromCloud(cloud, segment_indices, colored_segments);

    segment_indices_ = segment_indices;

    PointNormalTreePtr tree(new PointNormalTree);



    feature_estimator_->setSearchMethod(tree);
    feature_estimator_->setKSearch(fe_k_neighbours_);
    feature_estimator_->setInputCloud(cloud);

    typename pcl::FeatureFromNormals<PointNormalT, PointNormalT, FeatureT>::Ptr f = boost::dynamic_pointer_cast<
        pcl::FeatureFromNormals<PointNormalT, PointNormalT, FeatureT> >(feature_estimator_);

    if (f)
    {
      f ->setInputNormals(cloud);
      f->setKSearch(20);
    }

    BOOST_FOREACH(const boost::shared_ptr<vector<int> > & idx, segment_indices)
{   // compute deature for segment
    pcl::PointCloud<FeatureT> feature;
    //PointNormalCloudPtr p(new PointNormalCloud(*cloud, *idx));
    //feature_estimator_->setInputCloud(p);
    feature_estimator_->setIndices(idx);
    feature_estimator_->compute(feature);

    // Compute centroid of segment
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, *idx, centroid);
    PointT centroid_point;
    centroid_point.x = centroid[0];
    centroid_point.y = centroid[1];
    centroid_point.z = centroid[2];

    features_.push_back(feature.points[0]);
    centroids_.points.push_back(centroid_point);
    classes_.push_back(class_name);

    if(debug_)
    {
      PointNormalCloudPtr segm(new PointNormalCloud(*cloud, *idx));
      segment_pointclouds_.push_back(segm);
    }

  }

  centroids_.width = centroids_.points.size();
  centroids_.height = 1;
  centroids_.is_dense = true;

  // if debug dump partial views and segmentations
  if(debug_)
  {
    std::stringstream ss;
    ss << debug_folder_ << class_name << i << "_segmentation.pcd";
    std::cerr << "Writing to file " << ss.str() << std::endl;
    pcl::io::savePCDFileASCII(ss.str(), *colored_segments);
  }
}

template<class PointT, class PointNormalT, class FeatureT>
  void pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::normalizeFeatures(std::vector<FeatureT> & features)
  {

    int N = sizeof(min_.histogram) / sizeof(float);

    // Init max and min vales with first feature
    for (int j = 0; j < N; j++)
    {
      min_.histogram[j] = features[0].histogram[j];
      max_.histogram[j] = features[0].histogram[j];
    }

    // Find max and min values
    for (size_t i = 0; i < features.size(); i++)
    {
      for (int j = 0; j < N; j++)
      {
        if (features[i].histogram[j] < min_.histogram[j])
          min_.histogram[j] = features[i].histogram[j];
        if (features[i].histogram[j] > max_.histogram[j])
          max_.histogram[j] = features[i].histogram[j];
      }
    }

    // Normalize
    for (size_t i = 0; i < features.size(); i++)
    {
      for (int j = 0; j < N; j++)
      {
        if ((max_.histogram[j] - min_.histogram[j]) != 0)
        {
          features[i].histogram[j] = (features[i].histogram[j] - min_.histogram[j]) / (max_.histogram[j]
              - min_.histogram[j]);
        }
      }
    }

  }

template<class PointT, class PointNormalT, class FeatureT>
  void pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::normalizeFeaturesWithCurrentMinMax(std::vector<
      FeatureT> & features)
  {
    int N = sizeof(min_.histogram) / sizeof(float);

    FeatureT min = min_, max = max_;

    // Find max and min values
    for (size_t i = 0; i < features.size(); i++)
    {
      for (int j = 0; j < N; j++)
      {
        if (features[i].histogram[j] < min.histogram[j])
          min.histogram[j] = features[i].histogram[j];
        if (features[i].histogram[j] > max.histogram[j])
          max.histogram[j] = features[i].histogram[j];
      }
    }

    // Normalize
    for (size_t i = 0; i < features.size(); i++)
    {
      for (int j = 0; j < N; j++)
      {
        if ((max_.histogram[j] - min_.histogram[j]) != 0)
        {
          features[i].histogram[j] = (features[i].histogram[j] - min.histogram[j]) / (max.histogram[j]
              - min.histogram[j]);
        }
      }
    }

  }

template<class PointT, class PointNormalT, class FeatureT>
  void pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::vote()
  {

    pcl::search::KdTree<FeatureT> feature_search;
    feature_search.setInputCloud(database_features_cloud_);

    for (size_t i = 0; i < features_.size(); i++)
    {
      std::vector<int> indices;
      std::vector<float> distances;
      feature_search.nearestKSearch(features_[i], num_neighbours_, indices, distances);

      if (debug_)
      {
        std::stringstream ss;
        ss << debug_folder_ << "Segment" << i << ".pcd";
        PointNormalCloud p(*scene_, *segment_indices_[i]);
        pcl::io::savePCDFileASCII(ss.str(), p);
      }

      for (size_t j = 0; j < indices.size(); j++)
      {

        FeatureT closest_cluster = database_features_cloud_->at(indices[j]);
        for (std::map<std::string, pcl::PointCloud<pcl::PointXYZ> >::const_iterator it =
            database_[closest_cluster].begin(); it != database_[closest_cluster].end(); it++)
        {

          std::string class_name = it->first;
          PointCloud model_centers = it->second;
          PointCloud model_centers_transformed;
          pcl::PointCloud<pcl::PointXYZI> model_centers_transformed_weighted;

          Eigen::Affine3f transform;
          transform.setIdentity();
          transform.translate(centroids_[i].getVector3fMap());
          pcl::transformPointCloud(model_centers, model_centers_transformed, transform);

          pcl::copyPointCloud(model_centers_transformed, model_centers_transformed_weighted);

          // TODO revise weighting function
          for (size_t k = 0; k < model_centers_transformed_weighted.size(); k++)
          {
            model_centers_transformed_weighted.points[k].intensity = exp(-(distances[j] * distances[j])) * (1.0
                / model_centers.size());
            voted_segment_idx_[class_name].push_back(i);
          }

          if (debug_)
          {
            std::stringstream ss;
            ss << debug_folder_ << "Segment" << i << "_neighbour" << j << "_" << class_name << "_votes.pcd";
            pcl::io::savePCDFileASCII(ss.str(), model_centers_transformed_weighted);
          }

          votes_[class_name] += model_centers_transformed_weighted;
        }
      }

    }

  }

template<class PointT, class PointNormalT, class FeatureT>
  Eigen::MatrixXf pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::projectVotesToGrid(const pcl::PointCloud<
      pcl::PointXYZI> & model_centers, int & grid_center_x, int & grid_center_y)
  {

    Eigen::MatrixXf grid;

    int image_x_width = (int)((max_scene_bound_.x - min_scene_bound_.x) / cell_size_);
    int image_y_width = (int)((max_scene_bound_.y - min_scene_bound_.y) / cell_size_);

    grid = Eigen::MatrixXf::Zero(image_x_width, image_y_width);

    for (size_t i = 0; i < model_centers.points.size(); i++)
    {
      int vote_x = (model_centers.points[i].x - min_scene_bound_.x) / cell_size_;
      int vote_y = (model_centers.points[i].y - min_scene_bound_.y) / cell_size_;
      if ((vote_x >= 0) && (vote_y >= 0) && (vote_x < image_x_width) && (vote_y < image_y_width)
          && (model_centers.points[i].z >= 0))
        grid(vote_x, vote_y) += model_centers.points[i].intensity;
    }

    grid_center_x = -min_scene_bound_.x / cell_size_;
    grid_center_y = -min_scene_bound_.y / cell_size_;

    return grid;
  }

template<class PointT, class PointNormalT, class FeatureT>
  typename pcl::PointCloud<PointT>::Ptr pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::findLocalMaximaInGrid(
                                                                                                                        Eigen::MatrixXf grid,
                                                                                                                        float window_size)
  {

    PointCloudPtr local_maxima(new PointCloud);

    float max, min;
    max = grid.maxCoeff();
    min = grid.minCoeff();

    float threshold = min + (max - min) * local_maxima_threshold_;
    //float threshold = local_maxima_threshold_;

    int window_size_pixels = window_size / cell_size_;

    if (window_size_pixels >= std::min(grid.cols() - 3, grid.rows() - 3))
    {
      window_size_pixels = std::min(grid.cols() - 3, grid.rows() - 3);
    }

    // Make window_size_pixels even
    if (window_size_pixels % 2 == 0)
      window_size_pixels++;

    int side = window_size_pixels / 2;

    for (int i = side; i < (grid.rows() - side); i++)
    {
      for (int j = side; j < (grid.cols() - side); j++)
      {

        float max;
        Eigen::MatrixXf window = grid.block(i - side, j - side, window_size_pixels, window_size_pixels);
        max = window.maxCoeff();

        assert(window.cols() == window_size_pixels);
        assert(window.rows() == window_size_pixels);

        // if max of the window is in its center then this point is local maxima
        if ((max == grid(i, j)) && (max > 0) && (max > threshold))
        {
          PointT point;
          point.x = i * cell_size_ + min_scene_bound_.x;
          point.y = j * cell_size_ + min_scene_bound_.y;
          point.z = 0;
          local_maxima->points.push_back(point);
        }
      }
    }

    local_maxima->width = local_maxima->points.size();
    local_maxima->height = 1;
    local_maxima->is_dense = true;

    return local_maxima;

  }

template<class PointT, class PointNormalT, class FeatureT>
  typename Eigen::ArrayXXi pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::getLocalMaximaGrid(
                                                                                                        Eigen::MatrixXf & grid,
                                                                                                        float window_size)
  {

    Eigen::ArrayXXi local_max = Eigen::ArrayXXi::Zero(grid.rows(), grid.cols());

    float max, min;
    max = grid.maxCoeff();
    min = grid.minCoeff();

    //std::cerr << min << " " << max << std::endl;
    float threshold = min + (max - min) * local_maxima_threshold_;
    //float threshold = local_maxima_threshold_;

    int window_size_pixels = window_size / cell_size_;

    if (window_size_pixels >= std::min(grid.cols() - 3, grid.rows() - 3))
    {
      window_size_pixels = std::min(grid.cols() - 3, grid.rows() - 3);
    }

    // Make window_size_pixels even
    if (window_size_pixels % 2 == 0)
      window_size_pixels++;

    int side = window_size_pixels / 2;

    for (int i = side; i < (grid.rows() - side); i++)
    {
      for (int j = side; j < (grid.cols() - side); j++)
      {

        float max;
        Eigen::MatrixXf window = grid.block(i - side, j - side, window_size_pixels, window_size_pixels);
        max = window.maxCoeff();

        assert(window.cols() == window_size_pixels);
        assert(window.rows() == window_size_pixels);

        // if max of the window is in its center then this point is local maxima
        if ((max == grid(i, j)) && (max > 0) && (max > threshold))
        {
          local_max(i, j) = 1;
        }
      }
    }

    return local_max;

  }

template<class PointT, class PointNormalT, class FeatureT>
  vector<boost::shared_ptr<std::vector<int> > > pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::findVotedSegments(
                                                                                                                            typename pcl::PointCloud<
                                                                                                                                PointT>::Ptr local_maxima_,
                                                                                                                            const string & class_name,
                                                                                                                            float window_size)
  {

    vector<boost::shared_ptr<std::vector<int> > > voted_segments_;

    std::vector<std::set<int> > segment_combinations;

    for (size_t j = 0; j < local_maxima_->points.size(); j++)
    {
      PointT local_maxima = local_maxima_->points[j];
      std::vector<int> idx;

      for (size_t i = 0; i < votes_[class_name].points.size(); i++)
      {

        bool in_cell_x1 = votes_[class_name].points[i].x > (local_maxima.x - window_size / 2);
        bool in_cell_x2 = votes_[class_name].points[i].x < (local_maxima.x + window_size / 2);
        bool in_cell_y1 = votes_[class_name].points[i].y > (local_maxima.y - window_size / 2);
        bool in_cell_y2 = votes_[class_name].points[i].y < (local_maxima.y + window_size / 2);

        if (in_cell_x1 && in_cell_x2 && in_cell_y1 && in_cell_y2)
        {
          idx.push_back(i);
        }
      }

      std::set<int> segment_idx;

      for (size_t i = 0; i < idx.size(); i++)
      {
        segment_idx.insert(voted_segment_idx_[class_name][idx[i]]);
      }

      segment_combinations.push_back(segment_idx);

    }

    std::unique(segment_combinations.begin(), segment_combinations.end());

    for (size_t i = 0; i < segment_combinations.size(); i++)
    {
      //PointNormalCloudPtr cloud(new PointNormalCloud);
      boost::shared_ptr<std::vector<int> > cloud_idx(new std::vector<int>);

      for (std::set<int>::iterator it = segment_combinations[i].begin(); it != segment_combinations[i].end(); it++)
      {
        //PointNormalCloud segment(*scene_, *segment_indices_[*it]);
        cloud_idx->insert(cloud_idx->begin(), segment_indices_[*it]->begin(), segment_indices_[*it]->end());

        //*cloud += segment;
      }

      voted_segments_.push_back(cloud_idx);

    }

    if (debug_)
    {

      for (size_t i = 0; i < voted_segments_.size(); i++)
      {
        PointNormalCloud seg(*scene_, *voted_segments_[i]);
        std::stringstream ss;
        ss << debug_folder_ << class_name << "_segment_" << i << ".pcd";
        std::cerr << "Writing to file " << ss.str() << std::endl;
        pcl::io::savePCDFileASCII(ss.str(), seg);
      }
    }

    return voted_segments_;

  }

template<class PointT, class PointNormalT, class FeatureT>
  void pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::fitModelsWithRansac(
                                                                                     vector<boost::shared_ptr<
                                                                                         std::vector<int> > > & voted_segments_,
                                                                                     const string class_name,
                                                                                     RandomSampleConsensusSimple<
                                                                                         PointNormalT> & ransac,
                                                                                     vector<PointNormalCloudPtr> & result_,
                                                                                     vector<float> & scores_)
  {

    for (size_t i = 0; i < voted_segments_.size(); i++)
    {

      ransac.setSceneSegment(voted_segments_[i]);

      ransac.computeModel();

      float score = ransac.getBestScore();

      std::cerr << "Score " << score << std::endl;

      if (score > ransac_result_threshold_[class_name])
      {
        Eigen::VectorXf model_coefs = ransac.getBestModelCoeficients();

        Eigen::Affine3f transform;
        transform.setIdentity();
        transform.translate(Eigen::Vector3f(model_coefs[0], model_coefs[1], 0));
        transform.rotate(Eigen::AngleAxisf(model_coefs[2], Eigen::Vector3f(0, 0, 1)));

        PointNormalCloudPtr full_model_transformed(new PointNormalCloud);
        pcl::transformPointCloudWithNormals(*ransac.getModel(), *full_model_transformed, transform);

        result_.push_back(full_model_transformed);
        scores_.push_back(1 - score);

      }
    }
  }

template<class PointT, class PointNormalT, class FeatureT>
  void pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::generateVisibilityScore(
                                                                                         vector<PointNormalCloudPtr> & result_,
                                                                                         vector<float> & scores_)
  {

    pcl::octree::OctreePointCloudSearch<PointNormalT> octree(subsampling_resolution_ * 2.5f);
    octree.setInputCloud(scene_);
    octree.addPointsFromInputCloud();

    for (size_t i = 0; i < result_.size(); i++)
    {
      int free = 0, occupied = 0, occluded = 0;
      for (size_t j = 0; j < result_[i]->points.size(); j++)
      {
        PointNormalT point = result_[i]->points[j];

        if (octree.isVoxelOccupiedAtPoint(point))
        {
          occupied++;

          continue;
        }

        Eigen::Vector3f sensor_orig = scene_->sensor_origin_.head(3);
        Eigen::Vector3f look_at = point.getVector3fMap() - sensor_orig;

        std::vector<int> indices;
        octree.getIntersectedVoxelIndices(sensor_orig, look_at, indices);

        bool is_occluded = false;
        if (indices.size() > 0)
        {
          for (size_t k = 0; k < indices.size(); k++)
          {
            Eigen::Vector3f ray = scene_->points[indices[k]].getVector3fMap() - sensor_orig;

            if (ray.norm() < look_at.norm())
            {
              is_occluded = true;
            }

          }
        }

        if (is_occluded)
        {
          occluded++;
          continue;
        }

        free++;

      }

      scores_[i] = 1 - ((float)2 * occupied + occluded) / (2 * occupied + occluded + free);
      //std::cerr << "Score " << occupied << " " << occluded << " " << free << " " << score[i] << std::endl;

    }

  }

template<class PointT, class PointNormalT, class FeatureT>
  bool pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::intersectXY(
                                                                             const pcl::PointCloud<PointNormalT> & cloud1,
                                                                             const pcl::PointCloud<PointNormalT> & cloud2)
  {

    PointNormalT min1, max1, min2, max2;
    pcl::getMinMax3D<PointNormalT>(cloud1, min1, max1);
    pcl::getMinMax3D<PointNormalT>(cloud2, min2, max2);

    bool intersectX, intersectY;
    if (min1.x < min2.x)
      intersectX = max1.x > min2.x;
    else if (min1.x > min2.x)
      intersectX = max2.x > min1.x;
    else
      // min1.x == min2.x
      intersectX = true;

    if (min1.y < min2.y)
      intersectY = max1.y > min2.y;
    else if (min1.y > min2.y)
      intersectY = max2.y > min1.y;
    else
      // min1.y == min2.y
      intersectY = true;

    return intersectX && intersectY;

  }

template<class PointT, class PointNormalT, class FeatureT>
  vector<typename pcl::PointCloud<PointNormalT>::Ptr> pcl::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::removeIntersecting(
                                                                                                                                   vector<
                                                                                                                                       typename pcl::PointCloud<
                                                                                                                                           PointNormalT>::Ptr> & result_,
                                                                                                                                   vector<
                                                                                                                                       float> & scores_)
  {

    vector<PointNormalCloudPtr> no_intersect_result;

    if (result_.size() == 0)
      return no_intersect_result;

    for (size_t i = 0; i < result_.size(); i++)
    {
      bool best = true;
      for (size_t j = 0; j < result_.size(); j++)
      {
        if (intersectXY(*result_[i], *result_[j]))
        {
          if (scores_[i] > scores_[j])
            best = false;
        }

      }
      if (best)
      {
        no_intersect_result.push_back(result_[i]);
      }
    }

    return no_intersect_result;

  }

#endif /* PHVOBJECTCLASSIFIER_HPP_ */

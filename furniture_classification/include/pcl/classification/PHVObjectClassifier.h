/*
 * PHVObjectClassifier.h
 *
 *  Created on: Jun 7, 2012
 *      Author: vsu
 */

#ifndef PHVOBJECTCLASSIFIER_H_
#define PHVOBJECTCLASSIFIER_H_

#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/transforms.h>

#include <pcl/sample_consensus/ransac.h>
#include <sac_3dof.h>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <yaml-cpp/yaml.h>

#include <map>

using std::map;
using std::string;
using std::vector;

namespace pcl
{

template<class PointT, class PointNormalT, class FeatureT>
  class PHVObjectClassifier
  {
  public:

    typedef typename pcl::PointCloud<PointT> PointCloud;
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

    typedef typename pcl::search::KdTree<PointT> PointTree;
    typedef typename pcl::search::KdTree<PointT>::Ptr PointTreePtr;

    typedef typename pcl::search::KdTree<PointNormalT> PointNormalTree;
    typedef typename pcl::search::KdTree<PointNormalT>::Ptr PointNormalTreePtr;

    typedef typename pcl::PointCloud<PointNormalT> PointNormalCloud;
    typedef typename pcl::PointCloud<PointNormalT>::Ptr PointNormalCloudPtr;
    typedef typename pcl::PointCloud<PointNormalT>::ConstPtr PointNormalCloudConstPtr;

    typedef typename Feature<PointNormalT, FeatureT>::Ptr FeatureEstimatorType;
    typedef typename boost::shared_ptr<MovingLeastSquares<PointT, PointNormalT> > MovingLeastSquaresType;

    typedef map<FeatureT, map<string, PointCloud> > DatabaseType;
    typedef map<string, vector<PointNormalCloudPtr> > ModelMapType;
    typedef typename ModelMapType::value_type ModelMapValueType;

    PHVObjectClassifier() :
      subsampling_resolution_(0.02f), mls_polynomial_order_(2), mls_search_radius_(0.03f), min_points_in_segment_(100),
          rg_residual_threshold_(0.05f), rg_smoothness_threshold_(40 * M_PI / 180), fe_k_neighbours_(10),
          num_clusters_(40), num_neighbours_(1), cell_size_(0.01), window_size_(0.6f), local_maxima_threshold_(0.4),
          ransac_distance_threshold_(0.01f), ransac_probability_(0.9), ransac_num_iter_(100), debug_(false),
          debug_folder_(""), mls_(new MovingLeastSquares<PointT, PointNormalT> )
    {
    }
    virtual ~PHVObjectClassifier()
    {
    }
    ;

    virtual void addObjectPartialView(PointCloudConstPtr view, const std::string & class_name)
    {
      class_name_to_partial_views_map_[class_name].push_back(estimateNormalsAndSubsample(view));

    }
    virtual void addObjectFullModel(PointCloudConstPtr model, const std::string & class_name)
    {
      class_name_to_full_models_map_[class_name].push_back(estimateNormalsAndSubsample(model));
    }

    virtual void computeClassifier()
    {

      BOOST_FOREACH(ModelMapValueType &v, class_name_to_partial_views_map_)
{      for(size_t i=0; i<v.second.size(); i++)
      {
        appendFeaturesFromCloud(v.second[i], v.first, i);

      }

    }

    // Transform to model centroinds in local coordinate frame of the segment
    centroids_.getMatrixXfMap() *= -1;

    normalizeFeatures(features_);

    vector<FeatureT> cluster_centers;
    vector<int> cluster_labels;

    clusterFeatures(cluster_centers, cluster_labels);

    database_.clear();

    for (size_t i = 0; i < cluster_labels.size(); i++)
    {
      database_[cluster_centers[cluster_labels[i]]][classes_[i]].points.push_back(centroids_[i]);
      database_[cluster_centers[cluster_labels[i]]][classes_[i]].width
      = database_[cluster_centers[cluster_labels[i]]][classes_[i]].points.size();
      database_[cluster_centers[cluster_labels[i]]][classes_[i]].height = 1;
      database_[cluster_centers[cluster_labels[i]]][classes_[i]].is_dense = true;

    }

  }

  virtual bool isClassifierComputed()
  {
    return !this->database_.empty();
  }

  void saveToFile(string filename);

  void loadFromFile(string filename);

  void setDebug(bool debug)
  {
    debug_ = debug;
  }

  bool getDebug()
  {
    return debug_;
  }

  void setDebugFolder(const string & debug_folder)
  {
    debug_folder_ = debug_folder;
    boost::filesystem::path debug_path(debug_folder_);
    if(boost::filesystem::exists(debug_path))
    {
      boost::filesystem::remove_all(debug_path);
    }

    boost::filesystem::create_directories(debug_path);

  }

  string getDebugFolder()
  {
    return debug_folder_;
  }

  void setFeatureEstimator(FeatureEstimatorType feature_estimator)
  {
    feature_estimator_ = feature_estimator;
  }

  FeatureEstimatorType getFeatureEstimator()
  {
    return feature_estimator_;
  }

  virtual void setScene(PointCloudConstPtr model)
  {
    scene_ = estimateNormalsAndSubsample(model);
    pcl::getMinMax3D<PointNormalT>(*scene_, min_scene_bound_, max_scene_bound_);
  }

  void classify()
  {
    appendFeaturesFromCloud(scene_, "Scene", 0);
    normalizeFeaturesWithCurrentMinMax(features_);
    vote();

    for (std::map<std::string, pcl::PointCloud<pcl::PointXYZI> >::const_iterator it = votes_.begin(); it != votes_.end(); it++)
    {
      projectVotesToGrid(it->second);
      findLocalMaximaInGrid();
      findVotedSegments(it->first);
      fitModelsWithRansac(it->first);
      generateVisibility(it->first);

    }

  }

  template<class PT, class PNT, class FT> friend YAML::Emitter& operator <<(YAML::Emitter& out, const PHVObjectClassifier< PT, PNT, FT> & h);

protected:

  PointNormalCloudPtr estimateNormalsAndSubsample(PointCloudConstPtr cloud)
  {
    PointNormalCloudPtr cloud_with_normals(new PointNormalCloud), cloud_downsampled(new PointNormalCloud);

    PointTreePtr tree(new PointTree);

    mls_->setComputeNormals(true);

    mls_->setInputCloud(cloud);
    mls_->setPolynomialFit(true);
    mls_->setPolynomialOrder(mls_polynomial_order_);
    mls_->setSearchMethod(tree);
    mls_->setSearchRadius(mls_search_radius_);

    this->mls_->process(*cloud_with_normals);

    pcl::VoxelGrid<PointNormalT> grid;
    grid.setInputCloud(cloud_with_normals);
    grid.setLeafSize(subsampling_resolution_, subsampling_resolution_, subsampling_resolution_);
    grid.filter(*cloud_downsampled);

    return cloud_downsampled;

  }

  void getSegmentsFromCloud(PointNormalCloudPtr cloud_with_normals, vector<boost::shared_ptr<vector<int> > > & segment_indices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & colored_segments)
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
    {
      if((int)i.size() > min_points_in_segment_)
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

      colored_segments.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*colored_segments_all, valid_segment_indices));

    }

  }

  void appendFeaturesFromCloud(PointNormalCloudPtr & cloud, const string & class_name, const int i)
  {
    if(debug_)
    {
      std::stringstream ss;
      ss << debug_folder_ << class_name << i << ".pcd";
      std::cerr << "Writing to file " << ss.str() << std::endl;
      pcl::io::savePCDFileASCII(ss.str(), *cloud);

    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_segments;
    vector<boost::shared_ptr<vector<int> > > segment_indices;
    getSegmentsFromCloud(cloud, segment_indices, colored_segments);

    segment_indices_ = segment_indices;

    PointNormalTreePtr tree(new PointNormalTree);

    feature_estimator_->setInputCloud(cloud);
    feature_estimator_->setSearchMethod(tree);
    feature_estimator_->setKSearch(fe_k_neighbours_);

    BOOST_FOREACH(const boost::shared_ptr<vector<int> > & idx, segment_indices)
    {

      // compute deature for segment
      pcl::PointCloud<FeatureT> feature;
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

  void normalizeFeatures(std::vector<FeatureT> & features)
  {

    int N = sizeof(min_.histogram)/sizeof(float);

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
        features[i].histogram[j] = (features[i].histogram[j] - min_.histogram[j])
        / (max_.histogram[j] - min_.histogram[j]);
      }
    }

  }

  void normalizeFeaturesWithCurrentMinMax(std::vector<FeatureT> & features)
  {
    int N = sizeof(min_.histogram)/sizeof(float);

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
        features[i].histogram[j] = (features[i].histogram[j] - min.histogram[j])
        / (max.histogram[j] - min.histogram[j]);
      }
    }

  }

  void clusterFeatures(vector<FeatureT> & cluster_centers, vector<int> & cluster_labels);

  void vote()
  {

    pcl::search::KdTree<FeatureT> feature_search;
    feature_search.setInputCloud(database_features_cloud_);

    for (size_t i = 0; i < features_.size(); i++)
    {
      std::vector<int> indices;
      std::vector<float> distances;
      feature_search.nearestKSearch(features_[i], num_neighbours_, indices, distances);

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
            model_centers_transformed_weighted.points[k].intensity = (1.0 / distances[j]) * (1.0 / model_centers.size());
            voted_segment_idx_[class_name].push_back(i);
          }

          votes_[class_name] += model_centers_transformed_weighted;
        }
      }

    }

  }

  void projectVotesToGrid(const pcl::PointCloud<pcl::PointXYZI> & model_centers)
  {
    int image_x_width = (int)((max_scene_bound_.x - min_scene_bound_.x) / cell_size_);
    int image_y_width = (int)((max_scene_bound_.y - min_scene_bound_.y) / cell_size_);

    grid_ = Eigen::MatrixXf::Zero(image_x_width, image_y_width);

    for (size_t i = 0; i < model_centers.points.size(); i++)
    {
      int vote_x = (model_centers.points[i].x - min_scene_bound_.x) / cell_size_;
      int vote_y = (model_centers.points[i].y - min_scene_bound_.y) / cell_size_;
      if ((vote_x >= 0) && (vote_y >= 0) && (vote_x < image_x_width) && (vote_y < image_y_width))
      grid_(vote_x, vote_y) += model_centers.points[i].intensity;
    }
  }

  void findLocalMaximaInGrid()
  {

    float max, min;
    max = grid_.maxCoeff();
    min = grid_.minCoeff();

    float threshold = min + (max - min) * local_maxima_threshold_;

    int window_size_pixels = window_size_ / cell_size_;

    // Make window_size_pixels even
    if (window_size_pixels % 2 == 0)
    window_size_pixels++;

    int side = window_size_pixels / 2;

    for (int i = side; i < (grid_.rows() - side); i++)
    {
      for (int j = side; j < (grid_.cols() - side); j++)
      {

        float max;
        Eigen::MatrixXf window = grid_.block(i - side, j - side, window_size_pixels, window_size_pixels);
        max = window.maxCoeff();

        assert(window.cols() == window_size_pixels);
        assert(window.rows() == window_size_pixels);

        // if max of the window is in its center then this point is local maxima
        if ((max == grid_(i, j)) && (max > 0) && (max > threshold))
        {
          PointT point;
          point.x = i * cell_size_ + min_scene_bound_.x;
          point.y = j * cell_size_ + min_scene_bound_.y;
          point.z = 0;
          local_maxima_.points.push_back(point);
        }
      }
    }

    local_maxima_.width = local_maxima_.points.size();
    local_maxima_.height = 1;
    local_maxima_.is_dense = true;

  }

  void findVotedSegments(const string & class_name)
  {

    std::vector<std::set<int> > segment_combinations;

    for (size_t j = 0; j < local_maxima_.points.size(); j++)
    {
      PointT local_maxima = local_maxima_.points[j];
      std::vector<int> idx;

      for (size_t i = 0; i < votes_[class_name].points.size(); i++)
      {

        bool in_cell_x1 = votes_[class_name].points[i].x > (local_maxima.x - window_size_/2);
        bool in_cell_x2 = votes_[class_name].points[i].x < (local_maxima.x + window_size_/2);
        bool in_cell_y1 = votes_[class_name].points[i].y > (local_maxima.y - window_size_/2);
        bool in_cell_y2 = votes_[class_name].points[i].y < (local_maxima.y + window_size_/2);

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
      PointNormalCloud cloud;

      for (std::set<int>::iterator it = segment_combinations[i].begin(); it != segment_combinations[i].end(); it++)
      {
        PointNormalCloud segment(*scene_, *segment_indices_[*it]);
        cloud += segment;
      }

      voted_segments_.push_back(cloud);

    }

  }

  void fitModelsWithRansac(const string class_name)
  {

    BOOST_FOREACH(PointNormalCloudPtr & full_model, class_name_to_full_models_map_[class_name])
    {
      for (size_t i = 0; i < voted_segments_.size(); i++)
      {

        typename pcl::SampleConsensusModel3DOF<PointNormalT>::Ptr
        model(
            new pcl::SampleConsensusModel3DOF<PointNormalT>(
                full_model));

        // TODO dont use makeShared
        model->setTarget(voted_segments_[i].makeShared());
        pcl::RandomSampleConsensus<PointNormalT> ransac(model);

        ransac.setDistanceThreshold(ransac_distance_threshold_);
        ransac.setProbability(ransac_probability_);
        ransac.setMaxIterations(ransac_num_iter_);
        ransac.computeModel();

        std::vector<int> tmp1;
        ransac.getInliers(tmp1);
        float weight = ((float)tmp1.size()) / full_model->points.size();

        if (weight > ransac_result_threshold_)
        {
          Eigen::VectorXf model_coefs;
          ransac.getModelCoefficients(model_coefs);

          Eigen::Affine3f transform;
          transform.setIdentity();
          transform.translate(Eigen::Vector3f(model_coefs[0], model_coefs[1], 0));
          transform.rotate(Eigen::AngleAxisf(model_coefs[2], Eigen::Vector3f(0, 0, 1)));

          pcl::PointCloud<pcl::PointNormal> full_model_transformed;
          pcl::transformPointCloudWithNormals(*full_model, full_model_transformed, transform);

          result_.push_back(full_model_transformed);
          scores_.push_back(1 - weight);

        }
      }
    }
  }

  void generateVisibilityScore()
  {

    pcl::octree::OctreePointCloudSearch<PointNormalT> octree(0.05f);
    octree.setInputCloud(scene_);
    octree.addPointsFromInputCloud();

    for (size_t i = 0; i < result_.size(); i++)
    {
      int free = 0, occupied = 0, occluded = 0;
      for (size_t j = 0; j < result_[i].points.size(); j++)
      {
        PointNormalT point = result_[i].points[j];

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


  bool intersectXY(const pcl::PointCloud<PointNormalT> & cloud1, const pcl::PointCloud<PointNormalT> & cloud2)
  {

    PointNormalT min1, max1, min2, max2;
    pcl::getMinMax3D<PointNormalT>(cloud1, min1, max1);
    pcl::getMinMax3D<PointNormalT>(cloud2, min2, max2);

    bool intersectX, intersectY;
    if (min1.x < min2.x)
      intersectX = max1.x > min2.x;
    else if (min1.x > min2.x)
      intersectX = max2.x > min1.x;
    else // min1.x == min2.x
      intersectX = true;

    if (min1.y < min2.y)
      intersectY = max1.y > min2.y;
    else if (min1.y > min2.y)
      intersectY = max2.y > min1.y;
    else // min1.y == min2.y
      intersectY = true;

    return intersectX && intersectY;

  }

  void removeIntersecting()
  {

    if(result_.size() == 0)
      return;

    vector<PointNormalCloud> no_intersect_result;

    for (size_t i = 0; i < result_.size(); i++)
    {
      bool best = true;
      for (size_t j = 0; j < result_.size(); j++)
      {
        if (intersectXY(result_[i], result_[j]))
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


    result_ = no_intersect_result;

  }

public:

  float subsampling_resolution_;
  int mls_polynomial_order_;
  float mls_search_radius_;
  int min_points_in_segment_;
  float rg_residual_threshold_;
  float rg_smoothness_threshold_;
  float fe_k_neighbours_;
  int num_clusters_;
  int num_neighbours_;
  float cell_size_;
  float window_size_;
  float local_maxima_threshold_;

  float ransac_distance_threshold_;
  float ransac_probability_;
  int ransac_num_iter_;
  float ransac_result_threshold_;

  bool debug_;
  string debug_folder_;

  FeatureEstimatorType feature_estimator_;
  MovingLeastSquaresType mls_;

  DatabaseType database_;
  typename pcl::PointCloud<FeatureT>::Ptr database_features_cloud_;

  ModelMapType class_name_to_partial_views_map_;
  ModelMapType class_name_to_full_models_map_;

  vector<FeatureT> features_;
  PointCloud centroids_;
  vector<std::string> classes_;
  vector<boost::shared_ptr<vector<int> > > segment_indices_;

  PointNormalCloudPtr scene_;
  PointNormalT min_scene_bound_, max_scene_bound_;
  map<string, pcl::PointCloud<pcl::PointXYZI> > votes_;
  map<string, vector<int> > voted_segment_idx_;
  Eigen::MatrixXf grid_;
  PointCloud local_maxima_;
  vector<PointNormalCloud> voted_segments_;
  std::vector<PointNormalCloud> result_;
  std::vector<float> scores_;

  FeatureT min_;
  FeatureT max_;

};

template<class FeatureT>
inline bool operator <(const FeatureT & f, const FeatureT & s)
{
  int N = sizeof(f.histogram)/sizeof(float);

  std::vector<float> v_f(f.histogram, f.histogram + N), v_s(s.histogram, s.histogram + N);
  return v_f < v_s;
}

template<class PT, class PNT, class FT>
YAML::Emitter& operator <<(YAML::Emitter& out, const pcl::PHVObjectClassifier<PT, PNT, FT> & h)
{

  out << YAML::BeginMap;

  out << YAML::Key << "subsampling_resolution";
  out << YAML::Value << h.subsampling_resolution_;

  out << YAML::Key << "mls_polynomial_order";
  out << YAML::Value << h.mls_polynomial_order_;

  out << YAML::Key << "mls_search_radius";
  out << YAML::Value << h.mls_search_radius_;

  out << YAML::Key << "min_points_in_segment";
  out << YAML::Value << h.min_points_in_segment_;

  out << YAML::Key << "rg_residual_threshold";
  out << YAML::Value << h.rg_residual_threshold_;

  out << YAML::Key << "rg_smoothness_threshold";
  out << YAML::Value << h.rg_smoothness_threshold_;

  out << YAML::Key << "fe_k_neighbours";
  out << YAML::Value << h.fe_k_neighbours_;

  out << YAML::Key << "num_clusters";
  out << YAML::Value << h.num_clusters_;

  out << YAML::Key << "debug";
  out << YAML::Value << h.debug_;

  out << YAML::Key << "debug_folder";
  out << YAML::Value << h.debug_folder_;

  out << YAML::Key << "database";
  out << YAML::Value << h.database_;

  out << YAML::Key << "min_feature";
  out << YAML::Value << h.min_;

  out << YAML::Key << "max_feature";
  out << YAML::Value << h.max_;
  //out << YAML::Key << "full_models";
  //out << YAML::Value << class_to_full_pointcloud_;
  out << YAML::EndMap;

  return out;
}

template<int N>
YAML::Emitter& operator <<(YAML::Emitter& out, const pcl::Histogram<N> & h)
{
  out << YAML::Flow << YAML::BeginSeq;
  for (int j = 0; j < N; j++)
  {
    out << h.histogram[j];
  }
  out << YAML::EndSeq << YAML::Block;
  return out;
}

template<int N>
void operator >>(const YAML::Node& node, pcl::Histogram<N> & h)
{
  for (int j = 0; j < N; j++)
  {
    node[j] >> h.histogram[j];
  }
}

template<typename PointT>
YAML::Emitter& operator <<(YAML::Emitter& out, const pcl::PointCloud<PointT> & cloud)
{
  out << YAML::BeginSeq;
  for (size_t i = 0; i < cloud.points.size(); i++)
  {
    out << YAML::Flow << YAML::BeginSeq << cloud.points[i].x << cloud.points[i].y << cloud.points[i].z
    << YAML::EndSeq << YAML::Block;
  }
  out << YAML::EndSeq;

  return out;
}

template<typename PointT>
void operator >>(const YAML::Node& node, pcl::PointCloud<PointT> & cloud)
{
  cloud.clear();

  for (size_t i = 0; i < node.size(); i++)
  {
    PointT point;
    node[i][0] >> point.x;
    node[i][1] >> point.y;
    node[i][2] >> point.z;
    cloud.points.push_back(point);

  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
}

template<class PointT, class FeatureT>
YAML::Emitter& operator <<(YAML::Emitter& out,const
    map<FeatureT, map<string, pcl::PointCloud<PointT> > > & database)
{
  out << YAML::BeginSeq;

  for (typename map<FeatureT, map<string, pcl::PointCloud<PointT> > >::const_iterator it =
      database.begin(); it != database.end(); it++)
  {
    FeatureT cluster_center = it->first;
    map<string, pcl::PointCloud<PointT> > class_centroid_map = it->second;

    out << YAML::BeginMap;
    out << YAML::Key << "cluster_center";
    out << YAML::Value << cluster_center;

    out << YAML::Key << "classes";
    out << YAML::Value << YAML::BeginSeq;

    for (typename map<string, pcl::PointCloud<PointT> >::const_iterator it2 =
        class_centroid_map.begin(); it2 != class_centroid_map.end(); it2++)
    {
      std::string class_name = it2->first;
      pcl::PointCloud<PointT> centroids = it2->second;
      out << YAML::BeginMap << YAML::Key << "class_name";
      out << YAML::Value << class_name;
      out << YAML::Key << "centroids";
      out << YAML::Value << centroids;
      out << YAML::EndMap;

    }

    out << YAML::EndSeq;

    out << YAML::EndMap;
  }

  out << YAML::EndSeq;
  return out;
}

template<class PointT, class FeatureT>
void operator >>(const YAML::Node& node, map<FeatureT, map<string, pcl::PointCloud<PointT> > > & database)
{
  for (size_t i = 0; i < node.size(); i++)
  {
    FeatureT cluster_center;
    node[i]["cluster_center"] >> cluster_center;

    for (size_t j = 0; j < node[i]["classes"].size(); j++)
    {
      std::string class_name;
      node[i]["classes"][j]["class_name"] >> class_name;
      node[i]["classes"][j]["centroids"] >> database[cluster_center][class_name];
    }
  }
}

template<class PT, class PNT, class FT>
void operator >>(const YAML::Node& node, pcl::PHVObjectClassifier<PT, PNT, FT> & h)
{
  node["subsampling_resolution"] >> h.subsampling_resolution_;
  node["mls_polynomial_order"] >> h.mls_polynomial_order_;
  node["mls_search_radius"] >> h.mls_search_radius_;
  node["min_points_in_segment"] >> h.min_points_in_segment_;
  node["rg_residual_threshold"] >> h.rg_residual_threshold_;
  node["rg_smoothness_threshold"] >> h.rg_smoothness_threshold_;

  node["fe_k_neighbours"] >> h.fe_k_neighbours_;
  node["num_clusters"] >> h.num_clusters_;
  node["debug"] >> h.debug_;
  node["debug_folder"] >> h.debug_folder_;

  node["database"] >> h.database_;
  node["database"] >> *(h.database_features_cloud_);

  node["min_feature"] >> h.min_;
  node["max_feature"] >> h.max_;

  //node["full_models"] >> h.class_to_full_pointcloud;

}

template<int N>
void operator >>(const YAML::Node& node, pcl::PointCloud<pcl::Histogram<N> > & feature_cloud)
{

  for (size_t i = 0; i < node.size(); i++)
  {
    pcl::Histogram<N> cluster_center;
    node[i]["cluster_center"] >> cluster_center;
    feature_cloud.points.push_back(cluster_center);
  }
  feature_cloud.width = feature_cloud.points.size();
  feature_cloud.height = 1;
  feature_cloud.is_dense = true;
}

}

#endif /* PHVOBJECTCLASSIFIER_H_ */

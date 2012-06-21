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

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

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
      subsampling_resolution_(0.02f), mls_polynomial_order_(2), mls_search_radius_(0.06f), min_points_in_segment_(100),
          rg_residual_threshold_(0.05f), rg_smoothness_threshold_(40 * M_PI / 180), fe_k_neighbours_(10),
          num_clusters_(40), num_neighbours_(1), cell_size_(0.01), window_size_(0.6f), local_maxima_threshold_(0.4f),
          ransac_distance_threshold_(0.01f), ransac_probability_(0.9), ransac_num_iter_(100),
          ransac_result_threshold_(0.5), debug_(false), debug_folder_(""), mls_(new MovingLeastSquares<PointT,
              PointNormalT> )
    {

      typedef pcl::PointCloud<FeatureT> PointFeatureCloud;
      database_features_cloud_.reset(new PointFeatureCloud);
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

    virtual void computeClassifier();

    virtual bool isClassifierComputed()
    {
      return !this->database_.empty();
    }

    void saveToFile();

    void loadFromFile();

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
      boost::filesystem::path debug_path(debug_folder);
      debug_folder_ = boost::filesystem::system_complete(debug_path).c_str();

      if (boost::filesystem::exists(debug_path))
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

    PointNormalCloudPtr getScene()
    {
      return scene_;
    }

    void classify();

    void setDatabaseDir(const string & database_dir)
    {
      database_dir_ = database_dir;
    }

    string getDatabaseDir()
    {
      return database_dir_;
    }

    map<string, vector<PointNormalCloudPtr> > getFoundObjects()
    {
      return found_objects_;
    }

    template<class PT, class PNT, class FT>
      friend YAML::Emitter& operator <<(YAML::Emitter& out, const PHVObjectClassifier<PT, PNT, FT> & h);

  protected:

    typename pcl::PointCloud<PointNormalT>::Ptr
    estimateNormalsAndSubsample(typename pcl::PointCloud<PointT>::ConstPtr cloud_orig);
    void getSegmentsFromCloud(PointNormalCloudPtr cloud_with_normals,
                              vector<boost::shared_ptr<vector<int> > > & segment_indices, pcl::PointCloud<
                                  pcl::PointXYZRGB>::Ptr & colored_segments);
    void appendFeaturesFromCloud(PointNormalCloudPtr & cloud, const string & class_name, const int i);
    void normalizeFeatures(std::vector<FeatureT> & features);
    void normalizeFeaturesWithCurrentMinMax(std::vector<FeatureT> & features);
    void clusterFeatures(vector<FeatureT> & cluster_centers, vector<int> & cluster_labels);
    void vote();
    Eigen::MatrixXf projectVotesToGrid(const pcl::PointCloud<pcl::PointXYZI> & model_centers);
    typename pcl::PointCloud<PointT>::Ptr findLocalMaximaInGrid(Eigen::MatrixXf grid);
    vector<typename pcl::PointCloud<PointNormalT>::Ptr>
    findVotedSegments(typename pcl::PointCloud<PointT>::Ptr local_maxima_, const string & class_name);
    void fitModelsWithRansac(vector<PointNormalCloudPtr> & voted_segments_, const string class_name, vector<
        PointNormalCloudPtr> & result_, vector<float> & scores_);
    void generateVisibilityScore(vector<PointNormalCloudPtr> & result_, vector<float> & scores_);
    bool intersectXY(const pcl::PointCloud<PointNormalT> & cloud1, const pcl::PointCloud<PointNormalT> & cloud2);
    vector<typename pcl::PointCloud<PointNormalT>::Ptr> removeIntersecting(vector<
        typename pcl::PointCloud<PointNormalT>::Ptr> & result_, vector<float> & scores_);

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
    string database_dir_;

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

    map<string, vector<PointNormalCloudPtr> > found_objects_;

    FeatureT min_;
    FeatureT max_;

  };

template<class FeatureT>
  inline bool operator <(const FeatureT & f, const FeatureT & s)
  {
    int N = sizeof(f.histogram) / sizeof(float);

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

    out << YAML::Key << "num_neighbours";
    out << YAML::Value << h.num_neighbours_;

    out << YAML::Key << "cell_size";
    out << YAML::Value << h.cell_size_;

    out << YAML::Key << "window_size";
    out << YAML::Value << h.window_size_;

    out << YAML::Key << "local_maxima_threshold";
    out << YAML::Value << h.local_maxima_threshold_;

    out << YAML::Key << "ransac_distance_threshold";
    out << YAML::Value << h.ransac_distance_threshold_;

    out << YAML::Key << "ransac_probability";
    out << YAML::Value << h.ransac_probability_;

    out << YAML::Key << "ransac_num_iter";
    out << YAML::Value << h.ransac_num_iter_;

    out << YAML::Key << "ransac_result_threshold";
    out << YAML::Value << h.ransac_result_threshold_;

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
    out << YAML::Key << "full_models";

    out << YAML::Value;
    out << YAML::BeginMap;

    typedef typename pcl::PHVObjectClassifier<PT, PNT, FT>::ModelMapValueType M;

    BOOST_FOREACH(M v, h.class_name_to_full_models_map_)
{    out << YAML::Key << v.first;
    out << YAML::Value << YAML::BeginSeq;

    for(size_t i=0; i<v.second.size(); i++)
    {
      std::stringstream ss;
      ss << "models/" << v.first << i << ".pcd";
      pcl::io::savePCDFileASCII(h.database_dir_ + ss.str(), *v.second[i]);
      out << ss.str();

    }
    out << YAML::EndSeq;

  }
  out << YAML::EndMap;

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

  node["num_neighbours"] >> h.num_neighbours_;
  node["cell_size"] >> h.cell_size_;
  node["window_size"] >> h.window_size_;
  node["local_maxima_threshold"] >> h.local_maxima_threshold_;

  node["ransac_distance_threshold"] >> h.ransac_distance_threshold_;
  node["ransac_probability"] >> h.ransac_probability_;
  node["ransac_num_iter"] >> h.ransac_num_iter_;
  node["ransac_result_threshold"] >> h.ransac_result_threshold_;

  node["debug"] >> h.debug_;
  node["debug_folder"] >> h.debug_folder_;

  node["database"] >> h.database_;
  node["database"] >> *(h.database_features_cloud_);

  node["min_feature"] >> h.min_;
  node["max_feature"] >> h.max_;

  map<string, vector<string> > full_models_locations;
  node["full_models"] >> full_models_locations;

  typedef map<string, vector<string> >::value_type vt;

  BOOST_FOREACH(vt &v, full_models_locations)
  {
    for(size_t i=0; i<v.second.size(); i++)
    {
      typename PointCloud<PNT>::Ptr cloud(new PointCloud<PNT>);
      pcl::io::loadPCDFile(h.database_dir_ + v.second[i], *cloud);
      h.class_name_to_full_models_map_[v.first].push_back(cloud);

    }

  }

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

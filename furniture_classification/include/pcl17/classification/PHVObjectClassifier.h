/*
 * PHVObjectClassifier.h
 *
 *  Created on: Jun 7, 2012
 *      Author: vsu
 */

#ifndef PHVOBJECTCLASSIFIER_H_
#define PHVOBJECTCLASSIFIER_H_

#include <pcl17/point_types.h>
#include <pcl17/features/feature.h>
#include <pcl17/search/flann_search.h>
#include <pcl17/search/impl/flann_search.hpp>
#include <pcl17/kdtree/kdtree_flann.h>
#include <pcl17/kdtree/impl/kdtree_flann.hpp>
#include <pcl17/surface/mls.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/filters/voxel_grid.h>
#include <pcl17/segmentation/region_growing.h>
#include <pcl17/common/transforms.h>

#include <pcl17/ModelCoefficients.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/point_types.h>
#include <pcl17/sample_consensus/method_types.h>
#include <pcl17/sample_consensus/model_types.h>
#include <pcl17/segmentation/sac_segmentation.h>
#include <pcl17/features/feature.h>
#include <pcl17/registration/icp.h>
#include <pcl17/registration/transformation_estimation_point_to_plane.h>

#include <ransac_simple.h>
#include <sac_3dof.h>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <yaml-cpp/yaml.h>

#include <map>
#include <set>
#include <furniture_classification/Hypothesis.h>

using std::map;
using std::string;
using std::vector;
using std::set;

namespace pcl17
{

template<class PointT, class PointNormalT, class FeatureT>
  class PHVObjectClassifier
  {
  public:

    typedef typename pcl17::PointCloud<PointT> PointCloud;
    typedef typename pcl17::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl17::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

    typedef typename pcl17::search::KdTree<PointT> PointTree;
    typedef typename pcl17::search::KdTree<PointT>::Ptr PointTreePtr;

    typedef typename pcl17::search::KdTree<PointNormalT> PointNormalTree;
    typedef typename pcl17::search::KdTree<PointNormalT>::Ptr PointNormalTreePtr;

    typedef typename pcl17::PointCloud<PointNormalT> PointNormalCloud;
    typedef typename pcl17::PointCloud<PointNormalT>::Ptr PointNormalCloudPtr;
    typedef typename pcl17::PointCloud<PointNormalT>::ConstPtr PointNormalCloudConstPtr;

    typedef typename Feature<PointNormalT, FeatureT>::Ptr FeatureEstimatorType;
    typedef typename boost::shared_ptr<MovingLeastSquares<PointT, PointNormalT> > MovingLeastSquaresType;

    typedef map<FeatureT, map<string, PointCloud> > DatabaseType;
    typedef map<string, vector<PointNormalCloudPtr> > ModelMapType;
    typedef typename ModelMapType::value_type ModelMapValueType;

    PHVObjectClassifier() :
      subsampling_resolution_(0.02f), mls_polynomial_fit_(false), mls_polynomial_order_(2), mls_search_radius_(0.05f),
          min_points_in_segment_(100), rg_residual_threshold_(0.05f), rg_smoothness_threshold_(40 * M_PI / 180),
          fe_k_neighbours_(10), num_clusters_(40), num_neighbours_(1), cell_size_(0.01), local_maxima_threshold_(0.5f),
          window_size_(0.3), ransac_distance_threshold_(0.01f), ransac_vis_score_weight_(5), ransac_num_iter_(200),
          icp_treshold_(0.03), num_angles_(36), debug_(false), debug_folder_(""), mls_(new MovingLeastSquares<PointT,
              PointNormalT> )
    {

      typedef pcl17::PointCloud<FeatureT> PointFeatureCloud;
      database_features_cloud_.reset(new PointFeatureCloud);
      ransac_result_threshold_["Armchairs"] = 0.01;
      ransac_result_threshold_["Chairs"] = 0.01;
      ransac_result_threshold_["Sideboards"] = 0.01;
      ransac_result_threshold_["Tables:"] = 0.1;

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
    void computeExternalClassifier(const std::string & labels);

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

    void setNumberOfClusters(int num_clusters)
    {
      num_clusters_ = num_clusters;
    }

    int getNumberOfClusters()
    {
      return num_clusters_;
    }

    void setFeatureEstimator(FeatureEstimatorType feature_estimator)
    {
      feature_estimator_ = feature_estimator;
    }

    FeatureEstimatorType getFeatureEstimator()
    {
      return feature_estimator_;
    }

    void setLocalMaximaThreshold(float t)
    {
      local_maxima_threshold_ = t;
    }

    float getLocalMaximaThreshold()
    {
      return local_maxima_threshold_;
    }

    virtual void setScene(PointCloudConstPtr model, float cut_off_distance = 2.5f)
    {
      std::vector<int> idx;

      pcl17::PassThrough<PointT> pass;
      pass.setInputCloud(model);
      pass.setFilterFieldName("x");
      pass.setFilterLimits(-cut_off_distance, cut_off_distance);
      pass.filter(idx);

      PointCloudPtr model_cut(new PointCloud(*model, idx));

      scene_ = estimateNormalsAndSubsample(model_cut);
      pcl17::getMinMax3D<PointNormalT>(*scene_, min_scene_bound_, max_scene_bound_);
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

    void eval_clustering(const std::string & classname, const float search_radius, double &tp, double &fn, double &fp);

    template<class PT, class PNT, class FT>
      friend YAML::Emitter& operator <<(YAML::Emitter& out, const PHVObjectClassifier<PT, PNT, FT> & h);

    void eval_clustering_external(const std::string & classname, const float search_radius, double &tp, double &fn,
                                  double &fp, const std::string & matrix);

    void vote_external(const std::string & matrix);
    furniture_classification::Hypothesis::Ptr generate_hypothesis(std::map<std::string, pcl17::PointCloud<
        pcl17::PointXYZ>::Ptr> & votes_map);
    PointNormalCloudPtr fit_objects(furniture_classification::Hypothesis::ConstPtr hp);

  protected:

    typename pcl17::PointCloud<PointNormalT>::Ptr
    estimateNormalsAndSubsample(typename pcl17::PointCloud<PointT>::ConstPtr cloud_orig);
    void getSegmentsFromCloud(PointNormalCloudPtr cloud_with_normals,
                              vector<boost::shared_ptr<vector<int> > > & segment_indices, pcl17::PointCloud<
                                  pcl17::PointXYZRGBNormal>::Ptr & colored_segments);
    void appendFeaturesFromCloud(PointNormalCloudPtr & cloud, const string & class_name, const int i);
    void normalizeFeatures(std::vector<FeatureT> & features);
    void normalizeFeaturesWithCurrentMinMax(std::vector<FeatureT> & features);
    void clusterFeatures(vector<FeatureT> & cluster_centers, vector<int> & cluster_labels);
    void vote();
    Eigen::MatrixXf projectVotesToGrid(const pcl17::PointCloud<pcl17::PointXYZI> & model_centers, int & grid_center_x,
                                       int & grid_center_y);
    typename pcl17::PointCloud<PointT>::Ptr findLocalMaximaInGrid(Eigen::MatrixXf grid, float window_size);
    vector<boost::shared_ptr<std::vector<int> > >
    findVotedSegments(typename pcl17::PointCloud<PointT>::Ptr local_maxima_, const string & class_name,
                      float window_size);
    void fitModelsWithRansac(vector<boost::shared_ptr<std::vector<int> > > & voted_segments_, const string class_name,
                             RandomSampleConsensusSimple<PointNormalT> & ransac, vector<PointNormalCloudPtr> & result_,
                             vector<float> & scores_);
    void generateVisibilityScore(vector<PointNormalCloudPtr> & result_, vector<float> & scores_);
    bool intersectXY(const pcl17::PointCloud<PointNormalT> & cloud1, const pcl17::PointCloud<PointNormalT> & cloud2);
    vector<typename pcl17::PointCloud<PointNormalT>::Ptr> removeIntersecting(vector<typename pcl17::PointCloud<
        PointNormalT>::Ptr> & result_, vector<float> & scores_);
    typename Eigen::ArrayXXi getLocalMaximaGrid(Eigen::MatrixXf & grid, float window_size);

  public:

    float subsampling_resolution_;
    bool mls_polynomial_fit_;
    int mls_polynomial_order_;
    float mls_search_radius_;
    int min_points_in_segment_;
    float rg_residual_threshold_;
    float rg_smoothness_threshold_;
    float fe_k_neighbours_;
    int num_clusters_;
    int num_neighbours_;
    float cell_size_;
    float local_maxima_threshold_;
    float window_size_;

    float ransac_distance_threshold_;
    float ransac_vis_score_weight_;
    int ransac_num_iter_;
    double icp_treshold_;
    int num_angles_;

    bool debug_;
    string debug_folder_;
    string database_dir_;

    FeatureEstimatorType feature_estimator_;
    MovingLeastSquaresType mls_;

    DatabaseType database_;
    typename pcl17::PointCloud<FeatureT>::Ptr database_features_cloud_;

    ModelMapType class_name_to_partial_views_map_;
    ModelMapType class_name_to_full_models_map_;

    vector<FeatureT> features_;
    PointCloud centroids_;
    vector<std::string> classes_;
    vector<PointNormalCloudPtr> segment_pointclouds_;
    vector<boost::shared_ptr<vector<int> > > segment_indices_;

    PointNormalCloudPtr scene_;
    PointNormalT min_scene_bound_, max_scene_bound_;
    map<string, pcl17::PointCloud<pcl17::PointXYZI> > votes_;
    map<string, vector<int> > voted_segment_idx_;

    map<string, float> ransac_result_threshold_;

    map<string, vector<PointNormalCloudPtr> > found_objects_;

    std::string external_classifier_;

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
  YAML::Emitter& operator <<(YAML::Emitter& out, const pcl17::PHVObjectClassifier<PT, PNT, FT> & h)
  {

    out << YAML::BeginMap;

    out << YAML::Key << "subsampling_resolution";
    out << YAML::Value << h.subsampling_resolution_;

    out << YAML::Key << "mls_polynomial_fit";
    out << YAML::Value << h.mls_polynomial_fit_;

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

    out << YAML::Key << "local_maxima_threshold";
    out << YAML::Value << h.local_maxima_threshold_;

    out << YAML::Key << "ransac_distance_threshold";
    out << YAML::Value << h.ransac_distance_threshold_;

    out << YAML::Key << "ransac_vis_score_weight";
    out << YAML::Value << h.ransac_vis_score_weight_;

    out << YAML::Key << "ransac_num_iter";
    out << YAML::Value << h.ransac_num_iter_;

    out << YAML::Key << "debug";
    out << YAML::Value << h.debug_;

    out << YAML::Key << "debug_folder";
    out << YAML::Value << h.debug_folder_;

    out << YAML::Key << "ransac_result_threshold";
    out << YAML::Value << h.ransac_result_threshold_;

    out << YAML::Key << "icp_treshold";
    out << YAML::Value << h.icp_treshold_;

    out << YAML::Key << "num_angles";
    out << YAML::Value << h.num_angles_;

    out << YAML::Key << "window_size";
    out << YAML::Value << h.window_size_;

    out << YAML::Key << "external_classifier";
    out << YAML::Value << h.external_classifier_;

    out << YAML::Key << "database";
    out << YAML::Value << h.database_;

    out << YAML::Key << "min_feature";
    out << YAML::Value << h.min_;

    out << YAML::Key << "max_feature";
    out << YAML::Value << h.max_;
    out << YAML::Key << "full_models";

    out << YAML::Value;
    out << YAML::BeginMap;

    typedef typename pcl17::PHVObjectClassifier<PT, PNT, FT>::ModelMapValueType M;

    BOOST_FOREACH(M v, h.class_name_to_full_models_map_)
    { out << YAML::Key << v.first;
      out << YAML::Value << YAML::BeginSeq;

      for(size_t i=0; i<v.second.size(); i++)
      {
        std::stringstream ss;
        ss << "models/" << v.first << i << ".pcd";
        pcl17::io::savePCDFileASCII(h.database_dir_ + ss.str(), *v.second[i]);
        out << ss.str();

      }
      out << YAML::EndSeq;

    }
    out << YAML::EndMap;

    out << YAML::EndMap;

    return out;
  }

template<int N>
  YAML::Emitter& operator <<(YAML::Emitter& out, const pcl17::Histogram<N> & h)
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
  void operator >>(const YAML::Node& node, pcl17::Histogram<N> & h)
  {
    for (int j = 0; j < N; j++)
    {
      node[j] >> h.histogram[j];
    }
  }

YAML::Emitter& operator <<(YAML::Emitter& out, const pcl17::ESFSignature640 & h)
{
  out << YAML::BeginSeq;
  for (int j = 0; j < 640; j++)
  {
    out << h.histogram[j];
  }
  out << YAML::EndSeq;
  return out;
}

void operator >>(const YAML::Node& node, pcl17::ESFSignature640 & h)
{
  for (int j = 0; j < 640; j++)
  {
    node[j] >> h.histogram[j];
  }
}

YAML::Emitter& operator <<(YAML::Emitter& out, const pcl17::VFHSignature308 & h)
{
  out << YAML::Flow << YAML::BeginSeq;
  for (int j = 0; j < 308; j++)
  {
    out << h.histogram[j];
  }
  out << YAML::EndSeq << YAML::Block;
  return out;
}

void operator >>(const YAML::Node& node, pcl17::VFHSignature308 & h)
{
  for (int j = 0; j < 308; j++)
  {
    node[j] >> h.histogram[j];
  }
}

YAML::Emitter& operator <<(YAML::Emitter& out, const pcl17::PointCloud<pcl17::ESFSignature640> & cloud)
{
  out << YAML::Flow << YAML::BeginSeq;
  for (size_t i = 0; i < cloud.points.size(); i++)
  {
    out << cloud.points[i];
  }
  out << YAML::EndSeq << YAML::Block;

  return out;
}

void operator >>(const YAML::Node& node, pcl17::PointCloud<pcl17::ESFSignature640> & cloud)
{
  cloud.clear();

  for (size_t i = 0; i < node.size(); i++)
  {
    pcl17::ESFSignature640 point;
    node[i]["cluster_center"] >> point;
    cloud.points.push_back(point);

  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
}

YAML::Emitter& operator <<(YAML::Emitter& out, const pcl17::PointCloud<pcl17::VFHSignature308> & cloud)
{
  out << YAML::BeginSeq;
  for (size_t i = 0; i < cloud.points.size(); i++)
  {
    out << cloud.points[i];
  }
  out << YAML::EndSeq;

  return out;
}

void operator >>(const YAML::Node& node, pcl17::PointCloud<pcl17::VFHSignature308> & cloud)
{
  cloud.clear();

  for (size_t i = 0; i < node.size(); i++)
  {
    pcl17::VFHSignature308 point;
    node[i]["cluster_center"] >> point;
    cloud.points.push_back(point);

  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
}

template<typename PointT>
  YAML::Emitter& operator <<(YAML::Emitter& out, const pcl17::PointCloud<PointT> & cloud)
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
  void operator >>(const YAML::Node& node, pcl17::PointCloud<PointT> & cloud)
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
  YAML::Emitter& operator <<(YAML::Emitter& out,
                             const map<FeatureT, map<string, pcl17::PointCloud<PointT> > > & database)
  {
    out << YAML::BeginSeq;

    for (typename map<FeatureT, map<string, pcl17::PointCloud<PointT> > >::const_iterator it = database.begin(); it
        != database.end(); it++)
    {
      FeatureT cluster_center = it->first;
      map<string, pcl17::PointCloud<PointT> > class_centroid_map = it->second;

      out << YAML::BeginMap;
      out << YAML::Key << "cluster_center";
      out << YAML::Value << cluster_center;

      out << YAML::Key << "classes";
      out << YAML::Value << YAML::BeginSeq;

      for (typename map<string, pcl17::PointCloud<PointT> >::const_iterator it2 = class_centroid_map.begin(); it2
          != class_centroid_map.end(); it2++)
      {
        std::string class_name = it2->first;
        pcl17::PointCloud<PointT> centroids = it2->second;
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
  void operator >>(const YAML::Node& node, map<FeatureT, map<string, pcl17::PointCloud<PointT> > > & database)
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
  void operator >>(const YAML::Node& node, pcl17::PHVObjectClassifier<PT, PNT, FT> & h)
  {
    node["subsampling_resolution"] >> h.subsampling_resolution_;
    node["mls_polynomial_fit"] >> h.mls_polynomial_fit_;
    node["mls_polynomial_order"] >> h.mls_polynomial_order_;
    node["mls_search_radius"] >> h.mls_search_radius_;
    node["min_points_in_segment"] >> h.min_points_in_segment_;
    node["rg_residual_threshold"] >> h.rg_residual_threshold_;
    node["rg_smoothness_threshold"] >> h.rg_smoothness_threshold_;
    node["external_classifier"] >> h.external_classifier_;

    node["fe_k_neighbours"] >> h.fe_k_neighbours_;
    node["num_clusters"] >> h.num_clusters_;

    node["num_neighbours"] >> h.num_neighbours_;
    node["cell_size"] >> h.cell_size_;
    node["local_maxima_threshold"] >> h.local_maxima_threshold_;

    node["ransac_distance_threshold"] >> h.ransac_distance_threshold_;
    node["ransac_vis_score_weight"] >> h.ransac_vis_score_weight_;
    node["ransac_num_iter"] >> h.ransac_num_iter_;
    node["ransac_result_threshold"] >> h.ransac_result_threshold_;


    node["icp_treshold"] >> h.icp_treshold_;
    node["num_angles"] >> h.num_angles_;
    node["window_size"] >> h.window_size_;


    node["debug"] >> h.debug_;
    node["debug_folder"] >> h.debug_folder_;

    node["min_feature"] >> h.min_;
    node["max_feature"] >> h.max_;

    node["database"] >> h.database_;
    node["database"] >> *(h.database_features_cloud_);

    map<string, vector<string> > full_models_locations;
    node["full_models"] >> full_models_locations;

    typedef map<string, vector<string> >::value_type vt;

BOOST_FOREACH  (vt &v, full_models_locations)
  {
    for(size_t i=0; i<v.second.size(); i++)
    {
      typename PointCloud<PNT>::Ptr cloud(new PointCloud<PNT>);
      pcl17::io::loadPCDFile(h.database_dir_ + v.second[i], *cloud);
      h.class_name_to_full_models_map_[v.first].push_back(cloud);

    }

  }

}

template<int N>
void operator >>(const YAML::Node& node, pcl17::PointCloud<pcl17::Histogram<N> > & feature_cloud)
{

  for (size_t i = 0; i < node.size(); i++)
  {
    pcl17::Histogram<N> cluster_center;
    node[i]["cluster_center"] >> cluster_center;
    feature_cloud.points.push_back(cluster_center);
  }
  feature_cloud.width = feature_cloud.points.size();
  feature_cloud.height = 1;
  feature_cloud.is_dense = true;
}

}

#endif /* PHVOBJECTCLASSIFIER_H_ */

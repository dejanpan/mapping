/*
 * training.cpp
 *
 *  Created on: Apr 25, 2012
 *      Author: vsu
 */

#include <training.h>

template<int N>
  void show_hist(const pcl::PointCloud<pcl::PointXYZ> & model, const pcl::PointCloud<pcl::Histogram<N> > & hist)
  {

  }

void filter_segments(const std::vector<std::vector<int> > & segment_indices,
                     std::vector<std::vector<int> > & new_segment_indices, size_t min_points_in_segment)
{
  new_segment_indices.clear();
  for (size_t i = 0; i < segment_indices.size(); i++)
  {
    if (segment_indices[i].size() > min_points_in_segment)
    {
      new_segment_indices.push_back(segment_indices[i]);
    }
  }
}

cv::Mat transform_to_mat(const std::vector<featureType> & features)
{
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

void transform_to_features(const cv::Mat & mat, std::vector<featureType> & features)
{
  features.clear();
  for (int i = 0; i < mat.rows; i++)
  {
    featureType f;
    for (int j = 0; j < mat.cols; j++)
    {
      f.histogram[j] = mat.at<float> (i, j);
    }
    features.push_back(f);
  }

}

void append_segments_from_file(const std::string & filename, std::vector<featureType> & features, pcl::PointCloud<
    pcl::PointXYZ> & centroids, std::vector<std::string> & classes, size_t min_points_in_segment, pcl::PointCloud<
    pcl::PointNormal> & scene, std::vector<std::vector<int> > & new_segment_indices, pcl::PointXYZ * min_bound,
                               pcl::PointXYZ * max_bound)
{
  std::vector<std::string> st;
  boost::split(st, filename, boost::is_any_of("/"), boost::token_compress_on);
  std::string class_name = st.at(st.size() - 3);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile(filename, *cloud);

  // Compute bounds if needed
  if ((min_bound != NULL) && (max_bound != NULL))
  {
    pcl::getMinMax3D<pcl::PointXYZ>(*cloud, *min_bound, *max_bound);
  }

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>),
                                         mls_points_filtered(new pcl::PointCloud<pcl::PointNormal>);

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

  mls.setComputeNormals(true);

  // Set parameters
  mls.setInputCloud(cloud);
  mls.setPolynomialFit(true);
  mls.setPolynomialOrder(2);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.03);

  // Reconstruct
  mls.process(*mls_points);

  pcl::PassThrough<pcl::PointNormal> ptfilter(true);
  ptfilter.setInputCloud(mls_points);
  ptfilter.filter(*mls_points_filtered);

  pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr mls_normal(new pcl::PointCloud<pcl::Normal>());
  pcl::copyPointCloud(*mls_points_filtered, *mls_cloud);
  pcl::copyPointCloud(*mls_points_filtered, *mls_normal);

  std::vector<std::vector<int> > segment_indices;

  // TODO Threshold to begining
  pcl::RegionGrowing<pcl::PointXYZ> region_growing;
  region_growing.setCloud(mls_cloud);
  region_growing.setNormals(mls_normal);
  region_growing.setNeighbourSearchMethod(tree);
  region_growing.setResidualTest(true);
  region_growing.setResidualThreshold(0.05);
  region_growing.setCurvatureTest(false);
  region_growing.setSmoothMode(false);
  region_growing.setSmoothnessThreshold(40 * M_PI / 180);
  region_growing.segmentPoints();
  segment_indices = region_growing.getSegments();

  // Return cloud scene
  scene = *mls_points_filtered;
  scene.sensor_origin_ = cloud->sensor_origin_;
  scene.sensor_orientation_ = cloud->sensor_orientation_;

  pcl::PointCloud<featureType> feature;
  featureEstimation feature_estimator;
  feature_estimator.setInputCloud(mls_cloud);
  feature_estimator.setSearchMethod(tree);
  feature_estimator.setKSearch(10);

  //std::vector<std::vector<int> > new_segment_indices;
  filter_segments(segment_indices, new_segment_indices, min_points_in_segment);

  for (size_t i = 0; i < new_segment_indices.size(); i++)
  {
    // Compute feature vector for segment
    boost::shared_ptr<std::vector<int> > idx(new std::vector<int>());
    *idx = new_segment_indices[i];
    feature_estimator.setIndices(idx);
    feature_estimator.compute(feature);

    // Compute centroid of segment
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*mls_cloud, new_segment_indices[i], centroid);
    pcl::PointXYZ centroid_point;
    centroid_point.x = centroid[0];
    centroid_point.y = centroid[1];
    centroid_point.z = centroid[2];

    features.push_back(feature.points[0]);
    classes.push_back(class_name);

    centroids.points.push_back(centroid_point);

  }

  centroids.width = centroids.points.size();
  centroids.height = 1;
  centroids.is_dense = true;

}

// TODO pass extension as parameter
void get_files_to_process(const std::string & input_dir, std::vector<std::string> & files_to_process, std::map<
    std::string, std::vector<std::string> > & class_to_full_pointcloud)
{
  PCL_INFO("Processing following files:\n");
  boost::filesystem::path input_path(input_dir);
  boost::filesystem::directory_iterator end_iter;
  for (boost::filesystem::directory_iterator iter(input_path); iter != end_iter; iter++)
  {
    boost::filesystem::path class_dir_path(*iter);
    std::string class_name = class_dir_path.filename().c_str();
    for (boost::filesystem::directory_iterator iter2(class_dir_path); iter2 != end_iter; iter2++)
    {
      boost::filesystem::path model_dir_path(*iter2);
      for (boost::filesystem::directory_iterator iter3(model_dir_path); iter3 != end_iter; iter3++)
      {
        boost::filesystem::path file(*iter3);
        if ((file.extension() == ".pcd") && (file.filename() != "full.pcd"))
        {
          files_to_process.push_back(file.c_str());
          PCL_INFO("\t%s\n", file.c_str());
        }
        if (file.filename() == "full.pcd")
        {
          class_to_full_pointcloud[class_name].push_back(boost::filesystem::system_complete(file).c_str());
          //PCL_INFO("\tClass name: %s Modes: %s\n", class_name.c_str(), file.c_str());
        }
      }

    }

  }
}

void cluster_features(const std::vector<featureType> & features, int num_clusters,
                      std::vector<featureType> & cluster_centers, std::vector<int> & cluster_labels)
{
  cv::Mat feature_vectors = transform_to_mat(features);
  cv::Mat centers(num_clusters, featureLength, feature_vectors.type()), labels;

  cv::kmeans(feature_vectors, num_clusters, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0), 3,
             cv::KMEANS_PP_CENTERS, centers);

  transform_to_features(centers, cluster_centers);
  cluster_labels = labels;
}

void create_codebook(const std::vector<featureType> & features, const pcl::PointCloud<pcl::PointXYZ> & centroids,
                     const std::vector<std::string> & classes, const std::vector<featureType> & cluster_centers,
                     const std::vector<int> & cluster_labels, databaseType & codebook)
{
  for (size_t i = 0; i < cluster_labels.size(); i++)
  {
    codebook[cluster_centers[cluster_labels[i]]][classes[i]].points.push_back(centroids[i]);
    codebook[cluster_centers[cluster_labels[i]]][classes[i]].width
        = codebook[cluster_centers[cluster_labels[i]]][classes[i]].points.size();
    codebook[cluster_centers[cluster_labels[i]]][classes[i]].height = 1;
    codebook[cluster_centers[cluster_labels[i]]][classes[i]].is_dense = true;

  }
}

YAML::Emitter& operator <<(YAML::Emitter& out, const databaseType & database)
{
  out << YAML::BeginSeq;

  for (databaseType::const_iterator it = database.begin(); it != database.end(); it++)
  {
    featureType cluster_center = it->first;
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ> > class_centroid_map = it->second;
    out << YAML::BeginMap;
    out << YAML::Key << "cluster_center";
    out << YAML::Value << cluster_center;

    out << YAML::Key << "classes";
    out << YAML::Value << YAML::BeginSeq;

    for (std::map<std::string, pcl::PointCloud<pcl::PointXYZ> >::const_iterator it2 = class_centroid_map.begin(); it2
        != class_centroid_map.end(); it2++)
    {
      std::string class_name = it2->first;
      pcl::PointCloud<pcl::PointXYZ> centroids = it2->second;
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

void operator >>(const YAML::Node& node, databaseType & database)
{
  for (size_t i = 0; i < node.size(); i++)
  {
    featureType cluster_center;
    node[i]["cluster_center"] >> cluster_center;

    for (size_t j = 0; j < node[i]["classes"].size(); j++)
    {
      std::string class_name;
      node[i]["classes"][j]["class_name"] >> class_name;
      node[i]["classes"][j]["centroids"] >> database[cluster_center][class_name];
    }
  }
}
void operator >>(const YAML::Node& node, pcl::PointCloud<featureType> & feature_cloud)
{

  for (size_t i = 0; i < node.size(); i++)
  {
    featureType cluster_center;
    node[i]["cluster_center"] >> cluster_center;
    feature_cloud.points.push_back(cluster_center);
  }
  feature_cloud.width = feature_cloud.points.size();
  feature_cloud.height = 1;
  feature_cloud.is_dense = true;
}

void save_codebook(const std::string & filename, const databaseType & database, const featureType & min,
                   const featureType & max,
                   const std::map<std::string, std::vector<std::string> > & class_to_full_pointcloud)
{

  YAML::Emitter out;

  out << YAML::BeginMap;
  out << YAML::Key << "database";
  out << YAML::Value << database;
  out << YAML::Key << "min_feature";
  out << YAML::Value << min;
  out << YAML::Key << "max_feature";
  out << YAML::Value << max;
  out << YAML::Key << "full_models";
  out << YAML::Value << class_to_full_pointcloud;
  out << YAML::EndMap;

  std::ofstream f;
  f.open(filename.c_str());
  f << out.c_str();
  f.close();

}

void load_codebook(const std::string & filename, databaseType & database, pcl::PointCloud<featureType> & feature_cloud,
                   featureType & min, featureType & max,
                   std::map<std::string, std::vector<std::string> > & class_to_full_pointcloud)
{
  feature_cloud.points.clear();

  std::ifstream fin(filename.c_str());
  YAML::Parser parser(fin);
  YAML::Node doc;
  parser.GetNextDocument(doc);

  doc["database"] >> database;
  doc["database"] >> feature_cloud;

  doc["min_feature"] >> min;
  doc["max_feature"] >> max;
  doc["full_models"] >> class_to_full_pointcloud;

}

void refineWithICP(const std::vector<std::string> & models, const pcl::PointCloud<pcl::PointXYZ> & local_maxima,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr scene, int num_rotations_icp, float icp_threshold, std::vector<
                       pcl::PointCloud<pcl::PointXYZ> > & result, std::vector<float> & score)
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  for (size_t model_idx = 0; model_idx < models.size(); model_idx++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr full_model(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(models[model_idx], *full_model);

    std::cerr << "Checking model:" << models[model_idx] << std::endl;

    for (size_t i = 0; i < local_maxima.points.size(); i++)
    {
      for (int j = 0; j < num_rotations_icp; j++)
      {
        float angle = 2 * M_PI * j / num_rotations_icp;
        Eigen::AngleAxis<float> rot(angle, Eigen::Vector3f(0, 0, 1));

        Eigen::Affine3f transform;
        transform.setIdentity();
        transform.translate(local_maxima.points[i].getVector3fMap());
        transform.rotate(rot);

        pcl::PointCloud<pcl::PointXYZ>::Ptr full_model_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*full_model, *full_model_transformed, transform);

        icp.setInputCloud(full_model_transformed);
        icp.setInputTarget(scene);
        pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
        icp.align(*Final);
        std::cerr << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

        if (icp.getFitnessScore() < icp_threshold)
        {
          result.push_back(*Final);
          score.push_back(icp.getFitnessScore());

          //              pcl::visualization::PCLVisualizer viz;
          //              viz.initCameraParameters();
          //              viz.updateCamera();
          //              pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(Final, 0, 255, 0);
          //              viz.addPointCloud<pcl::PointXYZ> (Final, single_color);
          //              viz.addPointCloud<pcl::PointXYZ> (scene, "cloud2");
          //              viz.spin();
        }

      }

    }
  }
}

void voteToGrid(pcl::PointCloud<pcl::PointXYZI> & model_centers, Eigen::MatrixXf & grid,
                const pcl::PointXYZ & min_bound, const pcl::PointXYZ & max_bound, float cell_size)
{

  int image_x_width = (int)((max_bound.x - min_bound.x) / cell_size);
  int image_y_width = (int)((max_bound.y - min_bound.y) / cell_size);

  grid = Eigen::MatrixXf::Zero(image_x_width, image_y_width);

  for (size_t i = 0; i < model_centers.points.size(); i++)
  {
    int vote_x = (model_centers.points[i].x - min_bound.x) / cell_size;
    int vote_y = (model_centers.points[i].y - min_bound.y) / cell_size;
    if ((vote_x >= 0) && (vote_y >= 0) && (vote_x < image_x_width) && (vote_y < image_y_width))
      grid(vote_x, vote_y) += model_centers.points[i].intensity;
  }
}

void saveGridToPGMFile(const std::string & filename, const Eigen::MatrixXf & grid)
{
  Eigen::MatrixXi img = (grid * 255.0 / grid.maxCoeff()).cast<int> ();

  std::ofstream f(filename.c_str());
  f << "P2\n" << grid.cols() << " " << grid.rows() << "\n255\n";
  f << img;
}

void findLocalMaxima(const Eigen::MatrixXf & grid, const float window_size, const pcl::PointXYZ & min_bound,
                     const float cell_size, const float local_maxima_threshold,
                     pcl::PointCloud<pcl::PointXYZ> & local_maxima)
{

  float max, min;
  max = grid.maxCoeff();
  min = grid.minCoeff();

  float threshold = min + (max - min) * local_maxima_threshold;

  int window_size_pixels = window_size / cell_size;

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
        pcl::PointXYZ point;
        point.x = i * cell_size + min_bound.x;
        point.y = j * cell_size + min_bound.y;
        point.z = 0;
        local_maxima.points.push_back(point);
      }
    }
  }

  local_maxima.width = local_maxima.points.size();
  local_maxima.height = 1;
  local_maxima.is_dense = true;

}


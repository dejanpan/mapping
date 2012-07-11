#ifndef PCL_SAMPLE_CONSENSUS_RANSAC_SIMPLE_H_
#define PCL_SAMPLE_CONSENSUS_RANSAC_SIMPLE_H_

#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace pcl
{
template<typename PointT>
  class RandomSampleConsensusSimple
  {
  public:
    RandomSampleConsensusSimple(float octree_res) :
      scene_octree_(octree_res)
    {
      max_iterations_ = 10000;
      eps_ = 0.01;
      weight_ = 2;
    }

    void setMaxIterations(int max_iterations)
    {
      max_iterations_ = max_iterations;
    }

    int getMaxIterations()
    {
      return max_iterations_;
    }

    void setScene(typename pcl::PointCloud<PointT>::Ptr & scene)
    {
      scene_ = scene;
      scene_octree_.setInputCloud(scene_);
      scene_octree_.addPointsFromInputCloud();
      std::cerr << "Tree depth " << scene_octree_.getTreeDepth() << " Scene size " << scene_->points.size()
          << std::endl;
    }

    void setSceneSegment(boost::shared_ptr<std::vector<int> > idx)
    {
      scene_segment_idx_ = idx;
    }

    void setModel(typename pcl::PointCloud<PointT>::Ptr & model)
    {
      model_ = model;
    }

    void computeModel()
    {

      best_score_ = 0;

      int max_iter_all = max_iterations_ * 10;
      int iter_all = 0;

      for (int i = 0; i < max_iterations_; i++)
      {

        int sample = scene_segment_idx_->at(rand() % scene_segment_idx_->size());

        //        while (!isSampleGood(sample) && iter_all < max_iter_all)
        //        {
        //          iter_all++;
        //          sample = scene_segment_idx_->at(rand() % scene_segment_idx_->size());
        //        }
        //
        //        if(iter_all >= max_iter_all) continue;

        Eigen::VectorXf model_coefficients;
        while (!computeModelCoefficients(sample, model_coefficients) && iter_all < max_iter_all)
        {
          iter_all++;
        }

        if (iter_all >= max_iter_all)
          continue;

        float score = countScore(model_coefficients);

        if (score > best_score_)
        {
          best_model_coefficients_ = model_coefficients;
          best_score_ = score;
        }

      }
    }

    bool computeModelCoefficients(int sample, Eigen::VectorXf & model_coefficients)
    {

      PointT scene_point = scene_->points[sample];

      // Select points with the same height
      std::vector<int> idx;

      pcl::PassThrough<PointT> pass;
      pass.setInputCloud(model_);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(scene_point.z - eps_, scene_point.z + eps_);
      pass.filter(idx);

      if (idx.size() == 0)
        return false;

      int rand_idx = rand() % (int)idx.size();

      PointT model_point = model_->points[idx[rand_idx]];

      model_coefficients.resize(3);

      model_coefficients[0] = model_point.x - scene_point.x;
      model_coefficients[1] = model_point.y - scene_point.y;

      if (scene_point.z < 0.8)
      {

        model_coefficients[2] = atan2(model_point.normal_y, model_point.normal_x) - atan2(scene_point.normal_y,
                                                                                          scene_point.normal_x);
      } else {
        model_coefficients[2] = ((double) rand())/RAND_MAX * M_2_PI;
      }
      model_coefficients *= -1;

      return true;

    }

    float countScore(const Eigen::VectorXf & model_coefficients)
    {
      pcl::PointCloud<PointT> transformed_model;

      Eigen::Affine3f transform;
      transform.setIdentity();
      transform.translate(Eigen::Vector3f(model_coefficients[0], model_coefficients[1], 0));
      transform.rotate(Eigen::AngleAxisf(model_coefficients[2], Eigen::Vector3f(0, 0, 1)));

      pcl::transformPointCloudWithNormals(*model_, transformed_model, transform);

      float score = generateVisibilityScore(transformed_model);

      //std::cerr << "Score " << score << std::endl;

      return score;

    }

    float generateVisibilityScore(const pcl::PointCloud<PointT> & cloud)
    {

      //      pcl::visualization::PCLVisualizer viz;
      //
      //      viz.addPointCloud<PointT>(scene_);
      //
      //      typename pcl::PointCloud<PointT>::Ptr cloud_ptr = cloud.makeShared();
      //
      //      pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud_ptr, 255, 0, 0);
      //
      //      viz.addPointCloud<PointT>(cloud_ptr, single_color, "cloud1");
      //
      //      viz.spin();

      int free = 0, occupied = 0, occluded = 0;
      for (size_t j = 0; j < cloud.points.size(); j++)
      {
        PointT point = cloud.points[j];

        if (scene_octree_.isVoxelOccupiedAtPoint(point))
        {
          occupied++;

          continue;
        }

        Eigen::Vector3f sensor_orig = scene_->sensor_origin_.head(3);
        //std::cerr << "Sensor origin\n" << sensor_orig << std::endl;
        Eigen::Vector3f look_at = point.getVector3fMap() - sensor_orig;

        std::vector<int> indices;
        scene_octree_.getIntersectedVoxelIndices(sensor_orig, look_at, indices);

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

      //std::cerr << "Score " << occupied << " " << occluded << " " << free << std::endl;
      return (weight_ * occupied + occluded) / (weight_ * occupied + occluded + free);

    }

    bool isSampleGood(int sample) const
    {
      return scene_->points[sample].normal_z < 0.8;
    }

    Eigen::VectorXf getBestModelCoeficients()
    {
      return best_model_coefficients_;
    }

    float getBestScore()
    {
      return best_score_;
    }

    void setWeight(float weight)
    {
      weight_ = weight;
    }

    typename pcl::PointCloud<PointT>::Ptr getModel(){
      return model_;
    }

    pcl::octree::OctreePointCloudSearch<PointT> scene_octree_;
    typename pcl::PointCloud<PointT>::Ptr scene_;
    boost::shared_ptr<std::vector<int> > scene_segment_idx_;
    typename pcl::PointCloud<PointT>::Ptr model_;

    Eigen::VectorXf best_model_coefficients_;
    float best_score_;
    float weight_;

    int max_iterations_;
    float eps_;

  };
}

#endif  //#ifndef PCL_SAMPLE_CONSENSUS_RANSAC_H_

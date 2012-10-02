#ifndef SAC_3DOF_H_
#define SAC_3DOF_H_

#include <pcl17/sample_consensus/sac_model.h>
#include <pcl17/sample_consensus/model_types.h>
#include <pcl17/filters/passthrough.h>
#include <pcl17/common/transforms.h>
#include <pcl17/search/kdtree.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/octree/octree_search.h>

namespace pcl17
{

template<typename PointT>
  class SampleConsensusModel3DOF : public SampleConsensusModel<PointT>
  {
    using SampleConsensusModel<PointT>::input_;
    using SampleConsensusModel<PointT>::indices_;
    using SampleConsensusModel<PointT>::samples_radius_search_;

  public:
    typedef typename SampleConsensusModel<PointT>::PointCloud PointCloud;
    typedef typename SampleConsensusModel<PointT>::PointCloudPtr PointCloudPtr;
    typedef typename SampleConsensusModel<PointT>::PointCloudConstPtr PointCloudConstPtr;

    typedef boost::shared_ptr<SampleConsensusModel3DOF> Ptr;
    typedef boost::shared_ptr<const SampleConsensusModel3DOF> ConstPtr;

    SampleConsensusModel3DOF(const PointCloudConstPtr & cloud, float octree_res) :
      SampleConsensusModel<PointT> (cloud), octree(octree_res)
    {
      setInputCloud(cloud);
      octree.setInputCloud(cloud);
      octree.addPointsFromInputCloud();

      std::cerr << "Sensor orig" << cloud->sensor_origin_ << std::endl;

    }

    PointCloudConstPtr getTarget() const
    {
      return target;
    }

    void setTarget(PointCloudConstPtr target)
    {
      this->target = target;
      this->target_tree.setInputCloud(this->target);
    }

    virtual bool computeModelCoefficients(const std::vector<int> & samples, Eigen::VectorXf & model_coefficients)
    {

      PointT input_point = input_->points[samples[0]];
      //std::cerr << "IP " << input_point << std::endl;

      // Select points with the same height
      std::vector<int> idx;

      pcl17::PassThrough<PointT> pass;
      pass.setInputCloud(target);
      pass.setIndices(target_idx);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(input_point.z - eps, input_point.z + eps);
      pass.filter(idx);

      if (idx.size() == 0)
        return false;

      int rand_idx = rand() % (int)idx.size();

      PointT target_point = target->points[idx[rand_idx]];
      //std::cerr << "TP " << target_point << std::endl;

      model_coefficients.resize(3);

      model_coefficients[0] = target_point.x - input_point.x;
      model_coefficients[1] = target_point.y - input_point.y;
      model_coefficients[2] = atan2(target_point.normal_y, target_point.normal_x) - atan2(input_point.normal_y,
                                                                                          input_point.normal_x);

      model_coefficients *= -1;

      //      PointCloud transformed_input;
      //
      //      Eigen::Affine3f transform;
      //      transform.setIdentity();
      //      transform.translate(Eigen::Vector3f(model_coefficients[0], model_coefficients[1], 0));
      //      transform.rotate(Eigen::AngleAxisf(model_coefficients[2], Eigen::Vector3f(0, 0, 1)));
      //
      //      //pcl17::io::savePCDFileASCII("tmp_inp.pcd", *input_);
      //      //pcl17::io::savePCDFileASCII("tmp_tgt.pcd", *target);
      //      ///std::cerr << "Transform " << transform.matrix() << std::endl;
      //      pcl17::transformPointCloudWithNormals(*input_, transformed_input, transform);
      //      //pcl17::io::savePCDFileASCII("tmp_res.pcd", transformed_input);
      //      PointT input_point_transformed = transformed_input.points[samples[0]];
      //
      //      //std::cerr << "IPT " << input_point_transformed << std::endl;
      //      //std::cerr << "MC " << model_coefficients << std::endl;

      return true;

    }

    virtual void selectWithinDistance(const Eigen::VectorXf & model_coefficients, const double threshold, std::vector<
        int> & inliers)
    {

      // return vector of zeros of size num_inliers
      int num_inliers = countWithinDistance(model_coefficients, threshold);
      inliers.clear();
      inliers.push_back(num_inliers);

      std::cerr << "Returned weight " << inliers[0] << std::endl;
    }

    virtual int countWithinDistance(const Eigen::VectorXf & model_coefficients, const double threshold)
    {
      PointCloud transformed_input;

      Eigen::Affine3f transform;
      transform.setIdentity();
      transform.translate(Eigen::Vector3f(model_coefficients[0], model_coefficients[1], 0));
      transform.rotate(Eigen::AngleAxisf(model_coefficients[2], Eigen::Vector3f(0, 0, 1)));

      pcl17::transformPointCloudWithNormals(*target, transformed_input, transform);

      std::vector<int> idx;
      std::vector<float> dist;

      int num_inliers = ((float)INT_MAX) * generateVisibilityScore(transformed_input);

      std::cerr << "Number of inliers " << num_inliers << std::endl;

      return num_inliers;

    }

    virtual void optimizeModelCoefficients(const std::vector<int> & inliers,
                                           const Eigen::VectorXf & model_coefficients,
                                           Eigen::VectorXf & optimized_coefficients)
    {
      optimized_coefficients = model_coefficients;
    }

    virtual void getDistancesToModel(const Eigen::VectorXf & model_coefficients, std::vector<double> & distances)
    {

    }

    virtual void projectPoints(const std::vector<int> & inliers, const Eigen::VectorXf & model_coefficients,
                               PointCloud & projected_points, bool copy_data_fields = true)
    {
    }

    virtual bool doSamplesVerifyModel(const std::set<int> & indices, const Eigen::VectorXf & model_coefficients,
                                      const double threshold)
    {
      return true;
    }

    virtual SacModel getModelType() const
    {
      return pcl17::SACMODEL_SPHERE;
    }

    inline virtual bool isModelValid(const Eigen::VectorXf &model_coefficients)
    {
      return true;
    }
    virtual bool isSampleGood(const std::vector<int> &samples) const
    {
      return input_->points[samples[0]].normal_z < 0.8;
    }

    inline unsigned int getSampleSize() const
    {
      return 1;
    }

    void setTargetIndices(boost::shared_ptr<std::vector<int> > & idx)
    {
      target_idx = idx;
    }

  protected:

    float generateVisibilityScore(const PointCloud & cloud)
    {

      int free = 0, occupied = 0, occluded = 0;
      for (size_t j = 0; j < cloud.points.size(); j++)
      {
        PointT point = cloud.points[j];

        if (octree.isVoxelOccupiedAtPoint(point))
        {
          occupied++;

          continue;
        }

        Eigen::Vector3f sensor_orig = input_->sensor_origin_.head(3);
        Eigen::Vector3f look_at = point.getVector3fMap() - sensor_orig;

        std::vector<int> indices;
        octree.getIntersectedVoxelIndices(sensor_orig, look_at, indices);

        bool is_occluded = false;
        if (indices.size() > 0)
        {
          for (size_t k = 0; k < indices.size(); k++)
          {
            Eigen::Vector3f ray = input_->points[indices[k]].getVector3fMap() - sensor_orig;

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

      return ((float)2 * occupied + occluded) / (2 * occupied + occluded + free);
      //std::cerr << "Score " << occupied << " " << occluded << " " << free << " " << score[i] << std::endl;


    }

    pcl17::octree::OctreePointCloudSearch<PointT> octree;
    PointCloudConstPtr target;
    boost::shared_ptr<std::vector<int> > target_idx;
    pcl17::search::KdTree<PointT> target_tree;

    float eps;

  };
}

#endif

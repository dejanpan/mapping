#ifndef SAC_3DOF_H_
#define SAC_3DOF_H_

#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>

namespace pcl
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

    SampleConsensusModel3DOF(const PointCloudConstPtr & cloud) :
      SampleConsensusModel<PointT> (cloud), eps(0.01)
    {
      setInputCloud(cloud);
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

      pcl::PassThrough<PointT> pass;
      pass.setInputCloud(target);
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



//      PointCloud transformed_input;
//
//      Eigen::Affine3f transform;
//      transform.setIdentity();
//      transform.translate(Eigen::Vector3f(model_coefficients[0], model_coefficients[1], 0));
//      transform.rotate(Eigen::AngleAxisf(model_coefficients[2], Eigen::Vector3f(0, 0, 1)));
//
//      //pcl::io::savePCDFileASCII("tmp_inp.pcd", *input_);
//      //pcl::io::savePCDFileASCII("tmp_tgt.pcd", *target);
//      ///std::cerr << "Transform " << transform.matrix() << std::endl;
//      pcl::transformPointCloudWithNormals(*input_, transformed_input, transform);
//      //pcl::io::savePCDFileASCII("tmp_res.pcd", transformed_input);
//      PointT input_point_transformed = transformed_input.points[samples[0]];
//
//      //std::cerr << "IPT " << input_point_transformed << std::endl;
//      //std::cerr << "MC " << model_coefficients << std::endl;

      return true;

    }

    virtual void selectWithinDistance(const Eigen::VectorXf & model_coefficients, const double threshold, std::vector<
        int> & inliers)
    {
      PointCloud transformed_input;

      Eigen::Affine3f transform;
      transform.setIdentity();
      transform.translate(Eigen::Vector3f(model_coefficients[0], model_coefficients[1], 0));
      transform.rotate(Eigen::AngleAxisf(model_coefficients[2], Eigen::Vector3f(0, 0, 1)));

      pcl::transformPointCloudWithNormals(*input_, transformed_input, transform);
      inliers.clear();

      std::vector<int> idx;
      std::vector<float> dist;

      //pcl::search::KdTree<PointT> target_tree;
      //target_tree.setInputCloud(this->target);

      for (size_t i = 0; i < transformed_input.points.size(); i++)
      {
        idx.clear();
        dist.clear();
        target_tree.nearestKSearch(transformed_input.points[i], 1, idx, dist);
        if (dist[0] < threshold)
          inliers.push_back(i);
      }
    }

    virtual int countWithinDistance(const Eigen::VectorXf & model_coefficients, const double threshold)
    {
      std::vector<int> inliers;
      selectWithinDistance(model_coefficients, threshold, inliers);
      //std::cerr << "Number of inliers " << inliers.size() << std::endl;
      return inliers.size();
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
      return pcl::SACMODEL_SPHERE;
    }

    inline virtual bool isModelValid(const Eigen::VectorXf &model_coefficients)
    {
      return true;
    }
    virtual bool isSampleGood(const std::vector<int> &samples) const
    {
      return input_->points[samples[0]].normal_z < 0.3;
    }

    inline unsigned int getSampleSize() const
    {
      return 1;
    }

  protected:
    PointCloudConstPtr target;
    pcl::search::KdTree<PointT> target_tree;

    float eps;

  };
}

#endif

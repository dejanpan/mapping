#ifndef PCL_IAS_SAMPLE_CONSENSUS_SACMODELORIENTATION_H_
#define PCL_IAS_SAMPLE_CONSENSUS_SACMODELORIENTATION_H_

// SaC
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/features/normal_3d.h>

#include <boost/make_shared.hpp>
// Kd Tree
//#include <pcl/kdtree/kdtree_ann.h>

// Eigen
//#include <Eigen/Array>
//#include <Eigen/Geometry>

namespace pcl
{
  /** \brief Given a point/vector, an axis and an angle, it returns the rotated input
   * \note if it needs to be done multiple times, computing the rotation matrix and applying it is faster! -- see: cloud_geometry::transforms::convertAxisAngleToRotationMatrix ()
   * \param r the point or vector to be rotated
   * \param axis the rotation axis
   * \param angle the rotation angle in radians
   */
  inline Eigen::Vector3f rotateAroundAxis (Eigen::Vector3f r, Eigen::Vector3f axis, float angle)
  {
    // Rodrigues' rotation formula
    return r * cos(angle) + (axis.cross(r)) * sin(angle) + axis * axis.dot(r) * (1-cos(angle));
  }

  /** \brief Making sure that only the normals are used from the input point cloud. */
  template <typename NormalT>
  class NormalPointRepresentation : public PointRepresentation <NormalT>
  {
    public:

      using PointRepresentation<NormalT>::nr_dimensions_;

      typedef boost::shared_ptr<NormalPointRepresentation<NormalT> > Ptr;
      typedef boost::shared_ptr<const NormalPointRepresentation<NormalT> > ConstPtr;

      NormalPointRepresentation ()
      {
        nr_dimensions_ = 3;
      }

      virtual void
      copyToFloatArray (const NormalT &p, float * out) const
      {
        out[0] = p.normal[0];
        out[1] = p.normal[1];
        out[2] = p.normal[2];
      }
  };

  /** \brief A Sample Consensus Model class for determining the 2 perpendicular directions to which most normals align.
   */
  template <typename NormalT>
  class SACModelOrientation : public SampleConsensusModel<NormalT>
  {
    public:
      using SampleConsensusModel<NormalT>::input_;
      using SampleConsensusModel<NormalT>::indices_;

      typedef typename SampleConsensusModel<NormalT>::PointCloud Normals;
      typedef typename SampleConsensusModel<NormalT>::PointCloudPtr NormalsPtr;
      typedef typename SampleConsensusModel<NormalT>::PointCloudConstPtr NormalsConstPtr;

      typedef boost::shared_ptr<SACModelOrientation> Ptr;
      typedef boost::shared_ptr<const SACModelOrientation> ConstPtr;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief The fixed axis (the sides will be perpendicular to it). */
      Eigen::Vector3f axis_;

      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor for base SACModelOrientation
        * \param cloud the input point cloud dataset containing the normals
        */
      SACModelOrientation (const NormalsConstPtr &cloud) : SampleConsensusModel<NormalT> (cloud)
      {
        typename NormalPointRepresentation<NormalT>::ConstPtr repr (new NormalPointRepresentation<NormalT>);
        kdtree_ = boost::make_shared<KdTreeFLANN <NormalT> >();
        kdtree_->setPointRepresentation (repr);
        kdtree_->setInputCloud (cloud);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor for base SACModelOrientation
        * \param cloud the input point cloud dataset
        * \param indices a vector of point indices to be used from \a cloud
        */
      SACModelOrientation (const NormalsConstPtr &cloud, const std::vector<int> &indices) : SampleConsensusModel<NormalT> (cloud, indices)
      {
        kdtree_ = boost::make_shared<KdTreeFLANN <NormalT> >();
        kdtree_->setInputCloud (cloud);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Specify the fixed "up" axis (the sides will be perpendicular to it).
        * \param axis a direction in 3D space.
        */
      inline void setAxis (Eigen::Vector3f axis) { axis_ = axis; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get the fixed axis. */
      inline Eigen::Vector3f getAxis () const { return (axis_); }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get a random point and return its index.
        * \param iterations the internal number of iterations used by SAC methods (incremented at samplings that can not produce a model) -- Not needed.
        * \param samples the resultant model samples
        */
      void getSamples (int &iterations, std::vector<int> &samples);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Check whether the given index samples can form a valid model, compute the model coefficients from
        * these samples and store them internally in model_coefficients_.
        * @note Model coefficients are the normal of the sample point (and it's index for testModelCoefficients)
        * \param samples the point indices found as possible good candidates for creating a valid model
        */
      bool computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXf &model_coefficients);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Recompute the model coefficients using the given inlier set and return them to the user. Pure virtual.
        * @note: These are the coefficients of the model after refinement (eg. after a least-squares optimization)
        * \param inliers the data inliers found as supporting the model (each element is an index of indices_)
        * \param refit_coefficients the resultant recomputed coefficients (has -1 as source index in refit_coefficients[3] at failures)
        */
      void refitModel (const std::vector<int> &inliers, std::vector<double> &refit_coefficients);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Compute all distances from the cloud data to a given model. -- Not needed.
        * \param model_coefficients the coefficients of a model that we need to compute distances to
        * \param distances the resultant estimated distances
        */
      void getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) {}

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Select all the points which respect the given model coefficients as inliers.
        * \param model_coefficients the coefficients of a model that we need to compute distances to
        * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
        * \param inliers the resultant model inliers (each element is an index of indices_)
        * @note: To get the refined inliers of a model, use:
        *        refined_coeff = refitModel (...); selectWithinDistance (refined_coeff, threshold);
        */
      void selectWithinDistance (const Eigen::VectorXf &model_coefficients, double threshold, std::vector<int> &inliers);
      
      
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Count all the points which respect the given model coefficients as inliers.
        * 
        * \param[in] model_coefficients the coefficients of a model that we need to compute distances to
        * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
        * \return the resultant number of inliers
        */
      int countWithinDistance(const Eigen::VectorXf& model_coefficients, double threshold)
      {
        std::vector<int> inliers;
        selectWithinDistance (model_coefficients, threshold, inliers);
        return inliers.size ();
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Create a new point cloud with inliers projected onto the model.
        * \param inliers the data inliers that we want to project on the model (each element is an index of indices_)
        * \param model_coefficients the coefficients of a model
        * \param projected_points the resultant projected points
        */
      void projectPoints (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Normals &projected_points, bool copy_data_fields = true){}


      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Verify whether a subset of indices verifies the internal model coefficients.
        * \param indices the data indices that need to be tested against the model (each element is an index of indices_)
        * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
        */
      bool doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, double threshold) { return true; }

     /** \brief Check if a sample of indices results in a good sample of points    
        * indices. Pure virtual.
        * \param samples the resultant index samples                                     
        */
    bool
    isSampleGood(const std::vector<int> &samples) const;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return an unique id for each type of model employed. */
      //@TODO: fix return to return SACMODEL_ORIENTATION
      inline pcl::SacModel getModelType () const { return pcl::SacModel(SACMODEL_PLANE);}

      inline void optimizeModelCoefficients(   const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
                                        Eigen::VectorXf &optimized_coefficients){}

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:

       //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       /** \brief Check whether a model is valid given the user constraints.
        * \param model_coefficients the set of model coefficients
        */
       inline bool
          isModelValid (const Eigen::VectorXf &model_coefficients)
       {
          // Needs a valid model coefficients
          ROS_ASSERT (model_coefficients.size () == 4);
          return (true);
       }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Index of the nx channel (the x component of the normal vector). The next indices are assumed to be that of ny and nz! */
      //int nx_idx_;

    private:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Kd-Tree for searching in normal space. */
      typename KdTreeFLANN<NormalT>::Ptr kdtree_;
      std::vector<int> front_indices_;
      std::vector<int>  back_indices_;
      std::vector<int>  left_indices_;
      std::vector<int> right_indices_;
      std::vector<float> points_sqr_distances_;

  };
}

#include "pcl_ias_sample_consensus/pcl_sac_model_orientation.hpp"

#endif

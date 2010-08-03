/*
 * Based on code written by Radu Rusu
 * Modified by Hozefa Indorewala
 */
#ifndef _REGISTRATION_CORRESPONDENCES_CHECK_H_
#define _REGISTRATION_CORRESPONDENCES_CHECK_H_

#include <boost/function.hpp>
#include <boost/bind.hpp>

// PCL includes
#include <pcl/pcl_base.h>

#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_ann.h"

#include "pcl/registration/transforms.h"

namespace pcl
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b Registration represents the base registration class. All 3D registration methods should inherit from
    * this class.
    */
  template <typename PointSource, typename PointTarget>
  class RegistrationCorrespondencesCheck: public PCLBase<PointSource>
  {
    using PCLBase<PointSource>::initCompute;
    using PCLBase<PointSource>::deinitCompute;

    public:
      using PCLBase<PointSource>::input_;
      using PCLBase<PointSource>::indices_;

      typedef typename pcl::KdTree<PointTarget> KdTree;
      typedef typename pcl::KdTree<PointTarget>::Ptr KdTreePtr;

      typedef pcl::PointCloud<PointSource> PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

      typedef pcl::PointCloud<PointTarget> PointCloudTarget;
      typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
      typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty constructor. */
      RegistrationCorrespondencesCheck () : target_ (),
                        final_transformation_ (Eigen::Matrix4f::Identity ()),
                        transformation_ (Eigen::Matrix4f::Identity ()),
                        previous_transformation_ (Eigen::Matrix4f::Identity ()),
                        transformation_epsilon_ (0.0), corr_dist_threshold_ (std::numeric_limits<double>::max ()),
                        converged_ (false), min_number_correspondences_ (3), k_ (1)
      {
        tree_ = boost::make_shared<pcl::KdTreeANN<PointTarget> > ();     // ANN tree for nearest neighbor search
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty destructor. */
      virtual ~RegistrationCorrespondencesCheck () {};

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Provide a pointer to the input target target (e.g., the point cloud that we want to align the input
        * source to)
        * \param cloud the input point cloud target
        */
      virtual inline void setInputTarget (const PointCloudTargetConstPtr &cloud);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get a pointer to the input point cloud dataset target target. */
      inline PointCloudTargetConstPtr const getInputTarget () { return (target_ ); }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get the final transformation matrix estimated by the registration method. */
      inline Eigen::Matrix4f getFinalTransformation () { return (final_transformation_); }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get the last incremental transformation matrix estimated by the registration method. */
      inline Eigen::Matrix4f getLastIncrementalTransformation () { return (transformation_); }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the maximum number of iterations the internal optimization should run for.
        * \param nr_iterations the maximum number of iterations the internal optimization should run for
        */
      inline void setMaximumIterations (int nr_iterations) { max_iterations_ = nr_iterations; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get the maximum number of iterations the internal optimization should run for, as set by the user. */
      inline int getMaximumIterations () { return (max_iterations_); }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the maximum distance threshold between two correspondent points in source <-> target. If the
        * distance is larger than this threshold, the points will be ignored in the alignment process.
        * \param distance_threshold the maximum distance threshold between a point and its nearest neighbor correspondent
        * in order to be considered in the alignment process
        */
      inline void setMaxCorrespondenceDistance (double distance_threshold) { corr_dist_threshold_ = distance_threshold; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get the maximum distance threshold between two correspondent points in source <-> target. If the
        * distance is larger than this threshold, the points will be ignored in the alignment process.
        */
      inline double getMaxCorrespondenceDistance () { return (corr_dist_threshold_); }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the transformation epsilon (maximum allowable difference between two consecutive transformations)
        * in order for an optimization to be considered as having converged to the final solution.
        * \param epsilon the transformation epsilon in order for an optimization to be considered as having converged
        * to the final solution.
        */
      inline void setTransformationEpsilon (double epsilon) { transformation_epsilon_ = epsilon; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get the transformation epsilon (maximum allowable difference between two consecutive transformations)
        * as set by the user.
        */
      inline double getTransformationEpsilon () { return (transformation_epsilon_); }


      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Provide a boost shared pointer to the PointRepresentation to be used when comparing points
        * \param point_representation the PointRepresentation to be used by the k-D tree
        */
      inline void
        setPointRepresentation (const typename KdTree::PointRepresentationConstPtr &point_representation)
      {
        tree_->setPointRepresentation (point_representation);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Obtain the fitness score (e.g., sum of squared distances from the source to the target).
        * \param max_range maximum allowable distance between a point and its correspondent neighbor in the target (default: <double>max)
        */
      inline double getFitnessScore (double max_range = std::numeric_limits<double>::max ());

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Call the registration algorithm which estimates the transformation and returns the transformed source (input) as \a output.
        * \param output the resultant input transfomed point cloud dataset
        */
      inline void align (PointCloudSource &output);

    protected:
      /** \brief The registration method name. */
      std::string reg_name_;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief The number of iterations the internal optimization ran for (used internally). */
      int nr_iterations_;

      /** \brief The maximum number of iterations the internal optimization should run for. */
      int max_iterations_;

      /** \brief The input point cloud dataset target. */
      PointCloudTargetConstPtr target_;

      /** \brief The final transformation matrix estimated by the registration method after N iterations. */
      Eigen::Matrix4f final_transformation_;

      /** \brief The transformation matrix estimated by the registration method. */
      Eigen::Matrix4f transformation_;

      /** \brief The previous transformation matrix estimated by the registration method (used internally). */
      Eigen::Matrix4f previous_transformation_;

      /** \brief The maximum difference between two consecutive transformations in order to consider convergence (user defined). */
      double transformation_epsilon_;

      /** \brief The maximum distance threshold between two correspondent points in source <-> target. If the
        * distance is larger than this threshold, the points will not be ignored in the alignement process.
        */
      double corr_dist_threshold_;

      /** \brief Holds internal convergence state, given user parameters. */
      bool converged_;

      /** \brief The minimum number of correspondences that the algorithm needs before attempting to estimate the transformation. */
      int min_number_correspondences_;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Search for neigbors of a given point based on radius search
        * \param cloud the point cloud dataset to use for neighbor search
        * \param index the index of the query point
        * \param radius of the radiusSearch
        * \param maximum number of nearest neigbors
        * \param indices the resultant vector of indices representing the neighbors
        * \param distances the resultant distances from the query point to the neighbors
        */
      inline bool
        searchForNeighbors (const PointCloudSource &cloud, int index, double radius, int max_nn, std::vector<int> &indices, std::vector<float> &distances)
      {
        return (tree_->radiusSearch (cloud, index, radius, indices, distances, max_nn));
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Abstract class get name method. */
      inline const std::string& getClassName () const { return (reg_name_); }

    private:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Abstract transformation computation method. */
      virtual void computeTransformation (PointCloudSource &output) = 0;

      /** \brief The number of K nearest neighbors to use for each point. */
      int k_;
  };
}

#include "pointcloud_registration/icp/registration_correspondences_check.hpp"

#endif  //#ifndef _REGISTRATION_CORRESPONDENCES_CHECK_H_

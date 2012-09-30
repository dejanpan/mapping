/*
 * TransformationEstimationWDF.h
 *
 *  Created on: Mar 15, 2012
 *      Author: darko490
 */

#ifndef TRANSFORMATION_ESTIMATION_WDF_H
#define TRANSFORMATION_ESTIMATION_WDF_H

#include <pcl/registration/transformation_estimation.h>
//#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/icp.h>
#include "pcl/registration/warp_point_rigid.h"
#include "pcl/registration/warp_point_rigid_6d.h"
//#include "pcl/registration/distances.h"
//#include <pcl/registration/distances.h>
//#include "pcl/impl/point_types.hpp"
#include <unsupported/Eigen/NonLinearOptimization>
#include <iostream>

template <typename PointSource, typename PointTarget>
class TransformationEstimationWDF: public pcl::registration::TransformationEstimation<PointSource, PointTarget> {
	typedef pcl::PointCloud<PointSource> PointCloudSource;
	typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
	typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

	typedef pcl::PointCloud<PointTarget> PointCloudTarget;

	typedef pcl::PointIndices::Ptr PointIndicesPtr;
	typedef pcl::PointIndices::ConstPtr PointIndicesConstPtr;

	float alpha_; //<! weight given to distinct features term in optimization equation
	float beta_; //<! weight used in distance weighting of the points
	std::vector<int> indices_src_dfp_; //!< the vector of indices describing distinctive feature points in source cloud
	std::vector<int> indices_tgt_dfp_; //!< the vector of indices describing distinctive feature points in target cloud
	bool indices_src_dfp_set_; //!< flag indicating if indices_src_dfp_ is set
	bool indices_tgt_dfp_set_; //!< flag indicating if indices_tgt_dfp_ is set
	std::vector<float> weights_dfp_; //!< the vector containing weights of distinctive feature points correspondences
	bool weights_dfp_set_; //!< flag indicating if vector containing weights of distinctive feature points correspondences is set

    /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using LM.
     * \param[in] cloud_src the source point cloud dataset
     * \param[in] cloud_tgt the target point cloud dataset
     * \param[out] transformation_matrix the resultant transformation matrix
     */
	inline void
	estimateRigidTransformation (
	            const pcl::PointCloud<PointSource> &cloud_src,
	            const pcl::PointCloud<PointTarget> &cloud_tgt,
	            Eigen::Matrix4f &transformation_matrix);

    /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using LM.
     * \param[in] cloud_src the source point cloud dataset
     * \param[in] cloud_tgt the target point cloud dataset
     * \param[in] correspondences the vector of correspondences between source and target point cloud
     * \param[out] transformation_matrix the resultant transformation matrix
     */
    inline void
    estimateRigidTransformation (
        const pcl::PointCloud<PointSource> &cloud_src,
        const pcl::PointCloud<PointTarget> &cloud_tgt,
        const pcl::Correspondences &correspondences,
        Eigen::Matrix4f &transformation_matrix);

    /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using LM.
     * \param[in] cloud_src the source point cloud dataset
     * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
     * \param[in] cloud_tgt the target point cloud dataset
     * \param[out] transformation_matrix the resultant transformation matrix
     */
    inline void
    estimateRigidTransformation (
        const pcl::PointCloud<PointSource> &cloud_src,
        const std::vector<int> &indices_src,
        const pcl::PointCloud<PointTarget> &cloud_tgt,
        Eigen::Matrix4f &transformation_matrix);
public:

	TransformationEstimationWDF() {
		// Set default alpha weight
		alpha_ = 0.3;
		indices_src_dfp_set_ = false;
		indices_tgt_dfp_set_ = false;
		weights_dfp_set_ = false;
	};

	virtual ~TransformationEstimationWDF() {};

	inline void
	setAlpha(float alpha_arg);

	inline float
	getAlpha(void);

	inline void
	setBeta(float alpha_arg);

	inline float
	getBeta(void);


	/** \brief Set correspondences between source and target cloud distinctive feature points. This has to be set when using
	 *	this class with pcl::IterativeClosestPoint
	 *
	 *	\param[in] correspondences_dfp the vector of distinctive feature point correspondences between source and target point
	 *	cloud. Source cloud point is stored in pcl::Correspondence::index_query, and corresponding target cloud point in
	 *	pcl::Correspondence::index_match.
	 */
	inline void
	setCorrespondecesDFP ( pcl::Correspondences correspondences_dfp );

	/** \brief Set correspondences between source and target cloud distinctive feature points. This has to be set when using
	 *	this class with pcl::IterativeClosestPoint
	 *
	 *	\param[in] indices_src_dfp_arg the vector of indices describing the distinctive feature points in cloud source
 	 *	\param[in] indices_tgt_dfp_arg the vector of indices describing the correspondences of distinctive feature points
 	 *	\a indices_src_dfp_arg in cloud target
	 */
	inline void
	setCorrespondecesDFP( std::vector<int> &indices_src_dfp_arg, std::vector<int> &indices_tgt_dfp_arg );

	inline void
	setWeightsDFP (std::vector<float> weights_dfp_arg);

	/** \brief Estimate a rigid rotation transformation between a source and a target point cloud using WDF.
	 * \param[in] cloud_src the source point cloud dataset
	 * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
	 * \param[in] indices_src_dfp the vector of indices describing distinctive feature points in \a cloud_src
	 * \param[in] cloud_tgt the target point cloud dataset
	 * \param[in] indices_tgt the vector of indices describing the correspondences of the interest points from
	 * \a indices_src
	 * \param[in] indices_tgt_dfp the vector of indices describing correspondences of distinctive feature points
	 * from \a indices_src_dfp in \a cloud_tgt
	 * \param[in] alpha weight given to distinct features term in optimization equation (range [0,1])
	 * \param[out] transformation_matrix the resultant transformation matrix
	 */
	inline void
	estimateRigidTransformation (
			const pcl::PointCloud<PointSource> &cloud_src,
			const std::vector<int> &indices_src,
			const std::vector<int> &indices_src_dfp,
			const pcl::PointCloud<PointTarget> &cloud_tgt,
			const std::vector<int> &indices_tgt,
			const std::vector<int> &indices_tgt_dfp,
			float alpha_arg,
			Eigen::Matrix4f &transformation_matrix);

	/** \brief Estimate a rigid rotation transformation between a source and a target point cloud using WDF.
	 *
	 * \param[in] cloud_src the source point cloud dataset
	 * \param[in] cloud_tgt the target point cloud dataset
	 * \param[in] correspondences the vector of correspondences between source and target point cloud
	 * \param[in] correspondences_dfp the vector of distinctive feature point correspondences between source and target point cloud
	 * \param[in] alpha weight given to distinct features term in optimization equation (range [0,1])
	 * \param[out] transformation_matrix the resultant transformation matrix
	 */
	inline void
	estimateRigidTransformation (
			const pcl::PointCloud<PointSource> &cloud_src,
			const pcl::PointCloud<PointTarget> &cloud_tgt,
			const pcl::Correspondences &correspondences,
			const pcl::Correspondences &correspondences_dfp,
			float alpha_arg,
			Eigen::Matrix4f &transformation_matrix);

    /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using WDF. This method has to
     * be used in conjunction with methods setCorrespondecesDFP and setAlpha. In this manner it can be combined with
     * pcl::IterativeClosestPoint via setTransformationEstimation method.
     *
     * \param[in] cloud_src the source point cloud dataset
     * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
     * \param[in] cloud_tgt the target point cloud dataset
     * \param[in] indices_tgt the vector of indices describing the correspondences of the interst points from
     * \a indices_src
     * \param[out] transformation_matrix the resultant transformation matrix
     */
    inline void
    estimateRigidTransformation (
        const pcl::PointCloud<PointSource> &cloud_src,
        const std::vector<int> &indices_src,
        const pcl::PointCloud<PointTarget> &cloud_tgt,
        const std::vector<int> &indices_tgt,
        Eigen::Matrix4f &transformation_matrix);

	/** \brief Set the function we use to warp points. Defaults to rigid 6D warp.
	 * \param[in] shared pointer to object that warps points
	 */
	void
	setWarpFunction (const boost::shared_ptr<pcl::WarpPointRigid<PointSource, PointTarget> > &warp_fcn)
	{
		warp_point_ = warp_fcn;
	}

protected:
	// Declare ICP class as friend
	friend class pcl::IterativeClosestPoint<PointSource, PointTarget>;

	/** \brief Compute the distance between a source point and its corresponding target point
	 * \param p_src The source point
	 * \param p_tgt The target point
	 * \return The distance between \a p_src and \a p_tgt
	 *
	 * \note A different distance function can be defined by creating a subclass of TransformationEstimationLM and
	 * overriding this method. (See \a TransformationEstimationPointToPlane)
	 */
	virtual double
	computeDistance (const PointSource &p_src, const PointTarget &p_tgt)
	{
		/*pcl::Vector4fMapConst*/ Eigen::Vector4f s = p_src.getVector4fMap ();
		/*pcl::Vector4fMapConst*/ Eigen::Vector4f t = p_tgt.getVector4fMap ();
		return ( (s - t).norm () );
	}

    virtual double
    computeDistancePointToPlane (const PointSource &p_src, const PointTarget &p_tgt)
    {
      // Compute the point-to-plane distance
      pcl::Vector4fMapConst s = p_src.getVector4fMap ();
      pcl::Vector4fMapConst t = p_tgt.getVector4fMap ();
      pcl::Vector4fMapConst n = p_tgt.getNormalVector4fMap ();
      return ((s - t).dot (n));
    }

	virtual inline double
	computeDistanceWeight ( const double &depth )
	{
//		return (0.8*0.8)/(depth*depth);
		int a = 0.8/depth;
		if (a>1.07) return 0;	// if depth is less than 0.75 discard point
			else return a;
	}

	/** \brief The vector of residual weights. Used internally in the LM loop. */
	std::vector<double> weights_;

	/** \brief Temporary pointer to the source dataset. */
	const PointCloudSource *tmp_src_;

	/** \brief Temporary pointer to the target dataset. */
	const PointCloudTarget  *tmp_tgt_;

	/** \brief Temporary pointer to the source dataset indices. */
	const std::vector<int> *tmp_idx_src_;

	/** \brief Temporary pointer to the target dataset indices. */
	const std::vector<int> *tmp_idx_tgt_;

	/** \brief Temporary pointer to the source dataset distinctive feature point indices. */
	const std::vector<int> *tmp_idx_src_dfp_;

	/** \brief Temporary pointer to the target dataset distinctive feature point indices. */
	const std::vector<int> *tmp_idx_tgt_dfp_;

	/** \brief Temporary pointer to weights of common point pairs. */
	const std::vector<float> *tmp_weights_;

	/** \brief Temporary pointer to weights of distinctive feature point pairs. */
	const std::vector<float> *tmp_dfp_weights_;

	/** \brief The parameterized function used to warp the source to the target. */
	boost::shared_ptr<pcl::WarpPointRigid<PointSource, PointTarget> > warp_point_;

	/** Generic functor for the optimization */
	template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
	struct Functor
	{
		typedef _Scalar Scalar;
		enum {
			InputsAtCompileTime = NX,
			ValuesAtCompileTime = NY
		};
		typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
		typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
		typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

		const int m_inputs, m_values;

		Functor () : m_inputs (InputsAtCompileTime), m_values (ValuesAtCompileTime) {}
		Functor (int inputs, int values, int np, int ndfp) : m_inputs (inputs), m_values (values), number_p(np), number_dfp(ndfp) {}

		int number_p; //!< Number of common points
		int number_dfp; //!< Number of distinctive feature points

		int inputs () const { return m_inputs; }
		int values () const { return m_values; }
	};

	struct OptimizationFunctor : Functor<double>
	{
		using Functor<double>::m_values;
		using Functor<double>::number_p;
		using Functor<double>::number_dfp;

		/** Functor constructor
		 * \param n Number of unknowns to be solved
		 * \param m Number of values
		 * \param estimator_ pointer to the estimator object
		 * \param distance distance computation function pointer
		 */
		OptimizationFunctor (int n, int m, int np, int ndfp, TransformationEstimationWDF<PointSource, PointTarget> *estimator) :
			Functor<double> (n,m, np, ndfp), estimator_ (estimator) {}

		/** Fill fvec from x. For the current state vector x fill the f values
		 * \param x state vector
		 * \param fvec function value scalar
		 * \return 0
		 */
		int operator () (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const;

		TransformationEstimationWDF<PointSource, PointTarget> *estimator_;
	};

	struct OptimizationFunctorWithWeights : Functor<double>
	{
		using Functor<double>::m_values;
		using Functor<double>::number_p;
		using Functor<double>::number_dfp;

		/** Functor constructor
		 * \param n Number of unknowns to be solved
		 * \param m Number of values
		 * \param estimator_ pointer to the estimator object
		 * \param distance distance computation function pointer
		 */
		OptimizationFunctorWithWeights (int n, int m, int np, int ndfp, TransformationEstimationWDF<PointSource, PointTarget> *estimator) :
			Functor<double> (n,m, np, ndfp), estimator_ (estimator) {}

		/** Fill fvec from x. For the current state vector x fill the f values
		 * \param x state vector
		 * \param fvec function value scalar
		 * \return 0
		 */
		int operator () (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const;

		TransformationEstimationWDF<PointSource, PointTarget> *estimator_;
	};
};

#include "../../src/transformation_estimation_wdf.hpp"

#endif /* TRANSFORMATION_ESTIMATION_WDF_H */

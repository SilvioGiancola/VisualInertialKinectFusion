#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_TRANSLATION_H_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_TRANSLATION_H_

#include <pcl/registration/transformation_estimation.h>
#include <pcl/cloud_iterator.h>

namespace pcl
{
  namespace registration
  {
    /** @b TransformationEstimationTranslation implements center of mass Based estimation of
      * the translation aligning the given correspondences.
      *
      * \note The class is templated on the source and target point types as well as on the output scalar of the transformation matrix (i.e., float or double). Default: float.
      * \author Silvio Giancola
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename Scalar = float>
    class TransformationEstimationTranslation : public TransformationEstimation<PointSource, PointTarget, Scalar>
    {
      public:
        typedef boost::shared_ptr<TransformationEstimationTranslation<PointSource, PointTarget, Scalar> > Ptr;
        typedef boost::shared_ptr<const TransformationEstimationTranslation<PointSource, PointTarget, Scalar> > ConstPtr;

        typedef typename TransformationEstimation<PointSource, PointTarget, Scalar>::Matrix4 Matrix4;

        /** \brief Constructor
          */
        TransformationEstimationTranslation () {}

        virtual ~TransformationEstimationTranslation () {}

        /** \brief Estimate the translation of a rigid registration between a source and a target point cloud.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        inline void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            Matrix4 &transformation_matrix) const;

        /** \brief Estimate the translation of a rigid registration between a source and a target point cloud.
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
            Matrix4 &transformation_matrix) const;

        /** \brief Estimate the translation of a rigid registration between a source and a target point cloud.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[in] indices_tgt the vector of indices describing the correspondences of the interst points from \a indices_src
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        inline void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const std::vector<int> &indices_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            const std::vector<int> &indices_tgt,
            Matrix4 &transformation_matrix) const;

        /** \brief Estimate the translation of a rigid registration between a source and a target point cloud.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[in] correspondences the vector of correspondences between source and target point cloud
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            const pcl::Correspondences &correspondences,
            Matrix4 &transformation_matrix) const;

      protected:

        /** \brief Estimate the translation of a rigid registration between a source and a target point cloud
          * \param[in] source_it an iterator over the source point cloud dataset
          * \param[in] target_it an iterator over the target point cloud dataset
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        void
        estimateRigidTransformation (ConstCloudIterator<PointSource>& source_it,
                                     ConstCloudIterator<PointTarget>& target_it,
                                     Matrix4 &transformation_matrix) const;

        /** \brief Obtain a 4x4 rigid transformation matrix from a correlation matrix H = src * tgt'
          * \param[in] cloud_src_demean the input source cloud, demeaned, in Eigen format
          * \param[in] centroid_src the input source centroid, in Eigen format
          * \param[in] cloud_tgt_demean the input target cloud, demeaned, in Eigen format
          * \param[in] centroid_tgt the input target cloud, in Eigen format
          * \param[out] transformation_matrix the resultant 4x4 rigid transformation matrix
          */
        virtual void
        getTransformationFromCorrelation (
            const Eigen::Matrix<Scalar, 4, 1> &centroid_src,
            const Eigen::Matrix<Scalar, 4, 1> &centroid_tgt,
            Matrix4 &transformation_matrix) const;
     };

  }
}

#include <pcl/registration/impl/transformation_estimation_translation.hpp>

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_TRANSLATION_H_ */

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_MODEL_REGISTRATION_TRANSLATION_H_
#define PCL_SAMPLE_CONSENSUS_MODEL_REGISTRATION_TRANSLATION_H_

#include <pcl/sample_consensus/eigen.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/sample_consensus/sac_model_registration_2d.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
#include <map>

namespace pcl
{
  /** \brief SampleConsensusModelRegistrationTranslation defines a model for Point-To-Point registration outlier rejection.
    * \author Silvio Giancola
    * \ingroup sample_consensus
    */
  template <typename PointT>
  class SampleConsensusModelRegistrationTranslation : public SampleConsensusModelRegistration<PointT>
  {
    public:
      using pcl::SampleConsensusModelRegistration<PointT>::model_name_;
      using pcl::SampleConsensusModelRegistration<PointT>::input_;
      using pcl::SampleConsensusModelRegistration<PointT>::target_;
      using pcl::SampleConsensusModelRegistration<PointT>::indices_;
      using pcl::SampleConsensusModelRegistration<PointT>::indices_tgt_;
      using pcl::SampleConsensusModelRegistration<PointT>::error_sqr_dists_;
      using pcl::SampleConsensusModelRegistration<PointT>::correspondences_;
      using pcl::SampleConsensusModelRegistration<PointT>::sample_dist_thresh_;
      using pcl::SampleConsensusModelRegistration<PointT>::computeOriginalIndexMapping;
      using pcl::SampleConsensusModelRegistration<PointT>::estimateRigidTransformationSVD;
      using pcl::SampleConsensusModelRegistration<PointT>::optimizeModelCoefficients;
      using pcl::SampleConsensusModel<PointT>::isModelValid;

      typedef typename SampleConsensusModel<PointT>::PointCloud PointCloud;
      typedef typename SampleConsensusModel<PointT>::PointCloudPtr PointCloudPtr;
      typedef typename SampleConsensusModel<PointT>::PointCloudConstPtr PointCloudConstPtr;

      typedef boost::shared_ptr<SampleConsensusModelRegistrationTranslation> Ptr;
      typedef boost::shared_ptr<const SampleConsensusModelRegistrationTranslation> ConstPtr;

      /** \brief Constructor for base SampleConsensusModelRegistrationTranslation.
        * \param[in] cloud the input point cloud dataset
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelRegistrationTranslation (const PointCloudConstPtr &cloud,
                                        bool random = false)
        : SampleConsensusModelRegistration<PointT> (cloud, random)
      {
        // Call our own setInputCloud
        setInputCloud (cloud);
        model_name_ = "SampleConsensusModelRegistrationTranslation";
        sample_size_ = 1;
        model_size_ = 16;
      }

      /** \brief Constructor for base SampleConsensusModelRegistrationTranslation.
        * \param[in] cloud the input point cloud dataset
        * \param[in] indices a vector of point indices to be used from \a cloud
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelRegistrationTranslation (const PointCloudConstPtr &cloud,
                                        const std::vector<int> &indices,
                                        bool random = false)
        : SampleConsensusModelRegistration<PointT> (cloud, indices, random)
      {
        computeOriginalIndexMapping ();
        computeSampleDistanceThreshold (cloud, indices);
        model_name_ = "SampleConsensusModelRegistrationTranslation";
        sample_size_ = 1;
        model_size_ = 16;
      }

      /** \brief Empty destructor */
      virtual ~SampleConsensusModelRegistrationTranslation () {}

      /** \brief Compute a 4x4 rigid transformation matrix from the samples given
        * \param[in] samples the indices found as good candidates for creating a valid model
        * \param[out] model_coefficients the resultant model coefficients
        */
      bool
      computeModelCoefficients (const std::vector<int> &samples,
                                Eigen::VectorXf &model_coefficients);

      void
      projectPoints (const std::vector<int> &,
                     const Eigen::VectorXf &,
                     PointCloud &, bool = true)
      {
      };

      bool
      doSamplesVerifyModel (const std::set<int> &,
                            const Eigen::VectorXf &,
                            const double)
      {
        return (false);
      }

      /** \brief Return an unique id for this model (SACMODEL_REGISTRATION_TRANSLATION). */
      inline pcl::SacModel
      getModelType () const { return (SACMODEL_REGISTRATION_TRANSLATION); }

  protected:
      using SampleConsensusModel<PointT>::sample_size_;
      using SampleConsensusModel<PointT>::model_size_;

      /** \brief Check if a sample of indices results in a good sample of points
        * indices.
        * \param[in] samples the resultant index samples
        */
      virtual bool
      isSampleGood (const std::vector<int> &samples) const;

      /** \brief Computes an "optimal" sample distance threshold based on the
        * principal directions of the input cloud.
        * \param[in] cloud the const boost shared pointer to a PointCloud message
        */
      inline void
      computeSampleDistanceThreshold (const PointCloudConstPtr &cloud)
      {
        // Compute the principal directions via PCA
        Eigen::Vector4f xyz_centroid;
        Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero ();

        computeMeanAndCovarianceMatrix (*cloud, covariance_matrix, xyz_centroid);

        // Check if the covariance matrix is finite or not.
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j)
            if (!pcl_isfinite (covariance_matrix.coeffRef (i, j)))
              PCL_ERROR ("[pcl::SampleConsensusModelRegistration::computeSampleDistanceThreshold] Covariance matrix has NaN values! Is the input cloud finite?\n");

        Eigen::Vector3f eigen_values;
        pcl::eigen33 (covariance_matrix, eigen_values);

        // Compute the distance threshold for sample selection
        sample_dist_thresh_ = eigen_values.array ().sqrt ().sum () / 3.0;
        sample_dist_thresh_ *= sample_dist_thresh_;
        PCL_DEBUG ("[pcl::SampleConsensusModelRegistration::setInputCloud] Estimated a sample selection distance threshold of: %f\n", sample_dist_thresh_);
      }

      /** \brief Computes an "optimal" sample distance threshold based on the
        * principal directions of the input cloud.
        * \param[in] cloud the const boost shared pointer to a PointCloud message
        * \param indices
        */
      inline void
      computeSampleDistanceThreshold (const PointCloudConstPtr &cloud,
                                      const std::vector<int> &indices)
      {
        // Compute the principal directions via PCA
        Eigen::Vector4f xyz_centroid;
        Eigen::Matrix3f covariance_matrix;
        computeMeanAndCovarianceMatrix (*cloud, indices, covariance_matrix, xyz_centroid);

        // Check if the covariance matrix is finite or not.
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j)
            if (!pcl_isfinite (covariance_matrix.coeffRef (i, j)))
              PCL_ERROR ("[pcl::SampleConsensusModelRegistration::computeSampleDistanceThreshold] Covariance matrix has NaN values! Is the input cloud finite?\n");

        Eigen::Vector3f eigen_values;
        pcl::eigen33 (covariance_matrix, eigen_values);

        // Compute the distance threshold for sample selection
        sample_dist_thresh_ = eigen_values.array ().sqrt ().sum () / 3.0;
        sample_dist_thresh_ *= sample_dist_thresh_;
        PCL_DEBUG ("[pcl::SampleConsensusModelRegistration::setInputCloud] Estimated a sample selection distance threshold of: %f\n", sample_dist_thresh_);
      }

    /** \brief Estimate a rigid transformation between a source and a target point cloud using an SVD closed-form
      * solution of absolute orientation using unit quaternions
      * \param[in] cloud_src the source point cloud dataset
      * \param[in] indices_src the vector of indices describing the points of interest in cloud_src
      * \param[in] cloud_tgt the target point cloud dataset
      * \param[in] indices_tgt the vector of indices describing the correspondences of the interest points from
      * indices_src
      * \param[out] transform the resultant transformation matrix (as model coefficients)
      *
      * This method is an implementation of: Horn, B. “Closed-Form Solution of Absolute Orientation Using Unit Quaternions,” JOSA A, Vol. 4, No. 4, 1987
      */
           void
      estimateRigidTransformation (const pcl::PointCloud<PointT> &cloud_src,
                                      const std::vector<int> &indices_src,
                                      const pcl::PointCloud<PointT> &cloud_tgt,
                                      const std::vector<int> &indices_tgt,
                                      Eigen::VectorXf &transform);

      /** \brief Compute mappings between original indices of the input_/target_ clouds. */
      void
      computeOriginalIndexMapping ()
      {
        if (!indices_tgt_ || !indices_ || indices_->empty () || indices_->size () != indices_tgt_->size ())
          return;
        for (size_t i = 0; i < indices_->size (); ++i)
          correspondences_[(*indices_)[i]] = (*indices_tgt_)[i];
      }
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#include <pcl/sample_consensus/impl/sac_model_registration_translation.hpp>

#endif    // PCL_SAMPLE_CONSENSUS_MODEL_REGISTRATION_TRANSLATION_H_


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

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_REGISTRATION_TRANSLATION_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_REGISTRATION_TRANSLATION_H_

#include <pcl/sample_consensus/sac_model_registration_translation.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelRegistrationTranslation<PointT>::isSampleGood (const std::vector<int> &samples) const
{
    return true;
    /*  using namespace pcl::common;
  using namespace pcl::traits;

  PointT p10 = input_->points[samples[1]] - input_->points[samples[0]];
  PointT p20 = input_->points[samples[2]] - input_->points[samples[0]];
  PointT p21 = input_->points[samples[2]] - input_->points[samples[1]];

  return ((p10.x * p10.x + p10.y * p10.y + p10.z * p10.z) > sample_dist_thresh_ &&
          (p20.x * p20.x + p20.y * p20.y + p20.z * p20.z) > sample_dist_thresh_ &&
          (p21.x * p21.x + p21.y * p21.y + p21.z * p21.z) > sample_dist_thresh_);*/
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelRegistrationTranslation<PointT>::computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXf &model_coefficients)
{
    if (!target_)
    {
        PCL_ERROR ("[pcl::SampleConsensusModelRegistrationTranslation::computeModelCoefficients] No target dataset given!\n");
        return (false);
    }
    //std::cout << samples.size () << std::endl;// Need 1 samples
    if (samples.size () != 1)
        return (false);

    std::vector<int> indices_tgt (1);
    for (int i = 0; i < 1; ++i)
        indices_tgt[i] = correspondences_[samples[i]];

    estimateRigidTransformation (*input_, samples, *target_, indices_tgt, model_coefficients);
    return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelRegistrationTranslation<PointT>::estimateRigidTransformation (
        const pcl::PointCloud<PointT> &cloud_src,
        const std::vector<int> &indices_src,
        const pcl::PointCloud<PointT> &cloud_tgt,
        const std::vector<int> &indices_tgt,
        Eigen::VectorXf &transform)
{
    transform.resize (16);

    Eigen::Matrix4d transformation_matrix;
    transformation_matrix.setIdentity ();


    PointT P_tgt = cloud_tgt.at(indices_tgt.at(0));
    PointT P_src = cloud_src.at(indices_src.at(0));

    transformation_matrix(0,3) = P_tgt.x - P_src.x;
    transformation_matrix(1,3) = P_tgt.y - P_src.y;
    transformation_matrix(2,3) = P_tgt.z - P_src.z;

    // Return the correct transformation
    transform.segment<4> (0).matrix () = transformation_matrix.cast<float> ().row (0);
    transform.segment<4> (4).matrix () = transformation_matrix.cast<float> ().row (1);
    transform.segment<4> (8).matrix () = transformation_matrix.cast<float> ().row (2);
    transform.segment<4> (12).matrix () = transformation_matrix.cast<float> ().row (3);
}


#define PCL_INSTANTIATE_SampleConsensusModelRegistrationTranslation(T) template class PCL_EXPORTS pcl::SampleConsensusModelRegistrationTranslation<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_REGISTRATION_TRANSLATION_H_


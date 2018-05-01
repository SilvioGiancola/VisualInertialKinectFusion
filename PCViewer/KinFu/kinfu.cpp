/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 */

#include <iostream>
#include <algorithm>

#include <pcl/common/time.h>
#include <KinFu/kinfu.h>
#include "internal.h"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>

#ifdef HAVE_OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
#endif

using namespace std;
using namespace pcl::device;
using namespace pcl::gpu;

using Eigen::AngleAxisf;
using Eigen::Array3f;
using Eigen::Vector3i;
using Eigen::Vector3f;

namespace pcl
{
namespace gpu
{
Eigen::Vector3f rodrigues2(const Eigen::Matrix3f& matrix);
}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::gpu::KinfuTracker::KinfuTracker (float Size, int Resolution, int rows, int cols) : _resolution(Resolution), _size(Size), rows_(rows), cols_(cols), global_time_(0), max_icp_distance_(0), integration_metric_threshold_(0.f)
{
    const Vector3f volume_size = Vector3f::Constant (_size);
    const Vector3i volume_resolution(_resolution, _resolution, _resolution);

    tsdf_volume_ = TsdfVolume::Ptr( new TsdfVolume(volume_resolution) );
    tsdf_volume_->setSize(volume_size);

    setDepthIntrinsics (KINFU_DEFAULT_DEPTH_FOCAL_X, KINFU_DEFAULT_DEPTH_FOCAL_Y); // default values, can be overwritten

    init_Rcam_ = Eigen::Matrix3f::Identity ();// * AngleAxisf(-30.f/180*3.1415926, Vector3f::UnitX());
    init_tcam_ = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);

    const int iters[] = {10, 5, 4};
    //   const int iters[] = {5, 3, 2};
    std::copy (iters, iters + LEVELS, icp_iterations_);

    const float default_distThres = 0.10f; //meters
    const float default_angleThres = sin (20.f * 3.14159254f / 180.f);
    const float default_tranc_dist = 0.03f; //meters

    setIcpCorespFilteringParams (default_distThres, default_angleThres);
    tsdf_volume_->setTsdfTruncDist (default_tranc_dist);

    allocateBufffers (rows, cols);

    rmats_.reserve (30000);
    tvecs_.reserve (30000);

    reset ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::KinfuTracker::setDepthIntrinsics (float fx, float fy, float cx, float cy)
{
    fx_ = fx;
    fy_ = fy;
    cx_ = (cx == -1) ? cols_/2-0.5f : cx;
    cy_ = (cy == -1) ? rows_/2-0.5f : cy;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::KinfuTracker::getDepthIntrinsics (float& fx, float& fy, float& cx, float& cy)
{
    fx = fx_;
    fy = fy_;
    cx = cx_;
    cy = cy_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::KinfuTracker::setInitalCameraPose (const Eigen::Affine3f& pose)
{
    init_Rcam_ = pose.rotation ();
    init_tcam_ = pose.translation ();
    reset ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::KinfuTracker::setDepthTruncationForICP (float max_icp_distance)
{
    max_icp_distance_ = max_icp_distance;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::KinfuTracker::setCameraMovementThreshold(float threshold)
{
    integration_metric_threshold_ = threshold;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::KinfuTracker::setIcpCorespFilteringParams (float distThreshold, float sineOfAngle)
{
    distThres_  = distThreshold; //mm
    angleThres_ = sineOfAngle;
}

void
pcl::gpu::KinfuTracker::getMetricsOnNextFrame ()
{
    metrics_ = true;
}

void
pcl::gpu::KinfuTracker::printposes ()
{
    calib_ = true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::gpu::KinfuTracker::cols ()
{
    return (cols_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::gpu::KinfuTracker::rows ()
{
    return (rows_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::KinfuTracker::reset()
{
    if (global_time_)
        cout << "Reset" << endl;

    global_time_ = 0;
    rmats_.clear ();
    tvecs_.clear ();

    rmats_.push_back (init_Rcam_);
    tvecs_.push_back (init_tcam_);

    tsdf_volume_->reset();
    
    if (color_volume_) // color integration mode is enabled
        color_volume_->reset();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::KinfuTracker::allocateBufffers (int rows, int cols)
{    
    depths_curr_.resize (LEVELS);
    vmaps_g_curr_.resize (LEVELS);
    nmaps_g_curr_.resize (LEVELS);

    vmaps_g_prev_.resize (LEVELS);
    nmaps_g_prev_.resize (LEVELS);

    vmaps_curr_.resize (LEVELS);
    nmaps_curr_.resize (LEVELS);

    coresps_.resize (LEVELS);

    for (int i = 0; i < LEVELS; ++i)
    {
        int pyr_rows = rows >> i;
        int pyr_cols = cols >> i;

        depths_curr_[i].create (pyr_rows, pyr_cols);

        vmaps_g_curr_[i].create (pyr_rows*3, pyr_cols);
        nmaps_g_curr_[i].create (pyr_rows*3, pyr_cols);

        vmaps_g_prev_[i].create (pyr_rows*3, pyr_cols);
        nmaps_g_prev_[i].create (pyr_rows*3, pyr_cols);

        vmaps_curr_[i].create (pyr_rows*3, pyr_cols);
        nmaps_curr_[i].create (pyr_rows*3, pyr_cols);

        coresps_[i].create (pyr_rows, pyr_cols);
    }
    depthRawScaled_.create (rows, cols);
    // see estimate tranform for the magic numbers
    gbuf_.create (27, 20*60);
    sumbuf_.create (27);
}

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <KinFu/marching_cubes.h>
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::gpu::KinfuTracker::operator() (const DepthMap& depth_raw,
                                    const PtrStepSz<const unsigned short>& depthCPU,
                                    Eigen::Affine3f *hint)
{  
    device::Intr intr (fx_, fy_, cx_, cy_);
    {
      //  ScopeTime time(">>> Bilateral, pyr-down-all, create-maps-all");
        //depth_raw.copyTo(depths_curr[0]);
        device::bilateralFilter (depth_raw, depths_curr_[0]);

        if (max_icp_distance_ > 0)
            device::truncateDepth(depths_curr_[0], max_icp_distance_);

        for (int i = 1; i < LEVELS; ++i)
            device::pyrDown (depths_curr_[i-1], depths_curr_[i]);

        for (int i = 0; i < LEVELS; ++i)
        {
            device::createVMap (intr(i), depths_curr_[i], vmaps_curr_[i]);
            //device::createNMap(vmaps_curr_[i], nmaps_curr_[i]);
            computeNormalsEigen (vmaps_curr_[i], nmaps_curr_[i]);
        }
        pcl::device::sync ();
    }



    //can't perform more on first frame
    if (global_time_ == 0)
    {
        std::cout  << "here is first time" << std::endl;
        if (hint)
            rotation_initIMU_inv = hint->rotation().matrix().inverse(); // newest stuff 2017
        else
            rotation_initIMU_inv = Matrix3frm::Identity();

        Matrix3frm init_Rcam = rmats_[0]; //  [Ri|ti] - pos of camera, i.e.
        Vector3f   init_tcam = tvecs_[0]; //  transform from camera to global coo space for (i-1)th camera pose

        Mat33&  device_Rcam = device_cast<Mat33> (init_Rcam);
        float3& device_tcam = device_cast<float3>(init_tcam);

        Matrix3frm init_Rcam_inv = init_Rcam.inverse ();
        Mat33&   device_Rcam_inv = device_cast<Mat33> (init_Rcam_inv);
        float3 device_volume_size = device_cast<const float3>(tsdf_volume_->getSize());

        //integrateTsdfVolume(depth_raw, intr, device_volume_size, device_Rcam_inv, device_tcam, tranc_dist, volume_);
        device::integrateTsdfVolume(depth_raw, intr, device_volume_size, device_Rcam_inv, device_tcam, tsdf_volume_->getTsdfTruncDist(), tsdf_volume_->data(), depthRawScaled_);

        for (int i = 0; i < LEVELS; ++i)
            device::tranformMaps (vmaps_curr_[i], nmaps_curr_[i], device_Rcam, device_tcam, vmaps_g_prev_[i], nmaps_g_prev_[i]);

        ++global_time_;
        return (false);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Iterative Closest Point
    Matrix3frm Rprev = rmats_[global_time_ - 1]; //  [Ri|ti] - pos of camera, i.e.
    Vector3f   tprev = tvecs_[global_time_ - 1]; //  tranfrom from camera to global coo space for (i-1)th camera pose
    // Vector3f   tprev2 = (global_time_>1)? tvecs_[global_time_ - 2] : tvecs_[global_time_ - 1]; //  tranfrom from camera to global coo space for (i-2)th camera pose
    Matrix3frm Rprev_inv = Rprev.inverse (); //Rprev.t();

    //Mat33&  device_Rprev     = device_cast<Mat33> (Rprev);
    Mat33&  device_Rprev_inv = device_cast<Mat33> (Rprev_inv);
    float3& device_tprev     = device_cast<float3> (tprev);
    Matrix3frm Rcurr;
    Vector3f tcurr;
    if(hint)
    {
        Rcurr = rotation_initIMU_inv * hint->rotation().matrix(); // 2017 : add rot_ini_IMU_inv has PRE-mult.
        tcurr = tprev;// + (tprev - tprev2)*0.5;//hint->translation().matrix();
        if (calib_)
        {
            Eigen::Quaternionf q = Eigen::Quaternionf(hint->rotation().matrix());
            cout << "Adafruit : 0 0 0 " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << endl;
        }

    }
    else
    {
        if (calib_)
            calib_ = false;
        Rcurr = Rprev; // tranform to global coo for ith camera pose
        tcurr = tprev;
    }

    pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr Model(new pcl::PointCloud<PointXYZ>);


    std::string pathstring;
    if(hint)
        pathstring = "/home/silvio/PointClouds/timing_dev.txt";
    else
        pathstring = "/home/silvio/PointClouds/timing.txt";

    QFile file( QString::fromStdString(pathstring) );
    QTextStream streamError( &file );


    if(metrics_)
    {

        // saving cube
        this->volume().fetchCloudHost(*Model);

        if(hint)
            pcl::io::savePCDFile("/home/silvio/PointClouds/model_dev.pcd", *Model);
        else
            pcl::io::savePCDFile("/home/silvio/PointClouds/model.pcd", *Model);




        // resize point cloud if it doesn't fit
        if (depthCPU.rows != (int)cloud->height || depthCPU.cols != (int)cloud->width)
            cloud = pcl::PointCloud<PointXYZ>::Ptr (new pcl::PointCloud<PointXYZ> (depthCPU.cols, depthCPU.rows));

        // iterate over all depth and rgb values
        for (int y = 0; y < depthCPU.rows; ++y)
        {
            // get pointers to the values in one row
            const unsigned short *depth_row_ptr = depthCPU.ptr(y);
            // iterate over row and store values
            for (int x = 0; x < depthCPU.cols; ++x)
            {
                float u = (x - intr.cx) / intr.fx;
                float v = (y - intr.cy) / intr.fy;

                PointXYZ &point = cloud->at (x, y);

                point.z = depth_row_ptr[x] / 1000.0f;
                point.x = u * point.z;
                point.y = v * point.z;

            }
        }
        cout << "got PC" << endl;



        // build the condition
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.02)));
        // build the filter
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition(range_cond);
        condrem.setInputCloud (cloud);
        condrem.setKeepOrganized(true);
        // apply filter
        condrem.filter (*cloud);



        cloud->sensor_orientation_ = Eigen::Quaternionf(Rcurr);
        cloud->sensor_origin_.head(3) = tcurr;



        if(hint)
        {
            pcl::io::savePCDFile("/home/silvio/PointClouds/cloud_dev.pcd", *cloud);
            saveDistances(cloud, Model, QString::fromStdString("/home/silvio/PointClouds/cloud_dev.txt"));
        }
        else
        {
            pcl::io::savePCDFile("/home/silvio/PointClouds/cloud.pcd", *cloud);
            saveDistances(cloud, Model, QString::fromStdString("/home/silvio/PointClouds/cloud.txt"));
        }

        cloud->is_dense = false;





        file.open( QIODevice::WriteOnly );
        streamError.setRealNumberPrecision(20);

        if(file.isOpen())
            streamError << 0 << '\n';
        else
            cout<< "file not opened" << endl;

    }

    {
        ScopeTime icp_all("icp-all");

        QElapsedTimer t;
        double current_time = 0.0;
       // printf("angleThres_ = %f\n", angleThres_);
      //  printf("distThres_ = %f\n", distThres_);
        for (int level_index = LEVELS-1; level_index>=0; --level_index)
        {
            int iter_num = icp_iterations_[level_index];

            MapArr& vmap_curr = vmaps_curr_[level_index];
            MapArr& nmap_curr = nmaps_curr_[level_index];

            MapArr& vmap_g_curr = vmaps_g_curr_[level_index];//
            MapArr& nmap_g_curr = nmaps_g_curr_[level_index];//

            MapArr& vmap_g_prev = vmaps_g_prev_[level_index];
            MapArr& nmap_g_prev = nmaps_g_prev_[level_index];

            CorespMap& coresp = coresps_[level_index];//

            //  ScopeTime time("single Level");
            for (int iter = 0; iter < iter_num; ++iter)
            {
                //  ScopeTime time_single_iteration("single iteration");

                t.restart();

                Mat33&  device_Rcurr = device_cast<Mat33> (Rcurr);
                float3& device_tcurr = device_cast<float3>(tcurr);

                Eigen::Matrix<double, 6, 6, Eigen::RowMajor> A;
                Eigen::Matrix<double, 6, 1> b;


                {
                  /*  device::tranformMaps(vmap_curr, nmap_curr, device_Rcurr, device_tcurr, vmap_g_curr, nmap_g_curr);
                    findCoresp(vmap_g_curr, nmap_g_curr, device_Rprev_inv, device_tprev, intr(level_index), vmap_g_prev, nmap_g_prev, distThres_, angleThres_, coresp);
                    device::estimateTransform(vmap_g_prev, nmap_g_prev, vmap_g_curr, coresp, gbuf_, sumbuf_, A.data(), b.data());
*/

                    //ScopeTime time("icp-GPU");
                    estimateCombined (device_Rcurr, device_tcurr, vmap_curr, nmap_curr, device_Rprev_inv, device_tprev, intr (level_index),
                                      vmap_g_prev, nmap_g_prev, distThres_, angleThres_, gbuf_, sumbuf_, A.data (), b.data ());


                    for (int l = 0; l < 3; l++)
                        A(l,l) += lambda_;

                }

                {
                    //  ScopeTime time("icp-CPU");
                    //checking nullspace
                    double det = A.determinant ();

                    //cout << A << endl << endl;
                    if (fabs (det) < 1e-15 || pcl_isnan (det))
                    {
                        if (pcl_isnan (det)) cout << "qnan" << endl;


                        if(metrics_)
                            metrics_ = false;
                        //    reset ();
                        return (false);
                    }
                    //float maxc = A.maxCoeff();

                    Eigen::Matrix<float, 6, 1> result = A.llt ().solve (b).cast<float>();
                    //Eigen::Matrix<float, 6, 1> result = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

                    float alpha = result (0);
                    float beta  = result (1);
                    float gamma = result (2);

                    Eigen::Matrix3f Rinc = (Eigen::Matrix3f)AngleAxisf (gamma, Vector3f::UnitZ ()) * AngleAxisf (beta, Vector3f::UnitY ()) * AngleAxisf (alpha, Vector3f::UnitX ());
                    Vector3f tinc = result.tail<3> ();

                    //compose
                    tcurr = Rinc * tcurr + tinc;
                    Rcurr = Rinc * Rcurr;


                    // save in file
                    current_time = current_time + t.nsecsElapsed();

                    if(metrics_)
                    {
                        if(file.isOpen())   streamError << current_time << '\n';
                        else                cout<< "file not opened" << endl;


                        cloud->sensor_orientation_ = Eigen::Quaternionf(Rcurr);
                        cloud->sensor_origin_.head(3) = tcurr;


                        string path ;
                        if(hint)    path = "/home/silvio/PointClouds/cloud_dev" + std::to_string(level_index) + "_" + std::to_string(iter);
                        else        path = "/home/silvio/PointClouds/cloud" + std::to_string(level_index) + "_" + std::to_string(iter);


                        pcl::io::savePCDFile(path + ".pcd", *cloud);
                        saveDistances(cloud, Model, QString::fromStdString(path + ".txt"));

                    }
                }
            }
        }
    }


    file.close();


    if (calib_)
    {
        Eigen::Quaternionf q = Eigen::Quaternionf(Rcurr);
        cout << "Registration : " << tcurr[0] << " " << tcurr[1] << " " << tcurr[2] << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << endl;
        calib_ = false;
    }

    if(metrics_)
        metrics_ = false;


    //save tranform
    rmats_.push_back (Rcurr);
    tvecs_.push_back (tcurr);


    //    Matrix3frm Rprev = rmats_[global_time_ - 1];
    //    Vector3f   tprev = tvecs_[global_time_ - 1];

    //    Matrix3frm Rcurr = rmats_.back();
    //    Vector3f   tcurr = tvecs_.back();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Integration check - We do not integrate volume if camera does not move.
    float rnorm = rodrigues2(Rcurr.inverse() * Rprev).norm();
    float tnorm = (tcurr - tprev).norm();
    const float alpha = 1.f;
    bool integrate = (rnorm + alpha * tnorm)/2 >= integration_metric_threshold_;


    ///////////////////////////////////////////////////////////////////////////////////////////
    // Volume integration
    float3 device_volume_size = device_cast<const float3> (tsdf_volume_->getSize());

    Matrix3frm Rcurr_inv = Rcurr.inverse ();
    Mat33&  device_Rcurr_inv = device_cast<Mat33> (Rcurr_inv);
    float3& device_tcurr = device_cast<float3> (tcurr);
    if (integrate)
    {
       // ScopeTime time("tsdf");
        //integrateTsdfVolume(depth_raw, intr, device_volume_size, device_Rcurr_inv, device_tcurr, tranc_dist, volume_);
        integrateTsdfVolume (depth_raw, intr, device_volume_size, device_Rcurr_inv, device_tcurr, tsdf_volume_->getTsdfTruncDist(), tsdf_volume_->data(), depthRawScaled_);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Ray casting
    Mat33& device_Rcurr = device_cast<Mat33> (Rcurr);
    {
       // ScopeTime time("ray-cast-all");
        raycast (intr, device_Rcurr, device_tcurr, tsdf_volume_->getTsdfTruncDist(), device_volume_size, tsdf_volume_->data(), vmaps_g_prev_[0], nmaps_g_prev_[0]);
        for (int i = 1; i < LEVELS; ++i)
        {
            resizeVMap (vmaps_g_prev_[i-1], vmaps_g_prev_[i]);
            resizeNMap (nmaps_g_prev_[i-1], nmaps_g_prev_[i]);
        }
        pcl::device::sync ();
    }

    ++global_time_;
    return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<double> pcl::gpu::KinfuTracker::saveDistances(PointCloud<PointXYZ>::Ptr Input, PointCloud<PointXYZ>::Ptr Target, QString path)
{

    PointCloud<PointXYZ>::Ptr Target_rot(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr Input_rot(new PointCloud<PointXYZ>);

    pcl::transformPointCloud(*Target, *Target_rot, Target->sensor_origin_.head(3), Target->sensor_orientation_);
    pcl::transformPointCloud(*Input, *Input_rot, Input->sensor_origin_.head(3), Input->sensor_orientation_);

    std::vector<int> indexes;
    pcl::removeNaNFromPointCloud(*Target_rot, *Target_rot, indexes);
    pcl::removeNaNFromPointCloud(*Input_rot, *Input_rot, indexes);

    pcl::search::KdTree<PointXYZ> search;

    search.setInputCloud(Target_rot);

    std::vector<double> tot_dist;
    for (int i = 0; i < Input_rot->size(); i++)
    {
        // Return the correct index in the cloud instead of the index on the screen
        std::vector<int> indices (1);
        std::vector<float> distances (1);

        // Because VTK/OpenGL stores data without NaN, we lose the 1-1 correspondence, so we must search for the real point
        search.nearestKSearch(Input_rot->points[i], 1, indices, distances);


        tot_dist.push_back( std::sqrt(distances[0]));
    }


    // save in file
    if (!path.isEmpty())
    {
        QFile file( path );
        file.open( QIODevice::WriteOnly );

        if (file.isOpen())
        {
            QTextStream streamError( &file );
            streamError.setRealNumberPrecision(20);
            for(int i=0; i<tot_dist.size(); i++)
                streamError << tot_dist.at(i) << '\n';
        }
        else
            std::cout << "Error: file is not opened" <<std::endl;
        file.close();
    }


    return tot_dist;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Affine3f
pcl::gpu::KinfuTracker::getCameraPose (int time) const
{
    if (time > (int)rmats_.size () || time < 0)
        time = rmats_.size () - 1;

    Eigen::Affine3f aff;
    aff.linear () = rmats_[time];
    aff.translation () = tvecs_[time];
    return (aff);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

size_t
pcl::gpu::KinfuTracker::getNumberOfPoses () const
{
    return rmats_.size();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const TsdfVolume& 
pcl::gpu::KinfuTracker::volume() const 
{ 
    return *tsdf_volume_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TsdfVolume& 
pcl::gpu::KinfuTracker::volume()
{
    return *tsdf_volume_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const ColorVolume& 
pcl::gpu::KinfuTracker::colorVolume() const
{
    return *color_volume_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ColorVolume& 
pcl::gpu::KinfuTracker::colorVolume()
{
    return *color_volume_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::KinfuTracker::getImage (View& view) const
{
    //Eigen::Vector3f light_source_pose = tsdf_volume_->getSize() * (-3.f);
    Eigen::Vector3f light_source_pose = tvecs_[tvecs_.size () - 1];

    device::LightSource light;
    light.number = 1;
    light.pos[0] = device_cast<const float3>(light_source_pose);

    view.create (rows_, cols_);
    generateImage (vmaps_g_prev_[0], nmaps_g_prev_[0], light, view);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::KinfuTracker::getLastFrameCloud (DeviceArray2D<PointType>& cloud) const
{
    cloud.create (rows_, cols_);
    DeviceArray2D<float4>& c = (DeviceArray2D<float4>&)cloud;
    device::convert (vmaps_g_prev_[0], c);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::KinfuTracker::getLastFrameNormals (DeviceArray2D<NormalType>& normals) const
{
    normals.create (rows_, cols_);
    DeviceArray2D<float8>& n = (DeviceArray2D<float8>&)normals;
    device::convert (nmaps_g_prev_[0], n);
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::KinfuTracker::initColorIntegration(int max_weight)
{     
    color_volume_ = pcl::gpu::ColorVolume::Ptr( new ColorVolume(*tsdf_volume_, max_weight) );
}


bool
pcl::gpu::KinfuTracker::operator() (const DepthMap& depth, const View& colors)
{
  bool res = true;//(*this)(depth);

  if (res && color_volume_)
  {
    const float3 device_volume_size = device_cast<const float3> (tsdf_volume_->getSize());
    device::Intr intr(fx_, fy_, cx_, cy_);

    Matrix3frm R_inv = rmats_.back().inverse();
    Vector3f   t     = tvecs_.back();

    Mat33&  device_Rcurr_inv = device_cast<Mat33> (R_inv);
    float3& device_tcurr = device_cast<float3> (t);

    device::updateColorVolume(intr, tsdf_volume_->getTsdfTruncDist(), device_Rcurr_inv, device_tcurr, vmaps_g_prev_[0],
        colors, device_volume_size, color_volume_->data(), color_volume_->getMaxWeight());
  }

  return res;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
namespace gpu
{
PCL_EXPORTS void
paint3DView(const KinfuTracker::View& rgb24, KinfuTracker::View& view, float colors_weight = 0.5f)
{
    device::paint3DView(rgb24, view, colors_weight);
}

PCL_EXPORTS void
mergePointNormal(const DeviceArray<PointXYZ>& cloud, const DeviceArray<Normal>& normals, DeviceArray<PointNormal>& output)
{
    const size_t size = min(cloud.size(), normals.size());
    output.create(size);

    const DeviceArray<float4>& c = (const DeviceArray<float4>&)cloud;
    const DeviceArray<float8>& n = (const DeviceArray<float8>&)normals;
    const DeviceArray<float12>& o = (const DeviceArray<float12>&)output;
    device::mergePointNormal(c, n, o);
}

Eigen::Vector3f rodrigues2(const Eigen::Matrix3f& matrix)
{
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(matrix, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::Matrix3f R = svd.matrixU() * svd.matrixV().transpose();

    double rx = R(2, 1) - R(1, 2);
    double ry = R(0, 2) - R(2, 0);
    double rz = R(1, 0) - R(0, 1);

    double s = sqrt((rx*rx + ry*ry + rz*rz)*0.25);
    double c = (R.trace() - 1) * 0.5;
    c = c > 1. ? 1. : c < -1. ? -1. : c;

    double theta = acos(c);

    if( s < 1e-5 )
    {
        double t;

        if( c > 0 )
            rx = ry = rz = 0;
        else
        {
            t = (R(0, 0) + 1)*0.5;
            rx = sqrt( std::max(t, 0.0) );
            t = (R(1, 1) + 1)*0.5;
            ry = sqrt( std::max(t, 0.0) ) * (R(0, 1) < 0 ? -1.0 : 1.0);
            t = (R(2, 2) + 1)*0.5;
            rz = sqrt( std::max(t, 0.0) ) * (R(0, 2) < 0 ? -1.0 : 1.0);

            if( fabs(rx) < fabs(ry) && fabs(rx) < fabs(rz) && (R(1, 2) > 0) != (ry*rz > 0) )
                rz = -rz;
            theta /= sqrt(rx*rx + ry*ry + rz*rz);
            rx *= theta;
            ry *= theta;
            rz *= theta;
        }
    }
    else
    {
        double vth = 1/(2*s);
        vth *= theta;
        rx *= vth; ry *= vth; rz *= vth;
    }
    return Eigen::Vector3d(rx, ry, rz).cast<float>();
}
}
}

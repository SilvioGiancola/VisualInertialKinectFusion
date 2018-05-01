/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


#include <iostream>
#include <signal.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

#include <Devices/adafruit_uart.h>


#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_rejection_distance.h>

#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/time.h>


#include <QFile>
#include <QTextStream>
#include <QTime>
#include <QMutex>

#include "define.h"

using namespace std;



uint XStep = 150;
uint YStep = 70;
uint Size = 30;


libfreenect2::Freenect2 freenect2;
libfreenect2::Freenect2Device *dev;
libfreenect2::SyncMultiFrameListener *listener;
libfreenect2::Registration *registration;
libfreenect2::PacketPipeline *pipeline;

//Adafruit_UART * myada;


pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));




QMutex myMux;

//std::vector<

#include <opencv2/opencv.hpp>

# include "opencv2/core/core.hpp"
# include "opencv2/features2d/features2d.hpp"
# include "opencv2/highgui/highgui.hpp"
# include "opencv2/nonfree/features2d.hpp"


libfreenect2::Frame *frame_rgb_1, *frame_depth_1, *frame_undistorted_1, *frame_registered_1, *frame_bigdepth_1;
libfreenect2::Frame *frame_rgb_2, *frame_depth_2, *frame_undistorted_2, *frame_registered_2, *frame_bigdepth_2;


cv::Mat mat_rgb_1, mat_depth_1, mat_undistorted_1, mat_registered_1, mat_bigdepth_1;
cv::Mat mat_rgb_2, mat_depth_2, mat_undistorted_2, mat_registered_2, mat_bigdepth_2;


PointCloudT::Ptr KinectPointCloud1(new PointCloudT);
PointCloudT::Ptr KinectPointCloud2(new PointCloudT);
PointCloudT::Ptr KinectKeyPoints1(new PointCloudT);
PointCloudT::Ptr KinectKeyPoints2(new PointCloudT);

pcl::CorrespondencesPtr corres(new pcl::Correspondences);

void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

    if (event.getButton () == pcl::visualization::MouseEvent::LeftButton && event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
        std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;


        if (event.getY() < YStep+10)
        {
            if (event.getX() < XStep*1)
            {
                // Acquire Frames
                libfreenect2::FrameMap frames;
                listener->waitForNewFrame(frames);

                frame_rgb_1 = frames[libfreenect2::Frame::Color];
                frame_depth_1 = frames[libfreenect2::Frame::Depth];

                // Undistort and register frames
                frame_undistorted_1 = new libfreenect2::Frame(512, 424, 4);
                frame_registered_1 = new libfreenect2::Frame(512, 424, 4);
                frame_bigdepth_1 = new libfreenect2::Frame(1920, 1082, 4);
                registration->apply(frame_rgb_1, frame_depth_1, frame_undistorted_1, frame_registered_1, true, frame_bigdepth_1);

                cv::Mat(frame_depth_1->height, frame_depth_1->width, CV_32FC1, frame_depth_1->data).copyTo(mat_depth_1);
                cv::Mat(frame_rgb_1->height, frame_rgb_1->width, CV_8UC4, frame_rgb_1->data).copyTo(mat_rgb_1);
                //                cv::imshow("depth", mat_depth_1/4500.0f);
                //                cv::imshow("rgb", mat_rgb_1);

                cv::Mat(frame_undistorted_1->height, frame_undistorted_1->width, CV_32FC1, frame_undistorted_1->data).copyTo(mat_undistorted_1);
                cv::Mat(frame_registered_1->height, frame_registered_1->width, CV_8UC4, frame_registered_1->data).copyTo(mat_registered_1);
                cv::Mat(frame_bigdepth_1->height, frame_bigdepth_1->width, CV_32FC1, frame_bigdepth_1->data).copyTo(mat_bigdepth_1);
                //                cv::imshow("frame_undistorted_1", mat_undistorted_1/4500.0f);
                //                cv::imshow("registered", mat_registered_1);
                //                cv::imshow("bigdepth", mat_bigdepth_1/4500.0f);

                //                cv::waitKey();

                listener->release(frames);


                myMux.lock();
                // Initialize my Point Cloud
                KinectPointCloud1.reset(new PointCloudT());
                KinectPointCloud1->resize(frame_undistorted_1->width * frame_undistorted_1->height); // set the memory size to allocate
                KinectPointCloud1->height = frame_undistorted_1->height;        // set the height
                KinectPointCloud1->width = frame_undistorted_1->width;          // set the width
                KinectPointCloud1->is_dense = false;                   // Kinect V2 returns organized and not dense point clouds
                // KinectPointCloud1->sensor_orientation_ = myada->returnPose();      // Set the rotation of the Kinect


                for (unsigned int i = 0; i < frame_undistorted_1->height ;i++)
                    for (unsigned int j = 0; j < frame_undistorted_1->width ;j++)
                        registration->getPointXYZRGB(frame_undistorted_1,
                                                     frame_registered_1,
                                                     i,j,
                                                     KinectPointCloud1->at(j,i).x,
                                                     KinectPointCloud1->at(j,i).y,
                                                     KinectPointCloud1->at(j,i).z,
                                                     KinectPointCloud1->at(j,i).rgb);
                myMux.unlock();


            }

            else if (event.getX() < XStep*2)
            {
                // Acquire Frames
                libfreenect2::FrameMap frames;
                listener->waitForNewFrame(frames);

                frame_rgb_2 = frames[libfreenect2::Frame::Color];
                frame_depth_2 = frames[libfreenect2::Frame::Depth];

                // Undistort and register frames
                frame_undistorted_2 = new libfreenect2::Frame(512, 424, 4);
                frame_registered_2 = new libfreenect2::Frame(512, 424, 4);
                frame_bigdepth_2 = new libfreenect2::Frame(1920, 1082, 4);
                registration->apply(frame_rgb_2, frame_depth_2, frame_undistorted_2, frame_registered_2, true, frame_bigdepth_2);

                //  std::cout << frame_bigdepth_2->width << "x" << frame_bigdepth_2->height << std::endl;

                cv::Mat(frame_depth_2->height, frame_depth_2->width, CV_32FC1, frame_depth_2->data).copyTo(mat_depth_2);
                cv::Mat(frame_rgb_2->height, frame_rgb_2->width, CV_8UC4, frame_rgb_2->data).copyTo(mat_rgb_2);

                cv::Mat(frame_undistorted_2->height, frame_undistorted_2->width, CV_32FC1, frame_undistorted_2->data).copyTo(mat_undistorted_2);
                cv::Mat(frame_registered_2->height, frame_registered_2->width, CV_8UC4, frame_registered_2->data).copyTo(mat_registered_2);
                cv::Mat(frame_bigdepth_2->height, frame_bigdepth_2->width, CV_32FC1, frame_bigdepth_2->data).copyTo(mat_bigdepth_2);


                listener->release(frames);


                myMux.lock();
                // Initialize my Point Cloud
                KinectPointCloud2.reset(new PointCloudT());
                KinectPointCloud2->resize(frame_undistorted_2->width * frame_undistorted_2->height); // set the memory size to allocate
                KinectPointCloud2->height = frame_undistorted_2->height;        // set the height
                KinectPointCloud2->width = frame_undistorted_2->width;          // set the width
                KinectPointCloud2->is_dense = false;                   // Kinect V2 returns organized and not dense point clouds
                // KinectPointCloud1->sensor_orientation_ = myada->returnPose();      // Set the rotation of the Kinect


                for (unsigned int i = 0; i < frame_undistorted_2->height ;i++)
                    for (unsigned int j = 0; j < frame_undistorted_2->width ;j++)
                        registration->getPointXYZRGB(frame_undistorted_2,
                                                     frame_registered_2,
                                                     i,j,
                                                     KinectPointCloud2->at(j,i).x,
                                                     KinectPointCloud2->at(j,i).y,
                                                     KinectPointCloud2->at(j,i).z,
                                                     KinectPointCloud2->at(j,i).rgb);
                myMux.unlock();
            }

            else if (event.getX() < XStep*3)
            {

                //-- Step 0: Convert RGB in Gray Scale
                cv::Mat mat_gray_1, mat_gray_2;
                cvtColor(mat_registered_1, mat_gray_1, CV_BGR2GRAY);
                cvtColor(mat_registered_2, mat_gray_2, CV_BGR2GRAY);


                //-- Step 1: Detect the keypoints using SURF Detector
                int minHessian = 1000;

                cv::SurfFeatureDetector detector( minHessian );

                std::vector<cv::KeyPoint> keypoints_1, keypoints_2;

                detector.detect( mat_gray_1, keypoints_1 );
                detector.detect( mat_gray_2, keypoints_2 );
                KinectKeyPoints1->resize(keypoints_1.size());
                KinectKeyPoints2->resize(keypoints_2.size());



                for (int i = 0; i < keypoints_1.size(); i++)
                    registration->getPointXYZRGB(frame_undistorted_1,
                                                 frame_registered_1,
                                                 floor(keypoints_1.at(i).pt.y + 0.5),
                                                 floor(keypoints_1.at(i).pt.x + 0.5),
                                                 KinectKeyPoints1->at(i).x,
                                                 KinectKeyPoints1->at(i).y,
                                                 KinectKeyPoints1->at(i).z,
                                                 KinectKeyPoints1->at(i).rgb);

                for (int i = 0; i < keypoints_2.size(); i++)
                    registration->getPointXYZRGB(frame_undistorted_2,
                                                 frame_registered_2,
                                                 floor(keypoints_2.at(i).pt.x + 0.5),
                                                 floor(keypoints_2.at(i).pt.y + 0.5),
                                                 KinectKeyPoints2->at(i).x,
                                                 KinectKeyPoints2->at(i).y,
                                                 KinectKeyPoints2->at(i).z,
                                                 KinectKeyPoints2->at(i).rgb);



                //-- Step 2: Calculate descriptors (feature vectors)
                cv::SurfDescriptorExtractor extractor;

                cv::Mat descriptors_1, descriptors_2;

                extractor.compute( mat_gray_1, keypoints_1, descriptors_1 );
                extractor.compute( mat_gray_2, keypoints_2, descriptors_2 );



                //-- Step 3: Matching descriptor vectors using FLANN matcher
                cv::FlannBasedMatcher matcher;
                std::vector< cv::DMatch > matches;
                matcher.match( descriptors_1, descriptors_2, matches );

                double max_dist = 0; double min_dist = 100;



                //-- Quick calculation of max and min distances between keypoints
                for( int i = 0; i < descriptors_1.rows; i++ )
                {
                    double dist = matches[i].distance;
                    if( dist < min_dist )   min_dist = dist;
                    if( dist > max_dist )   max_dist = dist;
                    corres->push_back(pcl::Correspondence(matches[i].queryIdx, matches[i].trainIdx, dist));
                }

                printf("-- Max dist : %f \n", max_dist );
                printf("-- Min dist : %f \n", min_dist );


                int i = 0;
                std::cout << "Point " << i << std::endl;
                std::cout << "X = " << keypoints_1.at(i).pt.x << std::endl;
                std::cout << "Y = " << keypoints_1.at(i).pt.y << std::endl;

                std::cout << "Correspondences (before): " << corres->size() << std::endl;
                pcl::registration::CorrespondenceRejectorDistance::Ptr cor_rej_dist (new pcl::registration::CorrespondenceRejectorDistance);
                cor_rej_dist->setMaximumDistance(0.1);


                cor_rej_dist->setInputSource<PointT>(KinectKeyPoints1);
                cor_rej_dist->setInputTarget<PointT>(KinectKeyPoints2);

                cor_rej_dist->setInputCorrespondences (corres);
                cor_rej_dist->getCorrespondences (*corres);
                std::cout << "Correspondences (after) : " << corres->size() << std::endl;




                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "KinectKeyPoints1");

                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "KinectKeyPoints2");



                if (viewer->contains("corres"))
                    viewer->removeCorrespondences("corres");
                viewer->addCorrespondences<PointT>(KinectKeyPoints1, KinectKeyPoints2, *corres, "corres");



                pcl::SampleConsensusModelRegistration<PointT>::Ptr model;
                std::vector<int> inliers;
                Eigen::VectorXf model_coefficients;
                Eigen::Matrix4f Transformation;


                std::vector<int> source_indices(corres->size());
                std::vector<int> target_indices(corres->size());
                for (int i = 0; i < (int)corres->size(); ++i)
                {
                    source_indices[i] = corres->at(i).index_query;
                    target_indices[i] = corres->at(i).index_match;
                }



                {
                    pcl::ScopeTime t("ransac-main");

                    std::cout << "using SVD" << std::endl;
                    model.reset(new pcl::SampleConsensusModelRegistration<PointT>(KinectKeyPoints1, source_indices ));

                    model->setInputTarget(KinectKeyPoints2, target_indices);


                    double inlierThreshold = 0.1;
                    pcl::RandomSampleConsensus<PointT> sac(model, inlierThreshold);


                    sac.setMaxIterations(100000);
                    sac.computeModel();


                    sac.getInliers(inliers);
                    sac.getModelCoefficients (model_coefficients);

                    std::cout << "sac.getProbability() : " << sac.getProbability() << std::endl;


                    if (true) // refine
                    {
                     //  pcl::SampleConsensusModelRegistration<PointT>::Ptr newmodel = boost::static_pointer_cast<pcl::SampleConsensusModelRegistration<PointT> >(model);
                        pcl::ScopeTime t("ransac-refine");
                        // Refine Model
                        double refineModelSigma = 3.0;
                        double refineModelIterations = 10;

                        double inlier_distance_threshold_sqr = inlierThreshold * inlierThreshold;
                        double error_threshold = inlierThreshold;
                        double sigma_sqr = refineModelSigma * refineModelSigma;
                        int refine_iterations = 0;
                        bool inlier_changed = false, oscillating = false;
                        std::vector<int> new_inliers, prev_inliers = inliers;
                        std::vector<size_t> inliers_sizes;
                        do
                        {
                            // Optimize the model coefficients
                            //  model.reset(new pcl::SampleConsensusModelRegistration<PointT>(KP_PC_rot, source_indices ));

                            model->optimizeModelCoefficients (prev_inliers, model_coefficients, model_coefficients);
                            inliers_sizes.push_back (prev_inliers.size ());

                            // Select the new inliers based on the optimized coefficients and new threshold
                            model->selectWithinDistance (model_coefficients, error_threshold, new_inliers);

                            std::cout << QString("RANSAC refineModel: Number of inliers found (before/after): %1/%2, with an error threshold of %3.")
                                         .arg((int)prev_inliers.size ()).arg((int)new_inliers.size ()).arg(error_threshold).toStdString() << std::endl;

                            if (new_inliers.empty ())
                            {
                                ++refine_iterations;
                                if (refine_iterations >= refineModelIterations)
                                {
                                    break;
                                }
                                continue;
                            }

                            // Estimate the variance and the new threshold
                            double variance = model->computeVariance ();
                            error_threshold =  std::sqrt (std::min (inlier_distance_threshold_sqr, sigma_sqr * variance));

                            std::cout << QString("RANSAC refineModel: New estimated error threshold: %1 (variance=%2) on iteration %3 out of %4.")
                                         .arg(error_threshold).arg(variance).arg(refine_iterations).arg(refineModelIterations).toStdString() << std::endl;

                            //   UDEBUG ("RANSAC refineModel: New estimated error threshold: %f (variance=%f) on iteration %d out of %d.",
                            //        error_threshold, variance, refine_iterations, refineModelIterations);
                            inlier_changed = false;
                            std::swap (prev_inliers, new_inliers);

                            // If the number of inliers changed, then we are still optimizing
                            if (new_inliers.size () != prev_inliers.size ())
                            {
                                // Check if the number of inliers is oscillating in between two values
                                if (inliers_sizes.size () >= 4)
                                {
                                    if (inliers_sizes[inliers_sizes.size () - 1] == inliers_sizes[inliers_sizes.size () - 3] &&
                                            inliers_sizes[inliers_sizes.size () - 2] == inliers_sizes[inliers_sizes.size () - 4])
                                    {
                                        oscillating = true;
                                        break;
                                    }
                                }
                                inlier_changed = true;
                                continue;
                            }

                            // Check the values of the inlier set
                            for (size_t i = 0; i < prev_inliers.size (); ++i)
                            {
                                // If the value of the inliers changed, then we are still optimizing
                                if (prev_inliers[i] != new_inliers[i])
                                {
                                    inlier_changed = true;
                                    break;
                                }
                            }
                        }
                        while (inlier_changed && ++refine_iterations < refineModelIterations);

                        // If the new set of inliers is empty, we didn't do a good job refining
                        if (new_inliers.empty ())
                            std::cout << "RANSAC refineModel: Refinement failed: got an empty set of inliers!" << std::endl;


                        if (oscillating)
                            std::cout << "RANSAC refineModel: Detected oscillations in the model refinement." << std::endl;


                        std::swap (inliers, new_inliers);
                    }





                    Transformation.row (0) = model_coefficients.segment<4>(0);
                    Transformation.row (1) = model_coefficients.segment<4>(4);
                    Transformation.row (2) = model_coefficients.segment<4>(8);
                    Transformation.row (3) = model_coefficients.segment<4>(12);


                    KinectKeyPoints1->sensor_orientation_ = Eigen::Quaternionf(Transformation.block<3,3>(0,0));
                    KinectKeyPoints1->sensor_origin_ = Transformation.block<4,1>(0,3);

                    KinectPointCloud1->sensor_orientation_ = Eigen::Quaternionf(Transformation.block<3,3>(0,0));
                    KinectPointCloud1->sensor_origin_ = Transformation.block<4,1>(0,3);

                    std::cout << Transformation << std::endl << "  -> in " << t.getTime() << " ms" << std::endl;
                }




                // registration->getPointXYZRGB();


                // -- Draw keypoints
                /*     cv::Mat mat_gray_keypoints_1, mat_gray_keypoints_2;

                  cv::drawKeypoints( mat_gray_1, keypoints_1, mat_gray_keypoints_1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
                  cv::drawKeypoints( mat_gray_2, keypoints_2, mat_gray_keypoints_2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );

               // -- Show detected (drawn) keypoints
                  cv::imshow("Keypoints 1", mat_gray_keypoints_1 );
                  cv::imshow("Keypoints 2", mat_gray_keypoints_2 );

                 cv::waitKey(0);*/

            }
        }
    }
}

bool loadMatrix(std::string filename, Eigen::Matrix4f& m)
{
    std::ifstream input(filename.c_str());
    if (input.fail())
    {
        std::cerr << "ERROR. Cannot find file '" << filename << "'." << std::endl;
        m = Eigen::Matrix4f(0,0);
        return false;
    }
    std::string line;
    float d;

    std::vector<float> v;
    int n_rows = 0;
    while (getline(input, line))
    {
        ++n_rows;
        std::stringstream input_line(line);
        while (!input_line.eof())
        {
            input_line >> d;
            v.push_back(d);
        }
    }
    input.close();

    int n_cols = v.size()/n_rows;
    m = Eigen::Matrix4f(n_rows,n_cols);

    for (int i=0; i<n_rows; i++)
        for (int j=0; j<n_cols; j++)
            m(i,j) = v[i*n_cols + j];


}

int init()
{

    // Freenect2
    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    pipeline = new libfreenect2::OpenGLPacketPipeline();

    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return true;
    }

    std::string _serial = freenect2.getDefaultDeviceSerialNumber();
    dev = freenect2.openDevice(_serial, pipeline);

    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);
    dev->start();

    // Color
    registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());




    // Adafruit
    //   myada = new Adafruit_UART();

    //  myada->Open();
    //  myada->Init();



    Eigen::Matrix4f _PoseAdaOnKin;
    loadMatrix("../PCViewer/AdaOnKin.txt", _PoseAdaOnKin);
    //  myada->setCalibPose(Eigen::Quaternionf(_PoseAdaOnKin.block<3,3>(0,0)););




    // Viewer
    viewer->setBackgroundColor (0.5, 0.5, 0.5);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setCameraPosition(0.5,0.5, 5, // mi posiziono dietro ad un Kinect
                              0.5,0.5,0.5, // guardo un punto centrale
                              0,1,0);   // orientato con la y verso l'alto


    viewer->addText("CONN.",  XStep*0, YStep*0 + 10, Size, 0.3, 0.3, 0.3, "CONNECT");
    viewer->addText("PLAY",   XStep*1, YStep*0 + 10, Size, 0.3, 0.3, 0.3, "PLAY");
    viewer->addText("SAVE",   XStep*2, YStep*0 + 10, Size, 0.3, 0.3, 0.3, "SAVE");

    viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);


    viewer->addText("FPS",XStep,0,10,1,1,1,"FPS");


    return false;
}

int main(int argc, char *argv[])
{    

    if (init())
    {
        std::cout << "error in init" << std::endl;
        return true;
    }


    QTime t;
    t.start();

    while (!viewer->wasStopped ())
    {
        t.restart();




        viewer->updateText(QString("FPS = %1 Hz").arg(1000.0f/t.elapsed()).toStdString(),XStep,0,10,1,1,1,"FPS");

        myMux.lock();


        pcl::visualization::PointCloudColorHandlerRGBField<PointT> KinectColor1(KinectPointCloud1);
        if (viewer->updatePointCloud(KinectPointCloud1, KinectColor1, "Kinect1"))
            viewer->updatePointCloudPose("Kinect1",Eigen::Affine3f(KinectPointCloud1->sensor_orientation_));
        else viewer->addPointCloud(KinectPointCloud1, KinectColor1, "Kinect1");


        pcl::visualization::PointCloudColorHandlerRGBField<PointT> KinectColor2(KinectPointCloud2);
        if (viewer->updatePointCloud(KinectPointCloud2, KinectColor2, "Kinect2"))
            viewer->updatePointCloudPose("Kinect2",Eigen::Affine3f(KinectPointCloud2->sensor_orientation_));
        else    viewer->addPointCloud(KinectPointCloud2, KinectColor2, "Kinect2");






        pcl::visualization::PointCloudColorHandlerCustom<PointT> KinectKPColor1(KinectKeyPoints1, 1.0, 1.0, 1.0);
        if (viewer->updatePointCloud(KinectKeyPoints1, KinectKPColor1, "KinectKeyPoints1"))
            viewer->updatePointCloudPose("KinectKeyPoints1",Eigen::Affine3f(KinectKeyPoints1->sensor_orientation_));
        else    viewer->addPointCloud(KinectKeyPoints1, KinectKPColor1, "KinectKeyPoints1");


        pcl::visualization::PointCloudColorHandlerCustom<PointT> KinectKPColor2(KinectKeyPoints2, 1.0, 1.0, 1.0);
        if (viewer->updatePointCloud(KinectKeyPoints2, KinectKPColor2, "KinectKeyPoints2"))
            viewer->updatePointCloudPose("KinectKeyPoints2",Eigen::Affine3f(KinectKeyPoints2->sensor_orientation_));
        else    viewer->addPointCloud(KinectKeyPoints2, KinectKPColor2, "KinectKeyPoints2");

        myMux.unlock();
        viewer->spinOnce (1);


    }

    //myada->close();
    dev->stop();
    dev->close();

    return 0;
}

#pragma once

#ifndef KinFuWindowLight_H
#define KinFuWindowLight_H

#include <iostream>

// Qt
#include <QMainWindow>
#include <QTimer>
#include <QTime>
#include <vector>
#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <QDebug>

// KinFu
#include <pcl/gpu/containers/initialization.h>
#include <KinFu/kinfu.h>
#include <KinFu/raycaster.h>
#include <KinFu/marching_cubes.h>


// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/image_viewer.h>
//#include <pcl/filters/passthrough.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>



//#include <pcl/visualization/image_viewer.h>






namespace Ui
{
class KinFuWindowLight;
}

class KinFuWindowLight : public QMainWindow
{
    Q_OBJECT

public:
    explicit KinFuWindowLight (QWidget *parent = 0);
  //  explicit KinFuWindowLight (KinFuApp myKinFuApp, QWidget *parent = 0);
    ~KinFuWindowLight ();


    // Grabber
    pcl::Grabber* interface;


    // Point cloud callback
    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & DC3);

    // Run Kinect
    int run();

    // Stop Kinect
    int stop();


public:
    bool StopStream;

public slots:
    void processFrameAndUpdateGUI();

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  //  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerDepth_;

private slots:
    void on_btnStopStream_toggled(bool checked);

    void on_btnResetCamera_clicked();

    void on_btnUpdateScene_clicked();

private:
    Ui::KinFuWindowLight *ui;

    QTimer *tmrTimer;
    QTimer *scnTimer;

    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr KinectCloud;
    pcl::gpu::KinfuTracker kinfu_;
    pcl::gpu::KinfuTracker::DepthMap depth_device_;

    pcl::gpu::PtrStepSz<const unsigned short> depth_;
    pcl::gpu::PtrStepSz<const pcl::gpu::KinfuTracker::PixelRGB> rgb24_;
    std::vector<unsigned short> source_depth_data_;
    std::vector<pcl::gpu::KinfuTracker::PixelRGB> source_image_data_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_ptr_;
  //  pcl::visualization::ImageViewer::Ptr viewerDepth_;
  //  pcl::visualization::ImageViewer::Ptr viewerScene_;


    pcl::gpu::MarchingCubes::Ptr marching_cubes_;
    pcl::gpu::DeviceArray<pcl::PointXYZ> triangles_buffer_device_;

    boost::shared_ptr<pcl::PolygonMesh> mesh_ptr_;

    QMutex mex;

};

#endif // KinFuWindowLight_H

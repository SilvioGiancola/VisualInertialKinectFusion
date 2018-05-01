#pragma once

#ifndef KinFuWindow_H
#define KinFuWindow_H

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
#include <QFileDialog>
    #include <QDirModel>

// KinFu
#include <pcl/gpu/containers/initialization.h>
//#include <KinFu/kinfu.h>
#include <pcl/gpu/kinfu/kinfu.h>
#include <pcl/gpu/kinfu/marching_cubes.h>


// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/image_viewer.h>
//#include <pcl/filters/passthrough.h>
#include <pcl/common/time.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include <pcl/common/time.h>
#include <pcl/common/angles.h>

#define PI 3.14159265359


// Librealsense

//#include "example.hpp"

#include <chrono>
#include <vector>
#include <sstream>
#include <iostream>
#include <algorithm>



#include <exception>
//#include <pcl/visualization/image_viewer.h>


#include<Devices/adafruit_uart.h>
#include <QSettings>

#include <QKeyEvent>

#include <pcl/filters/conditional_removal.h> // ROI
#include <pcl/filters/extract_indices.h>



//#include <librealsense/rs.hpp>


#define NOISY       3.5		// Remove points past NOISY meters
#define FPS_MILLI   500		// Update fps every 0.5 seconds

namespace Ui
{
class KinFuWindow;
}


class ParamKinFuBatch
{
public:
    ParamKinFuBatch();
    ParamKinFuBatch(bool useIMU, float lambda, bool useNoiseModel)
    {
        this->useIMU = useIMU;
        this->lambda = lambda;
        this->useNoiseModel = useNoiseModel;
    }

    bool useIMU;
    float lambda;
    bool useNoiseModel;
};

class KinFuWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit KinFuWindow (QWidget *parent = 0);
    ~KinFuWindow ();


  //  rs::context rsContext;
    //  rs::device * RealSenseDevice = NULL;
    // Grabber
    //pcl::Grabber* interface;
    pcl::Grabber* interface_kinect = NULL;
   // pcl::Grabber* interface_files = NULL;


    // Point cloud callback
    void CallBackKinectV1 (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & DC3);
 //   void CallBackFiles (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & DC3);


    // Run Kinect
    // int run();

    // Stop Kinect
    // int stop();


    //public:
    //bool StopStream;

    void keyPressEvent(QKeyEvent* e);



    void saveKinFuParameters(QString path);
    void openKinFuParameters(QString path);



public slots:
    // void processFrame();
    void UpdatePointCloud();
    void UpdateModel();
       void CallBackFiles ();

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerDepth_;
  /*  pcl::visualization::ImageViewer::Ptr viewerDepth_;
    pcl::visualization::ImageViewer::Ptr viewerScene_;*/

public slots:

    void on_btnStopStream_clicked(bool checked);


    void on_btnUpdateScene_clicked(bool checked);

    void on_btnResetKinFu_clicked();

    void on_btnDownloadModel_clicked();



    void on_doubleSpinBox_FreqStream_valueChanged(double arg1)
    {
        if (tmrTimer->isActive())
        {
            tmrTimer->stop();
            tmrTimer->start(1000.0f/arg1);
        }
    }
    void on_doubleSpinBox_FreqModel_valueChanged(double arg1)
    {
        if (scnTimer->isActive())
        {
            scnTimer->stop();
            scnTimer->start(1000.0f/arg1);
        }
    }





    void on_tabWidget_currentChanged(int index);



    void Adafruit_init();



    void KinectV1_init();
    void on_KinectV1_start_clicked();
    void on_KinectV1_stop_clicked();
    void on_KinectV1_save_clicked();
    void on_KinectV1_toggle_clicked();
    bool is_KinectV1_active();



    void Files_init(QString Dir);
    void on_Files_openFiles_clicked();
    void on_Files_start_clicked(bool checked);
    void on_Files_stop_clicked();
    void on_Files_grabsingle_clicked();
    void on_Files_toggle_clicked();
    bool is_Files_active();


    bool ElabSinglePC(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &KinectCloud);

    // Update ICP
    void on_spinBox_ICPcoarse_valueChanged(int arg1);
    void on_spinBox_ICPmedium_valueChanged(int arg1);
    void on_spinBox_ICPfine_valueChanged(int arg1);

    //Update lambda



    // IMU regularization
    void on_doubleSpinBox_lambda_valueChanged(double arg1);



    // Initial Pose visualization
    void setInitialPose();
    void setWorkingCube();
    void setTrajectory();
    void setIcpCorespFilteringParams();

    //Vsiaulazation
    void showInitialPose(bool checked);
    void showWorkingCube(bool checked);
    void showTrajectory(bool checked);


    void on_pushButton_InitAdafruit_clicked();

    void on_doubleSpinBox_Size_X_valueChanged(double arg1);
    void on_doubleSpinBox_Size_Y_valueChanged(double arg1);
    void on_doubleSpinBox_Size_Z_valueChanged(double arg1);

    void on_pushButton_ResetViewer_clicked();

    void on_pushButton_SavePointofView_clicked();

    void on_doubleSpinBox_distThresh_valueChanged(double arg1);

    void on_doubleSpinBox_angleThresh_valueChanged(double arg1);

private slots:
    void on_pushButton_SaveParams_clicked();

    void on_pushButton_OpenParams_clicked();


    void on_btnSaveModel_clicked();
    void SaveModel(QString Path);
    void SavePoses(QString Path);

    void on_pushButton_BatchElab_clicked();

    void on_btnDownloadTrajectory_clicked();


    void on_btnSavePoses_clicked();

    void on_pushButton_BatchElab_TUM_clicked();

    void on_pushButton_CurrentPCPose_clicked();

    void on_pushButton_RefreshDatasets_clicked();

    void on_pushButton_SaveParamsAs_clicked();


    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

public:
    void RefreshTUMdatasets(QString RootFolder);
    Eigen::Quaternionf applyNoiseModel(Eigen::Quaternionf quat);

private:
    Ui::KinFuWindow *ui;

    QTimer *tmrTimer;
    QTimer *scnTimer;
    QTimer *kinFuTimer;
  //  int cnt;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr KinectCloud;
   // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr KinectCloud;
    QList<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> KinectCloudAll;
    pcl::gpu::KinfuTracker kinfu_;
    pcl::gpu::KinfuTracker::DepthMap depth_device_;

    pcl::gpu::PtrStepSz<const unsigned short> depth_;
    pcl::gpu::PtrStepSz<const pcl::gpu::KinfuTracker::PixelRGB> rgb24_;
    std::vector<unsigned short> source_depth_data_;
    std::vector<pcl::gpu::KinfuTracker::PixelRGB> source_image_data_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_ptr_;


    pcl::gpu::MarchingCubes::Ptr marching_cubes_;
    pcl::gpu::DeviceArray<pcl::PointXYZ> triangles_buffer_device_;

    boost::shared_ptr<pcl::PolygonMesh> mesh_ptr_;

    QMutex mex;



    std::vector<std::string> myListofPointCloudFiles;
    int current_indexFile;


    Adafruit_UART myAda;

    QString ParamsKinFuFile;


};

#endif // KinFuWindow_H

#pragma once

#ifndef KINECTVIEWER_H
#define KINECTVIEWER_H

#include <iostream>

// Qt
#include <QMainWindow>
#include <QTimer>
//#include <QTime>
#include <QDateTime>
#include <QDir>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include <vector>
#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <QDebug>

#include "Devices/adafruit_uart.h"

namespace Ui
{
  class KinectViewer;
}

class KinectViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit KinectViewer (QWidget *parent = 0);
  ~KinectViewer ();

public:
  // Grabber
  pcl::Grabber* interface;
  // Running
 // bool bRun;
  // Copying
 // bool bCopying;
  // Error
 // int er;
  //
 // bool firstCall;

public:
    // Point cloud callback
    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);

    // Run Kinect
    int run();

    // Stop Kinect
    int stop();

private:
    QTimer *tmrTimer;
    QTimer *KinFreqTimer;
    int KinFreqCounter;
    QTimer *WinFreqTimer;
    int WinFreqCounter;

public:
  //bool StopStream;

public slots:
  void processFrameAndUpdateGUI();
  void restartKinFreqCounter();
  void restartWinFreqCounter();

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

private slots:
//  void on_btnStopStream_toggled(bool checked);

  void on_btnResetCamera_clicked();

  void on_checkBox_startStream_clicked(bool checked);

  void on_checkBox_savePointCloud_clicked(bool checked);

  void on_checkBox_includeOrientation_clicked(bool checked);

  void on_pushButton_save_clicked();

private:
  Ui::KinectViewer *ui;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr KinectCloud;

  QMutex mex;


  Adafruit_UART myAda;



  QList<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> PCtoSave;
/*
// XYZ point vector
std::vector<float> cloudX, cloudY, cloudZ;
// RGB vector
std::vector<unsigned long> cloudRGB;
// Size of cloud
int cloudWidth;
int cloudHeight;*/

};

#endif // KINECTVIEWER_H

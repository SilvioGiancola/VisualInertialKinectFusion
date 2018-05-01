#ifndef KINECTTAB_H
#define KINECTTAB_H

#include <QWidget>

#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


//#include <libfreenect2/libfreenect2.hpp>
//#include <libfreenect2/frame_listener_impl.h>
////#include <libfreenect2/threading.h>
//#include <libfreenect2/registration.h>
//#include <libfreenect2/packet_pipeline.h>

#include <QTime>
#include <QDateTime>
#include <QFile>
#include <QTextStream>
#include <define.h>

#include <Eigen/Dense>
#include <PCViewer/modelpointcloudlist.h>
#include <Devices/kinect.h>



///////////////////////////////////////////////
#include <stdio.h>  // for printf, fprintf, NULL, etc
#include <stdlib.h> // for exit, atoi
#ifndef _WIN32_WCE
#include <signal.h> // for signal, SIGINT
#endif
#include <string.h>              // for strcmp, strncpy
#include <vrpn_FileConnection.h> // For preload and accumulate settings
#include <vrpn_Shared.h>         // for vrpn_SleepMsecs
#include <vrpn_Tracker.h>        // for vrpn_TRACKERACCCB, etc
#include <vector>                // for vector

#include "vrpn_BaseClass.h" // for vrpn_System_TextPrinter, etc
#include "vrpn_Configure.h" // for VRPN_CALLBACK
#include "vrpn_Types.h"     // for vrpn_float64, vrpn_int32

#include <Devices/adafruit_uart.h>
#include <Devices/myahrs_plus.hpp>

class t_user_callback {
public:
    Eigen::Matrix4f Pose;
    bool grabbed;
};


namespace Ui {
class KinectTab;
}

class KinectTab : public QWidget
{
    Q_OBJECT

public:
    explicit KinectTab(std::string serial = "");
    ~KinectTab();


signals:
    void Registration(int CurrentIndex, int ReferenceIndex);
    void PCgrabbed(PointCloudT::Ptr cloud);
public slots:


    void on_pushButton_setPose_clicked();
    void on_pushButton_getPose_clicked();
    void on_pushButton_resetPose_clicked();
    void on_pushButton_savePose_clicked();
    void on_pushButton_registerPose_clicked();



    void on_pushButton_Open_clicked();
    void on_pushButton_Close_clicked();
    void on_pushButton_Grab_clicked();

    bool Open_KinectPose();




private slots:
    void on_checkBox_Adafruit_clicked(bool checked);

    void on_pushButton_RotationSensor_Init_clicked();

    void on_checkBox_myAHRS_clicked(bool checked);

    void on_checkBox_NexCave_clicked(bool checked);

    void on_rB_Pipeline_CPU_clicked(bool checked);

    void on_rB_Pipeline_GL_clicked(bool checked);

    void on_rB_Pipeline_CL_clicked(bool checked);

private:
    Ui::KinectTab *ui;
    Kinect Kin;


    vrpn_Tracker_Remote *tkr;
    t_user_callback *tc1;
    Adafruit_UART * ada;
    WithRobot::MyAhrsPlus AHRS;

    Eigen::Matrix4f _PoseNexCaveOnKin;
    Eigen::Matrix4f _PoseAHRSOnKin;

    bool useAHRS = false;
    bool useAda = false;
    bool useNexCave = false;

};




#endif // KINECTTAB_H

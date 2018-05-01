#ifndef CALIBRATIONWINDOW_H
#define CALIBRATIONWINDOW_H

#include <QMainWindow>


#include <Devices/adafruit_uart.h>

#include <boost/thread.hpp>
#include <Eigen/Eigen>


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



class t_user_callback {
public:
    Eigen::Matrix4f Pose;
};

namespace Ui {
class CalibrationWindow;
}

class CalibrationWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit CalibrationWindow(QWidget *parent = 0);
    ~CalibrationWindow();



private slots:
    void on_Adafruit_Open_clicked();
    void on_Adafruit_Init_clicked();
    void on_AdaFruit_Close_clicked();
    void on_Adafruit_Grab_clicked();
    void on_Adafruit_EmptyList_clicked();


    void on_NexCave_Open_clicked();
    void on_NexCave_Grab_clicked();
    void on_NexCave_EmptyList_clicked();


    void on_Adafruit_getCal_clicked();

private:
    Ui::CalibrationWindow *ui;

    Adafruit_UART ada;

    vrpn_Tracker_Remote *tkr;
    t_user_callback *tc1;

    QList<Eigen::Matrix4f> PosesAdafruit;
    QList<Eigen::Matrix4f> PosesNexCave;



signals:
    void ShowCoordinateSystem(double,Eigen::Affine3f,std::string);
};

#endif // CALIBRATIONWINDOW_H

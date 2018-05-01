#include <QApplication>

#include <iostream>
#include <signal.h>



#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>


#include <QFile>
#include <QTextStream>
#include <QTime>


#include <boost/thread.hpp>
#include <Eigen/Eigen>

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

#include <Devices/adafruit_uart.h>

#include "vrpn_BaseClass.h" // for vrpn_System_TextPrinter, etc
#include "vrpn_Configure.h" // for VRPN_CALLBACK
#include "vrpn_Types.h"     // for vrpn_float64, vrpn_int32

using namespace std;


class t_user_callback {
public:
    char t_name[vrpn_MAX_TEXT_LEN];
    vector<unsigned> t_counts;
};

/*****************************************************************************
*
Callback handlers
*
*****************************************************************************/

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
pcl::visualization::PCLVisualizer::Ptr viewer;
boost::shared_ptr<pcl::visualization::PCLPlotter> plotter;

Eigen::Quaternionf quat_Ada = Eigen::Quaternionf::Identity();
Eigen::Quaternionf quat_Cave = Eigen::Quaternionf::Identity();
Eigen::Vector3f T_Cave = Eigen::Vector3f::Zero();
Adafruit_UART ada;

void
handle_tracker_pos_quat(void *userdata, const vrpn_TRACKERCB t)
{
    t_user_callback *t_data = static_cast<t_user_callback *>(userdata);

    // Make sure we have a count value for this sensor
    while (t_data->t_counts.size() <= static_cast<unsigned>(t.sensor))
        t_data->t_counts.push_back(0);


    T_Cave[0] = t.pos[0]; T_Cave[1] = t.pos[1]; T_Cave[2] = t.pos[2];
    quat_Cave.x() = t.quat[0]; quat_Cave.y() = t.quat[1]; quat_Cave.z() = t.quat[2]; quat_Cave.w() = t.quat[3];
}



uint XStep = 150;
uint YStep = 70;
uint Size = 30;
std::string Button1_Name = "CALIBER";
std::string Button2_Name = "REGISTER";
bool Button1_State = false;
bool Button2_State = false;

std::vector<double> Xs;
std::vector<double> Ys;
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
                Button1_State = !Button1_State;
                if (Button1_State)  viewer->updateText(Button1_Name, XStep*0, YStep*0+10, Size, 1,   1,   1,   Button1_Name);
                else                viewer->updateText(Button1_Name, XStep*0, YStep*0+10, Size, 0.3, 0.3, 0.3, Button1_Name);
            }

            else if (event.getX() < XStep*2)
            {
                Button2_State = !Button2_State;

                if (Button2_State)
                {
                    viewer->updateText(Button2_Name, XStep*1, YStep*0+10, Size, 1,   1,   1,   Button2_Name);
                }
                else
                {
                    viewer->updateText(Button2_Name, XStep*1, YStep*0+10, Size, 0.3, 0.3, 0.3, Button2_Name);
                    Xs.clear();
                    Ys.clear();
                }
            }
        }
    }
}



Eigen::Matrix4f loadMatrix(std::string filename)
{
    Eigen::Matrix4f m;
    std::ifstream input(filename.c_str());
    if (input.fail())
    {
        std::cerr << "ERROR. Cannot find file '" << filename << "'." << std::endl;
        m = Eigen::Matrix4f(0,0);
        return m;
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


    return m;

}
/**********************
 * MAIN
 ****************/



int
main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // Init Viewer
    viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();
    viewer->setBackgroundColor (0.5, 0.5, 0.5);
    viewer->addCoordinateSystem (1.000);

    pcl::ModelCoefficients line_coeff;
    line_coeff.values.resize (6);    // We need 6 values

    int XMin = -3; int XMax = 3;
    int YMin = -1; int YMax = 3;

    line_coeff.values[1] = YMin;
    line_coeff.values[2] = 0;
    line_coeff.values[3] = 0;
    line_coeff.values[4] = YMax-YMin;
    line_coeff.values[5] = 0;

    for (int i = XMin; i <= XMax; i++)
    {
        line_coeff.values[0] = i;
        viewer->addLine (line_coeff, QString("line X%1").arg(i).toStdString());
    }

    line_coeff.values[0] = XMin;
    line_coeff.values[2] = 0;
    line_coeff.values[3] = XMax-XMin;
    line_coeff.values[4] = 0;
    line_coeff.values[5] = 0;

    for (int i = YMin; i <= YMax; i++)
    {
        line_coeff.values[1] = i;
        viewer->addLine (line_coeff, QString("line Y%1").arg(i).toStdString());
    }


    viewer->setCameraClipDistances(-10, 10);
    viewer->setCameraPosition(0.3, -4.0, 1.5,    // From where I am looking at
                              0.0,  1.0, 1.0,    // Where I am looking at
                              0.0,  0.0, 1.0);   // What is the up orientation


    //  viewer->registerPointPickingCallback(&pointSelected, *this);    // Register the callback from Point Picking event to the function
    // viewer->registerAreaPickingCallback(&areaSelected, *this);      // Register the callback from Area Picking event to the function
    //    viewer->registerKeyboardCallback(&keyBoardPressed, *this);      // Register the callback from Keyborad event to the function
    viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

    viewer->addText(Button1_Name, XStep*0, YStep*0 + 10, Size, 0.3, 0.3, 0.3, Button1_Name);
    viewer->addText(Button2_Name, XStep*1, YStep*0 + 10, Size, 0.3, 0.3, 0.3, Button2_Name);

    viewer->spinOnce(1);

    plotter.reset(new pcl::visualization::PCLPlotter ("Plotter"));


    Xs.push_back(0); Xs.push_back(5); Xs.push_back(2);
    Ys.push_back(6); Ys.push_back(2); Ys.push_back(0);
    plotter->addPlotData(Xs, Ys);
    plotter->renderOnce();
    Xs.clear();
    Ys.clear();
    //defining the polynomial function, y = x^2. Index of x^2 is 1, rest is 0
    //  std::vector<double> func1 (3,0);
    //  func1[2] = 1;

    //adding the polynomial func1 to the plotter with [-10, 10] as the range in X axis and "y = x^2" as title
    //  plotter->addPlotData (func1, -10, 10, "y = x^2");

    //display the plot, DONE!




    if (ada.open() != SUCCESS)
    {
        std::cout << "error opening Adafruit sensor" << std::endl;
        return ERROR;
    }

    std::cout << "opened" << std::endl;

//    if (ada.Init() != SUCCESS)
  //  {
    //    std::cout << "error init" << std::endl;
        //   return ERROR;
    //f}



    QString myName("dtrackbody@109.171.138.177");

    vrpn_Tracker_Remote *tkr = new vrpn_Tracker_Remote(myName.toStdString().c_str());

    t_user_callback *tc1 = new t_user_callback;
    if (tc1 == NULL)
        fprintf(stderr, "Out of memory\n");


    // Fill in the user-data callback information
    strncpy(tc1->t_name, myName.toStdString().c_str(), sizeof(tc1->t_name));


    // Set up the tracker callback handlers
    tkr->register_change_handler(tc1, handle_tracker_pos_quat, vrpn_ALL_SENSORS);


    Eigen::Matrix4f pose_Ada = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f pose_Cave = Eigen::Matrix4f::Identity();

    Eigen::Quaternionf calib = Eigen::Quaternionf::Identity();
    Eigen::Matrix4f  calibMatrix = Eigen::Matrix4f::Identity();


    int i = 0;
    while(!viewer->wasStopped() && !plotter->wasStopped())
    {
        QTime t;
        t.start();


        if (ada.GetQuat(&quat_Ada) != SUCCESS)
            continue;


        tkr->mainloop();

        //   std::cout << t.elapsed() << std::endl;



        if (Button1_State)
        {
            calibMatrix = loadMatrix("../PCViewer/AdaOnNexCave.txt");

            calib = Eigen::Quaternionf( calibMatrix.block<3,3>(0,0)) ;

        //    calib = Eigen::Quaternionf(pose_Cave.block<3,3>(0,0)) * calib;
            std::cout << "Calib" << std::endl;
            std::cout << calib.w() << " " << calib.x() << " " << calib.y() << " " << calib.z() << " " << std::endl;
            std::cout << "quat_Ada" << std::endl;
            std::cout << quat_Ada.w() << " " << quat_Ada.x() << " " << quat_Ada.y() << " " << quat_Ada.z() << " " << std::endl;
            std::cout << "quat_Cave" << std::endl;
            std::cout << quat_Cave.w() << " " << quat_Cave.x() << " " << quat_Cave.y() << " " << quat_Cave.z() << " " << std::endl;
            Button1_State = false;
        }


        pose_Cave.block(0,0,3,3) = quat_Cave.matrix();
        pose_Cave.block(0,3,3,1) = T_Cave;

        pose_Ada.block(0,0,3,3) = calib.matrix() * quat_Ada.matrix();
        pose_Ada.block(0,3,3,1) = T_Cave;


        // Update NexCave
        if (viewer->contains("markers"))
            viewer->removeCoordinateSystem("markers");
        viewer->addCoordinateSystem(0.5,Eigen::Affine3f(pose_Cave),"markers");

        // Update AdaFruit
        if (viewer->contains("Adafruit"))
            viewer->removeCoordinateSystem("Adafruit");
        viewer->addCoordinateSystem(0.5,Eigen::Affine3f(pose_Ada),"Adafruit");

// update Diference
        if (viewer->contains("diff"))
            viewer->removeCoordinateSystem("diff");
        viewer->addCoordinateSystem(0.5,Eigen::Affine3f(pose_Ada.inverse() * pose_Cave),"diff");




        if(Button2_State)
        {
            Eigen::Matrix3f error_Matrix = quat_Cave.matrix() * quat_Ada.matrix().inverse();
            Eigen::Quaternionf q = Eigen::Quaternionf(error_Matrix);
            //std::cout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " " << std::endl;
            double angle = Eigen::AngleAxisf(q).angle() * 180 / 3.14159;
            //std::cout << angle << std::endl;
            //std::cout << std::acos(q.w()) * 2 * 180 / 3.14159 << std::endl;
            if (angle < 720 && angle > -720)
            {
                Xs.push_back(i);
                Ys.push_back(angle);
                plotter->clearPlots();
                plotter->addPlotData(Xs, Ys);
                i++;
                if (Xs.size() > 10000)  Xs.clear();
                if (Ys.size() > 10000)  Ys.clear();
            }
            plotter->renderOnce();
        }




        viewer->spinOnce(1);

        vrpn_SleepMsecs(1); // Sleep for 1ms so we don't eat the CPU
    }

    ada.close();


    delete tkr;

    return 0;
}

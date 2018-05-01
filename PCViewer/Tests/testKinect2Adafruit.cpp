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



#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>

#include <QFile>
#include <QTextStream>
#include <QTime>
#include <Devices/kinect.h>
#include <Devices/adafruit_uart.h>

using namespace std;


Kinect * mykinect;
Adafruit_UART * myada;

uint XStep = 150;
uint YStep = 70;
uint Size = 30;


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

    if (event.keyDown())
    {
        cout << "KeyboardEvent: " << event.getKeySym ()  << endl;

        /*       if (event.getKeySym () == "h" )
        {
            cout << "HELP - Press:" << endl;
            cout << "   - o for Opening Kinect Connection" << endl;
            cout << "   - c for Closing Kinect Connection" << endl;
            cout << "   - p for Play/Pause Kinect Connection" << endl;
            cout << "   - h for Help" << endl;
        }

        if (event.getKeySym () == "o" )
            kin.Open()  ;

        if (event.getKeySym () == "c" )
            kin.Close();

        if (event.getKeySym () == "p" )
            play = !play;

*/

    }
    return;
}

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
                if (!mykinect->isOpen())
                {
                    if (mykinect->Open(0) == SUCCESS)
                        viewer->updateText("CONN.", XStep*0, YStep*0+10, Size, 1, 1, 1, "CONNECT");
                }
                else
                {
                    if (mykinect->Close() == SUCCESS)
                        viewer->updateText("CONN.",  XStep*0, YStep*0+10, Size, 0.3,0.3,0.3, "CONNECT");
                }
            }


            else if (event.getX() < XStep*2)
            {
                mykinect->_play= !mykinect->_play;
                if (mykinect->_play)
                    viewer->updateText("PLAY",  XStep*1, YStep*0+10, Size, 1, 1, 1, "PLAY");

                else
                    viewer->updateText("PLAY",   XStep*1, YStep*0+10, Size, 0.3,0.3,0.3, "PLAY");
            }


            else if (event.getX() < XStep*3)
            {
                mykinect->_save = !mykinect->_save;
                if (mykinect->_save)
                    viewer->updateText("SAVE",  XStep*2, YStep*0+10, Size, 1, 1, 1, "SAVE");

                else
                    viewer->updateText("SAVE",  XStep*2, YStep*0+10, Size, 0.3, 0.3, 0.3, "SAVE");
            }


            else if (event.getX() < XStep*4)
            {

                if (!myada->isOpen())
                {
                    if (myada->open() == SUCCESS)
                    {
                        myada->init();
                        viewer->updateText("IMU",  XStep*3, YStep*0+10, Size, 1, 1, 1, "IMU");
                    }
                }
                else
                {
                    myada->close();
                    viewer->updateText("IMU",  XStep*3, YStep*0+10, Size, 0.3, 0.3, 0.3, "IMU");
                }
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

int main(int argc, char *argv[])
{    


    mykinect = new Kinect();
    myada = new Adafruit_UART();


    Eigen::Matrix4f _PoseAdaOnKin;
    loadMatrix("../PCViewer/AdaOnKin.txt", _PoseAdaOnKin);
    std::cout << _PoseAdaOnKin << std::endl;
    Eigen::Quaternionf q = Eigen::Quaternionf(_PoseAdaOnKin.block<3,3>(0,0));
    std::cout << q.matrix() << std::endl;
    myada->setCalibPose(q);
    std::cout << myada->getCalibPose().matrix() << std::endl;



    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0.5, 0.5, 0.5);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setCameraPosition(0.5,0.5,-2.5, // mi posiziono dietro ad un Kinect
                              0.5,0.5,0.5, // guardo un punto centrale
                              0,1,0);   // orientato con la y verso l'alto


    viewer->addText("CONN.",  XStep*0, YStep*0 + 10, Size, 0.3, 0.3, 0.3, "CONNECT");
    viewer->addText("PLAY",   XStep*1, YStep*0 + 10, Size, 0.3, 0.3, 0.3, "PLAY");
    viewer->addText("SAVE",   XStep*2, YStep*0 + 10, Size, 0.3, 0.3, 0.3, "SAVE");
    viewer->addText("IMU",    XStep*3, YStep*0 + 10, Size, 0.3, 0.3, 0.3, "IMU");



    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

    PointCloudT::Ptr KinectPointCloud(new PointCloudT);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> KinectColor(KinectPointCloud);
    viewer->addPointCloud(KinectPointCloud, KinectColor, "Kinect");



    viewer->addText("FPS",XStep,0,10,1,1,1,"FPS");

    QTime t;
    t.start();

   // myada->setCalibPose(Eigen::Quaternionf(0, M_SQRT1_2, -M_SQRT1_2, 0));

    while (!viewer->wasStopped ())
    {
        t.restart();

        if (mykinect->_play)            mykinect->GrabPointCloud(KinectPointCloud);

        if (myada->isOpen())            KinectPointCloud->sensor_orientation_ = myada->returnPose();
        else                            KinectPointCloud->sensor_orientation_ = Eigen::Quaternionf::Identity();


        if (mykinect->_play && mykinect->_save)
            pcl::io::savePCDFileBinary(KinectPointCloud->header.frame_id, *KinectPointCloud);

        if (!mykinect->_play)
            KinectPointCloud.reset(new PointCloudT());


        viewer->updateText(QString("FPS = %1 Hz").arg(1000.0f/t.elapsed()).toStdString(),XStep,0,10,1,1,1,"FPS");

        pcl::visualization::PointCloudColorHandlerRGBField<PointT> KinectColor(KinectPointCloud);
        if (viewer->contains("Kinect"))
            viewer->removePointCloud("Kinect");
        viewer->addPointCloud(KinectPointCloud,KinectColor,"Kinect");

        Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
        trans.block(0,0,3,3) = KinectPointCloud->sensor_orientation_.matrix();
        trans.block(0,3,4,1) = KinectPointCloud->sensor_origin_;

      //  if (viewer->contains("RefSyst"))
        viewer->removeAllCoordinateSystems();
        viewer->addCoordinateSystem(0.5, Eigen::Affine3f(trans),"RefSyst");
        viewer->addCoordinateSystem(1.0);



     /*   viewer->removeAllShapes();
        pcl::ModelCoefficients line_coeff;
        line_coeff.values.resize (6);    // We need 6 values

        if (mykinect->_play)
        {


            line_coeff.values[0] = 0;
            line_coeff.values[1] = 0;
            line_coeff.values[2] = 0;
            line_coeff.values[3] = -10.0 * (mykinect->dev->getIrCameraParams().cx - 512) / mykinect->dev->getIrCameraParams().fx;
            line_coeff.values[4] =  10.0 * (mykinect->dev->getIrCameraParams().cy - 424) / mykinect->dev->getIrCameraParams().fy;
            line_coeff.values[5] =  10.0;
            viewer->addLine (line_coeff, QString("line_corner_bottomleft").toStdString());
            line_coeff.values[2] = 0;
            line_coeff.values[3] = -10.0 * (mykinect->dev->getIrCameraParams().cx - 512) / mykinect->dev->getIrCameraParams().fx;
            line_coeff.values[4] =  10.0 * (mykinect->dev->getIrCameraParams().cy - 424) / mykinect->dev->getIrCameraParams().fy;
            line_coeff.values[5] =  10.0;
            viewer->addLine (line_coeff, QString("line_corner_topright").toStdString());
        }*/

        viewer->spinOnce (1);



        // cout << QString("Acquisition Time = %1 ms").arg(t.elapsed()).toStdString() << endl;
        //  viewer->updateText(QString("Acquisition Time = %1 ms").arg(t.elapsed()).toStdString(),50,50,20,1,1,1,"AcqTime");

    }

    myada->close();
    mykinect->Close();
    return 0;
}

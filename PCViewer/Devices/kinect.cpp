#include "kinect.h"



Kinect::Kinect(QWidget *parent) : QThread(parent)
{
    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

    _PoseKinect = Eigen::Matrix4f::Identity();


    _open = false;
    _play = false;

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
    pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
    pipeline = new libfreenect2::CpuPacketPipeline();
#endif
#endif

    //listener->onNewFrame();
}

Kinect::~Kinect()
{
    _play = false;
}


// Opening function
int Kinect::Open(int i)
{
    if (_open)
    {
        std::cout << "Device is already open!" << std::endl;
        return ERR_KIN_OPEN ;
    }

    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return ERR_KIN_OPEN;
    }

    _serial = freenect2.getDeviceSerialNumber(i);

    dev = freenect2.openDevice(_serial,pipeline);

    if(dev == 0)
    {
        std::cout << "no device connected or failure opening the default one!" << std::endl;
        return ERR_KIN_OPEN;
    }

    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);
    dev->start();


    cout << dev->getColorCameraParams().cx << endl;
    cout << dev->getColorCameraParams().cy << endl;
    cout << dev->getColorCameraParams().fx << endl;
    cout << dev->getColorCameraParams().fy << endl;

    cout << dev->getIrCameraParams().cx << endl;
    cout << dev->getIrCameraParams().cy << endl;
    cout << dev->getIrCameraParams().fx << endl;
    cout << dev->getIrCameraParams().fy << endl;
    cout << dev->getIrCameraParams().k1 << endl;
    cout << dev->getIrCameraParams().k2 << endl;
    cout << dev->getIrCameraParams().k3 << endl;
    cout << dev->getIrCameraParams().p1 << endl;
    cout << dev->getIrCameraParams().p2 << endl;


    // Color
    registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

    // Sensor

    _open = true;

    return SUCCESS;
}

// Closing function
int Kinect::Close()
{
    if (!_open)
    {
        std::cout << "already closed" << std::endl;
        return ERR_KIN_OPEN;
    }
    // TODO: restarting ir stream doesn't work!
    // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
    dev->stop();
    dev->close();
    _open = false;
    return SUCCESS;
}

// Grabing function
int Kinect::GrabPointCloud(PointCloudT::Ptr PC)
{

    if (!_open)
    {
        std::cout << "stream not opened" << std::endl;
        return ERR_KIN_OPEN;
    }



    QDateTime timestamp = QDateTime::currentDateTime();



    // Acquire Frames
    libfreenect2::FrameMap frames;
    listener->waitForNewFrame(frames);



    QTime t;
    t.start();




    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];



  //  cv::Mat rgbMat = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
  //  std::string str = QString("/home/silvio/PointClouds/Kinect%1_%2.png").arg(_serial.c_str()).arg(timestamp.toString("yyyy_MM_dd_HH_mm_ss_zzz")).toStdString();
   // cv::Mat rgbMatinv; //= cv::Mat(1080, 1920, CV_8UC4);
//    cv::flip(rgbMat, rgbMatinv, 1);
   // cv::imwrite(str,rgbMat);



    // Undistort and register frames
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
    registration->apply(rgb,depth,&undistorted,&registered);

    const float *undistorted_data = (float*)undistorted.data;
    const unsigned int *registered_data = (unsigned int*)registered.data;

    listener->release(frames);



    // Initialize my Point Cloud
    //   PC.reset(new PointCloudT());
    //PointCloudT::Ptr PC(new PointCloudT);
    PC->resize(undistorted.width * undistorted.height); // set the memory size to allocate
    PC->height = undistorted.height;        // set the height
    PC->width = undistorted.width;          // set the width
    PC->is_dense = false;                   // Kinect V2 returns organized and not dense point clouds



    int real_point = 0;
    // Set data into my Point cloud
    for (unsigned int i = 0; i < undistorted.height ;i++)
    {
        for (unsigned int j = 0; j < undistorted.width ;j++)
        {
            int index = i * undistorted.width + j;

            float depth = 0;

            //if (ui->radioButton_Meter->isChecked())
            depth = undistorted_data[index] / 1000.0f;
            // else if (ui->radioButton_MilliMeter->isChecked())
            // depth = undistorted_data[index];

            unsigned int rgba = registered_data[index];

            PointT P;

            if ( depth != 0 && rgba != 0)
            {
                P.x = -depth * (dev->getIrCameraParams().cx - j) / dev->getIrCameraParams().fx;
                P.y =  depth * (dev->getIrCameraParams().cy - i) / dev->getIrCameraParams().fy;
                P.z =  depth;
                //  P.y = -depth * (dev->getIrCameraParams().cx - j) / dev->getIrCameraParams().fx;
                // P.z =  depth * (dev->getIrCameraParams().cy - i) / dev->getIrCameraParams().fy;
                //  P.x =  depth;


                P.a = (rgba >> 24) & 0xFF;
                P.r = (rgba >> 16) & 0xFF;
                P.g = (rgba >> 8)  & 0xFF;
                P.b =  rgba        & 0xFF;

                real_point++;
            }
            else
            {
                P.x = P.y = P.z = std::numeric_limits<float>::quiet_NaN();
            }

            PC->at(j,i) = P;
        }
    }



    PC->sensor_origin_ = _PoseKinect.block<4,1>(0,3);                               // set the translation of the Kinect
    PC->sensor_orientation_ = Eigen::Quaternionf(_PoseKinect.block<3,3>(0,0));      // Set the rotation of the Kinect




    PC->header.frame_id = QString("/home/silvio/PointClouds/Kinect%1_%2.pcd").arg(_serial.c_str()).arg(timestamp.toString("yyyy_MM_dd_HH_mm_ss_zzz")).toStdString();
    PC->header.stamp = timestamp.toMSecsSinceEpoch();                               // the stamp correspond to the acquisition time



    std::cout << QDateTime::currentDateTime().toString().toStdString() << " time elapsed :" << t.elapsed() << "  PointCloud is : " << real_point << std::endl;




    //emit GrabbedPC(PC);

    return SUCCESS;
}

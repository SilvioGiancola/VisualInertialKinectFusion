#ifndef KINECT_H
#define KINECT_H

#include <QWidget>

#include <Eigen/Dense>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>  // transfromation

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <iostream>
#include <QTime>
#include <QString>
#include <QDateTime>
#include <QFile>
#include <QTextStream>
#include <QThread>
#include <define.h>
#include <fstream>


using namespace std;

#define SUCCESS 0
#define ERR_KIN_OPEN 1

//typedef pcl::PointXYZRGBA PointT;
//ypedef pcl::PointCloud<PointT> PointCloudT;


class Kinect : public QThread
{
    Q_OBJECT
public:
    // Constructor
    explicit Kinect(QWidget *parent = 0);
    ~Kinect();

    // Opening function
    int Open(int i = 0);

    // Closing function
    int Close();

    // Grabing function
    int GrabPointCloud(PointCloudT::Ptr PC);


    // Opening verification function
    bool isOpen() {return _open;}
    QString getFirmwareVersion(){return QString(dev->getFirmwareVersion().c_str());}
    QString getSerialNumber(){return QString(dev->getSerialNumber().c_str());}
    void setPose(Eigen::Matrix4f mypose) { _PoseKinect = mypose;}
    Eigen::Matrix4f getPose() { return _PoseKinect;}

    void setPipeline(string mypip)
    {
        if (mypip == "GL" && isGLcapable())
            pipeline = new libfreenect2::OpenGLPacketPipeline();

#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
       else if (mypip == "CL" && isCLcapable())
            pipeline = new libfreenect2::OpenCLPacketPipeline();
#endif
        else
            pipeline = new libfreenect2::CpuPacketPipeline();
    }


    bool isGLcapable()
    {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        return true;
#else
        return false;
#endif
    }
    bool isCLcapable()
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        return true;
#else
        return false;
#endif
    }



    bool _play;
    bool _save = false;


    libfreenect2::Freenect2Device *dev; //TODO: replace in private
signals:

    void GrabbedPC(PointCloudT::Ptr PC);

private:
    // freenect2 stuff
    libfreenect2::Freenect2 freenect2;
    libfreenect2::SyncMultiFrameListener *listener;
    libfreenect2::Registration *registration;
    libfreenect2::PacketPipeline *pipeline;

    // check opening
    bool _open;

    Eigen::Matrix4f _PoseKinect;
    // current serial of Kinect
    std::string _serial;



};

#endif // KINECT_H

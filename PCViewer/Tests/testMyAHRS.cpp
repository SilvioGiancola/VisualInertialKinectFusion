#include <Devices/myahrs_plus.hpp>

#include <stdio.h>
#include <stdlib.h>

#include <vector>
#include <map>
#include <QTime>

using namespace WithRobot;

static const int BAUDRATE = 115200;

static const char* DIVIDER = "1";  // 100 Hz


void handle_error(const char* error_msg)
{
    fprintf(stderr, "ERROR: %s\n", error_msg);
    exit(1);
}

void wait_for_user_input()
{
    printf("\npress enter key to quit.\n");
    char c = getchar();
    exit(1);
}

#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char *argv[])
{

    pcl::visualization::PCLVisualizer viewer("myARHS");
    viewer.initCameraParameters ();
    viewer.setBackgroundColor (0.5, 0.5, 0.5);
    viewer.addCoordinateSystem (1.000);
    viewer.setCameraClipDistances(-10, 10);
    viewer.setCameraPosition(0.3, -4.0, 1.5,    // From where I am looking at
                             0.0,  1.0, 1.0,    // Where I am looking at
                             0.0,  0.0, 1.0);   // What is the up orientation

    std::string serial_device = "/dev/ttyACM0";
    int baudrate = 115200;


    MyAhrsPlus sensor;
    SensorData sensor_data;
    uint32_t sample_count = 0;

    /*
       * 	start communication with the myAHRS+.
       */
    if(sensor.start(serial_device, baudrate) == false)
    {
        handle_error("start() returns false");
    }

    /*
       *  set binary output format
       *   - select Quaternion and IMU data
       */
    if(sensor.cmd_binary_data_format("QUATERNION, IMU") == false)
    {
        handle_error("cmd_binary_data_format() returns false");
    }

    /*
       *  set divider
       *   - output rate(Hz) = max_rate/divider
       */
    if(sensor.cmd_divider(DIVIDER) ==false)
    {
        handle_error("cmd_divider() returns false");
    }

    /*
       *  set transfer mode
       *   - BC : Binary Message & Continuous mode
       */
    if(sensor.cmd_mode("BC") ==false)
    {
        handle_error("cmd_mode() returns false");
    }

    QTime t;


    viewer.addText("Adafruit:", 0, 0, 30, 1, 1,1,"TIME");
    while(!viewer.wasStopped())
    {
        t.start();
        if(sensor.wait_data() == true)
        { // waiting for new data

            sample_count = sensor.get_sample_count();
            sensor.get_data(sensor_data);

            QString log = QString("myAHRS time: %1 ms").arg(t.elapsed());

            Quaternion& q = sensor_data.quaternion;


            Eigen::Quaternionf quateig;
            quateig.w() = q.w;
            quateig.x() = q.x;
            quateig.y() = q.y;
            quateig.z() = q.z;



            Eigen::Matrix4f Pose = Eigen::Matrix4f::Identity();
            Pose.block(0,0,3,3) = quateig.matrix();



            if (viewer.contains("markers"))
                viewer.removeCoordinateSystem("markers");
            viewer.addCoordinateSystem(0.5,Eigen::Affine3f(Pose),"markers");
            viewer.updateText(log.toStdString(), 0, 0, "TIME");

            viewer.spinOnce(1);
        }
    }

    /*
       * 	stop communication
       */
    sensor.stop();
}

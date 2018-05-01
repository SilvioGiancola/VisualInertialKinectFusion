#include <QCoreApplication>

// VRPN STUFF
#include <vrpn_FileConnection.h> // For preload and accumulate settings
#include <vrpn_Shared.h>         // for vrpn_SleepMsecs
#include <vrpn_Tracker.h>        // for vrpn_TRACKERACCCB, etc
#include "vrpn_BaseClass.h" // for vrpn_System_TextPrinter, etc
#include "vrpn_Configure.h" // for VRPN_CALLBACK
#include "vrpn_Types.h"     // for vrpn_float64, vrpn_int32

// PCL STUFF
#include <pcl/visualization/pcl_visualizer.h>

#include <fstream>
#include <chrono>
#include <ctime>
#include <QDateTime>


pcl::visualization::PCLVisualizer::Ptr viewer; // my PCL Viewer
std::ofstream myfile; // Filel to save pose


class t_user_callback {
public:
    char t_name[vrpn_MAX_TEXT_LEN];
    std::vector<unsigned> t_counts;
};



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

void
mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

    if (event.getButton () == pcl::visualization::MouseEvent::LeftButton && event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
        std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;
        if (event.getY() < 80 && event.getX() < 150)
        {
            if (myfile.is_open()) // IF CLOSED
            {
                viewer->updateText("SAVE", 0, 10, 30, 0.3, 0.3, 0.3, "SAVE");
                myfile.close();
            }
            else // IF OPENED
            {
                using namespace std::chrono;
                viewer->updateText("SAVE", 0, 10, 30, 1.0, 1.0, 1.0, "SAVE");
                std::string pathname = getenv("HOME");
                std::string filename = QDateTime::currentDateTime().toString("yyyy_MM_dd_HH_mm_ss_zzz").toStdString();
                myfile.open (pathname + "/nexcave_trajectory_" + filename + ".txt");
            }
        }
    }
}


Eigen::Matrix4f _PoseNexCaveOnKin;
void
handle_tracker_pos_quat(void *userdata, const vrpn_TRACKERCB t)
{
    t_user_callback *t_data = static_cast<t_user_callback *>(userdata);


    // Make sure we have a count value for this sensor
    while (t_data->t_counts.size() <= static_cast<unsigned>(t.sensor)) {
        t_data->t_counts.push_back(0);
    }

    // See if we have gotten enough reports from this sensor that we should
    // print this one.  If so, print and reset the count.

    std::cout << t.sensor << "  " << t_data->t_name << std::endl ;


    Eigen::Vector3f trans;
    trans[0] = t.pos[0]; trans[1] = t.pos[1]; trans[2] = t.pos[2];

    Eigen::Quaternionf quat;
    quat.x() = t.quat[0]; quat.y() = t.quat[1]; quat.z() = t.quat[2]; quat.w() = t.quat[3];

    Eigen::Matrix4f Pose = Eigen::Matrix4f::Identity();
    Pose.block(0,0,3,3) = quat.matrix();
    Pose.block(0,3,3,1) = trans;

    Pose = Pose * _PoseNexCaveOnKin;

    Eigen::Vector4f trans_Kinect = Pose.block<4,1>(0,3);
    Eigen::Quaternionf quat_Kinect = Eigen::Quaternionf(Pose.block<3,3>(0,0));
    std::chrono::milliseconds millis = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());


    std::ostringstream message;
    message << std::fixed << millis.count()/1000.0 << " " <<
               trans_Kinect[0] << " " << trans_Kinect[1] << " " << trans_Kinect[2] << " " <<
               quat_Kinect.w() << " " << quat_Kinect.x() << " " << quat_Kinect.y() << " " << quat_Kinect.z() << "\n";

    std::cout << message.str();
    //std::cout << "(Pose) : " << std::endl << Pose << std::endl;

    if (myfile.is_open())
        myfile << message.str();



    if (viewer->contains("markers"))
        viewer->removeCoordinateSystem("markers");

    viewer->addCoordinateSystem(0.5,Eigen::Affine3f(Pose),"markers");

}

int
main(int argc, char *argv[])
{
    viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->initCameraParameters ();
    viewer->setBackgroundColor (0.5, 0.5, 0.5);
    viewer->addCoordinateSystem (1.000);


    pcl::ModelCoefficients line_coeff;
    line_coeff.values.resize (6);    // We need 6 values

    int XMin = -3; int XMax = 3;
    int YMin = -1; int YMax = 3;

    line_coeff.values[2] = 0;
    line_coeff.values[5] = 0;

    for (int i = XMin; i <= XMax; i++)
    {
        line_coeff.values[0] = i;
        line_coeff.values[1] = YMin;
        line_coeff.values[3] = 0;
        line_coeff.values[4] = YMax-YMin;
        viewer->addLine (line_coeff, QString("line X%1").arg(i).toStdString());
    }

    for (int i = YMin; i <= YMax; i++)
    {
        line_coeff.values[0] = XMin;
        line_coeff.values[1] = i;
        line_coeff.values[3] = XMax-XMin;
        line_coeff.values[4] = 0;
        viewer->addLine (line_coeff, QString("line Y%1").arg(i).toStdString());
    }


    viewer->setCameraClipDistances(-10, 10);
    viewer->setCameraPosition(0.3, -4.0, 1.5,    // From where I am looking at
                              0.0,  1.0, 1.0,    // Where I am looking at
                              0.0,  0.0, 1.0);   // What is the up orientation

    viewer->addText("SAVE",  0, 10, 30, 0.3, 0.3, 0.3, "SAVE");
    viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);



    loadMatrix("../PCViewer/NexCaveOnKin.txt", _PoseNexCaveOnKin);

    QString myName("dtrackbody@109.171.138.177");

    vrpn_Tracker_Remote *tkr = new vrpn_Tracker_Remote(myName.toStdString().c_str());


    t_user_callback *tc1 = new t_user_callback;
    if (tc1 == NULL)
        fprintf(stderr, "Out of memory\n");

    // Fill in the user-data callback information
    strncpy(tc1->t_name, myName.toStdString().c_str(), sizeof(tc1->t_name));

    // Set up the tracker callback handlers
    tkr->register_change_handler(tc1, handle_tracker_pos_quat, vrpn_ALL_SENSORS);

    while(!viewer->wasStopped())
    {
        tkr->mainloop();
        viewer->spinOnce(1);
        vrpn_SleepMsecs(1); // Sleep for 1ms so we don't eat the CPU
    }

    delete tkr;

    return 0;
}

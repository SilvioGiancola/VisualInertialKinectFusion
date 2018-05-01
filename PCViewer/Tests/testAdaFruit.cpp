#include <Devices/adafruit_uart.h>
#include <QTime>
#include <QApplication>
#include <pcl/visualization/pcl_visualizer.h>

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
    QApplication a(argc, argv);

    std::cout << "start" << std::endl;

    Adafruit_UART ada;

    std::cout << "instance created" << std::endl;

    if (ada.open() != SUCCESS)
    {
        std::cout << "error opening" << std::endl;
        return ERROR;
    }

    std::cout << "opened" << std::endl;

    if (ada.init() != SUCCESS)
    {
        std::cout << "error init" << std::endl;
        return ERROR;
    }
   // ada.setPlayMode(true);

    Eigen::Matrix4f _PoseAdaOnKin;
    loadMatrix("../PCViewer/AdaOnKin.txt", _PoseAdaOnKin);
    std::cout << _PoseAdaOnKin << std::endl;
    Eigen::Quaternionf q = Eigen::Quaternionf(_PoseAdaOnKin.block<3,3>(0,0));
    std::cout << q.matrix() << std::endl;
    ada.setCalibPose(q);
    std::cout << ada.getCalibPose().matrix() << std::endl;


    std::cout << "intialized" << std::endl;

    pcl::visualization::PCLVisualizer viewer("Adafruit");
    viewer.initCameraParameters ();
    viewer.setBackgroundColor (0.5, 0.5, 0.5);
    viewer.addCoordinateSystem (1.000);
    viewer.setCameraClipDistances(-10, 10);
    viewer.setCameraPosition(0.3, -4.0, 1.5,    // From where I am looking at
                             0.0,  1.0, 1.0,    // Where I am looking at
                             0.0,  0.0, 1.0);   // What is the up orientation


    Eigen::Quaternionf * quateig = new Eigen::Quaternionf();
    QTime t;

    viewer.addText("Adafruit:", 0, 0, 30, 1, 1,1,"TIME");
    float i = 0;
    float err = 0;
    while(!viewer.wasStopped())
    {
        i++;
        t.start();

        *quateig = ada.returnPose();

       // if (ada.GetQuat(quateig) != SUCCESS)
        //    err++;

        QString log = QString("Adafruit time: %1 ms (err = %2 %)").arg(t.elapsed()).arg(100*err/i, 0, 'g', 3);

        Eigen::Matrix4f Pose = Eigen::Matrix4f::Identity();
        Pose.block(0,0,3,3) = quateig->matrix();

        if (viewer.contains("markers"))
            viewer.removeCoordinateSystem("markers");
        viewer.addCoordinateSystem(0.5,Eigen::Affine3f(Pose),"markers");
        viewer.updateText(log.toStdString(), 0, 0, "TIME");


        std::cout << log.toStdString() << " " << quateig->w() << " " << quateig->x() << " " << quateig->y() << " " << quateig->z() << std::endl;
        viewer.spinOnce(1);
    }

   // return SUCCESS;

    return SUCCESS;
}

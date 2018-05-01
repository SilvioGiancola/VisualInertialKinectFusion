#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/gpu/containers/initialization.h>
#include <KinFu/kinfu.h>
#include <KinFu/raycaster.h>
#include <KinFu/marching_cubes.h>
#include <pcl/visualization/image_viewer.h>

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

    /* void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    {
        if (!viewer.wasStopped())
            viewer.showCloud (cloud);
    }*/

    void cloud_cb_ (const boost::shared_ptr<openni_wrapper::DepthImage> &depth_wrapper)
    {
        pcl::gpu::PtrStepSz<const unsigned short> depth;

        depth.cols = depth_wrapper->getWidth();
        depth.rows = depth_wrapper->getHeight();
        depth.step = depth.cols * depth.elemSize();

        std::vector<unsigned short> source_depth_data_;
        source_depth_data_.resize(depth.cols * depth.rows);
        depth_wrapper->fillDepthImageRaw(depth.cols, depth.rows, &source_depth_data_[0]);
        depth.data = &source_depth_data_[0];
   //     depth_device_.upload (depth.data, depth.step, depth.rows, depth.cols);

        if (!viewer.wasStopped())
            viewer.showShortImage (depth.data, depth.cols, depth.rows, 0, 5000, true);

    }

    void run ()
    {
        pcl::Grabber* interface = new pcl::OpenNIGrabber();


        // boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
        boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

        interface->registerCallback (f);

        interface->start ();

        while (!viewer.wasStopped())
        {
            boost::this_thread::sleep (boost::posix_time::seconds (1));
        }

        interface->stop ();
    }

    pcl::visualization::ImageViewer viewer;
};

int main ()
{
    SimpleOpenNIViewer v;
    v.run ();
    return 0;
}

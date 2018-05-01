/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */


#define _CRT_SECURE_NO_DEPRECATE


#include "KinFuApp.h"





#include <iostream>
#include <vector>
#include <QtGui>
#include <pcl/console/parse.h>

#include <boost/filesystem.hpp>

#include <KinFu/kinfu.h>
#include <KinFu/raycaster.h>
#include <KinFu/marching_cubes.h>

#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/exceptions.h>

//#include "openni_capture.h"
#include <pcl/visualization/point_cloud_color_handlers.h>
#include "evaluation.h"

#include <Devices/adafruit_uart.h>   ////// ADD ADAFRUIT FOR ROTATION HINT

#include <pcl/common/angles.h>

#include "tsdf_volume.h"
#include "tsdf_volume.hpp"

#include "camera_pose.h"

typedef pcl::ScopeTime ScopeTimeT;

#include "internal.h"

using namespace std;
using namespace pcl;
using namespace pcl::gpu;
using namespace Eigen;
namespace pc = pcl::console;
int width = 640;
int height = 480;
namespace pcl
{
namespace gpu
{
void paint3DView (const KinfuTracker::View& rgb24, KinfuTracker::View& view, float colors_weight = 0.5f);
void mergePointNormal (const DeviceArray<PointXYZ>& cloud, const DeviceArray<Normal>& normals, DeviceArray<PointNormal>& output);
}

namespace visualization
{
//////////////////////////////////////////////////////////////////////////////////////
/** \brief RGB handler class for colors. Uses the data present in the "rgb" or "rgba"
      * fields from an additional cloud as the color at each point.
      * \author Anatoly Baksheev
      * \ingroup visualization
      */
template <typename PointT>
class PointCloudColorHandlerRGBCloud : public PointCloudColorHandler<PointT>
{
    using PointCloudColorHandler<PointT>::capable_;
    using PointCloudColorHandler<PointT>::cloud_;

    typedef typename PointCloudColorHandler<PointT>::PointCloud::ConstPtr PointCloudConstPtr;
    typedef typename pcl::PointCloud<RGB>::ConstPtr RgbCloudConstPtr;

public:
    typedef boost::shared_ptr<PointCloudColorHandlerRGBCloud<PointT> > Ptr;
    typedef boost::shared_ptr<const PointCloudColorHandlerRGBCloud<PointT> > ConstPtr;

    /** \brief Constructor. */
    PointCloudColorHandlerRGBCloud (const PointCloudConstPtr& cloud, const RgbCloudConstPtr& colors)
        : rgb_ (colors)
    {
        cloud_  = cloud;
        capable_ = true;
    }

    /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and
          * the input cloud was given as a valid pointer), false otherwise
          */
    virtual bool
    getColor (vtkSmartPointer<vtkDataArray> &scalars) const
    {
        if (!capable_ || !cloud_)
            return (false);

        if (!scalars)
            scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
        scalars->SetNumberOfComponents (3);

        vtkIdType nr_points = vtkIdType (cloud_->points.size ());
        reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);
        unsigned char* colors = reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->GetPointer (0);

        // Color every point
        if (nr_points != int (rgb_->points.size ()))
            std::fill (colors, colors + nr_points * 3, static_cast<unsigned char> (0xFF));
        else
            for (vtkIdType cp = 0; cp < nr_points; ++cp)
            {
                int idx = cp * 3;
                colors[idx + 0] = rgb_->points[cp].r;
                colors[idx + 1] = rgb_->points[cp].g;
                colors[idx + 2] = rgb_->points[cp].b;
            }
        return (true);
    }

private:
    virtual std::string
    getFieldName () const { return ("additional rgb"); }
    virtual std::string
    getName () const { return ("PointCloudColorHandlerRGBCloud"); }

    RgbCloudConstPtr rgb_;
};
}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
vector<string> getPcdFilesInDir(const string& directory)
{
    namespace fs = boost::filesystem;
    fs::path dir(directory);

    std::cout << "path: " << directory << std::endl;
    if (directory.empty() || !fs::exists(dir) || !fs::is_directory(dir))
        PCL_THROW_EXCEPTION (pcl::IOException, "No valid PCD directory given!\n");

    vector<string> result;
    fs::directory_iterator pos(dir);
    fs::directory_iterator end;

    for(; pos != end ; ++pos)
        if (fs::is_regular_file(pos->status()) )
            if (fs::extension(*pos) == ".pcd")
            {
#if BOOST_FILESYSTEM_VERSION == 3
                result.push_back (pos->path ().string ());
#else
                result.push_back (pos->path ());
#endif
                cout << "added: " << result.back() << endl;
            }

    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SampledScopeTime : public StopWatch
{
    enum { EACH = 33 };
    SampledScopeTime(int& time_ms) : time_ms_(time_ms) {}
    ~SampledScopeTime()
    {
        static int i_ = 0;
        static boost::posix_time::ptime starttime_ = boost::posix_time::microsec_clock::local_time();
        time_ms_ += getTime ();
        if (i_ % EACH == 0 && i_)
        {
            boost::posix_time::ptime endtime_ = boost::posix_time::microsec_clock::local_time();
            cout << "Average frame time = " << time_ms_ / EACH << "ms ( " << 1000.f * EACH / time_ms_ << "fps )"
                 << "( real: " << 1000.f * EACH / (endtime_-starttime_).total_milliseconds() << "fps )"  << endl;
            time_ms_ = 0;
            starttime_ = endtime_;
        }
        ++i_;
    }
private:
    int& time_ms_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
setViewerPose (visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
    Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
    Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
    Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
    viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
            look_at_vector[0], look_at_vector[1], look_at_vector[2],
            up_vector[0], up_vector[1], up_vector[2]);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::Affine3f
getViewerPose (visualization::PCLVisualizer& viewer)
{
    Eigen::Affine3f pose = viewer.getViewerPose();
    Eigen::Matrix3f rotation = pose.linear();

    Matrix3f axis_reorder;
    axis_reorder << 0,  0,  1,
            -1,  0,  0,
            0, -1,  0;

    rotation = rotation * axis_reorder;
    pose.linear() = rotation;
    return pose;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename CloudT> void
writeCloudFile (int format, const CloudT& cloud);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
writePolygonMeshFile (int format, const pcl::PolygonMesh& mesh);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename MergedT, typename PointT>
typename PointCloud<MergedT>::Ptr merge(const PointCloud<PointT>& points, const PointCloud<RGB>& colors)
{
    typename PointCloud<MergedT>::Ptr merged_ptr(new PointCloud<MergedT>());

    pcl::copyPointCloud (points, *merged_ptr);
    for (size_t i = 0; i < colors.size (); ++i)
        merged_ptr->points[i].rgba = colors.points[i].rgba;

    return merged_ptr;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

boost::shared_ptr<pcl::PolygonMesh> convertToMesh(const DeviceArray<PointXYZ>& triangles)
{
    if (triangles.empty())
        return boost::shared_ptr<pcl::PolygonMesh>();

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width  = (int)triangles.size();
    cloud.height = 1;
    triangles.download(cloud.points);

    boost::shared_ptr<pcl::PolygonMesh> mesh_ptr( new pcl::PolygonMesh() );
    pcl::toPCLPointCloud2(cloud, mesh_ptr->cloud);

    mesh_ptr->polygons.resize (triangles.size() / 3);
    for (size_t i = 0; i < mesh_ptr->polygons.size (); ++i)
    {
        pcl::Vertices v;
        v.vertices.push_back(i*3+0);
        v.vertices.push_back(i*3+2);
        v.vertices.push_back(i*3+1);
        mesh_ptr->polygons[i] = v;
    }
    return mesh_ptr;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct CurrentFrameCloudView
{
    CurrentFrameCloudView() : cloud_device_ (height, width), cloud_viewer_ ("Frame Cloud Viewer")
    {
        cloud_ptr_ = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);

        cloud_viewer_.setBackgroundColor (0, 0, 0.15);
        cloud_viewer_.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1);
        cloud_viewer_.addCoordinateSystem (1.0, "global");
        cloud_viewer_.initCameraParameters ();
        cloud_viewer_.setPosition (0, 500);
        cloud_viewer_.setSize (width, height);
        cloud_viewer_.setCameraClipDistances (0.01, 10.01);
    }

    void
    show (const KinfuTracker& kinfu)
    {
        kinfu.getLastFrameCloud (cloud_device_);

        int c;
        cloud_device_.download (cloud_ptr_->points, c);
        cloud_ptr_->width = cloud_device_.cols ();
        cloud_ptr_->height = cloud_device_.rows ();
        cloud_ptr_->is_dense = false;

        cloud_viewer_.removeAllPointClouds ();
        cloud_viewer_.addPointCloud<PointXYZ>(cloud_ptr_);
        cloud_viewer_.spinOnce ();
    }

    void
    setViewerPose (const Eigen::Affine3f& viewer_pose) {
        ::setViewerPose (cloud_viewer_, viewer_pose);
    }

    PointCloud<PointXYZ>::Ptr cloud_ptr_;
    DeviceArray2D<PointXYZ> cloud_device_;
    visualization::PCLVisualizer cloud_viewer_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct ImageView
{
    ImageView(int viz) : viz_(viz), paint_image_ (false), accumulate_views_ (false)
    {
        if (viz_)
        {
            viewerScene_ = pcl::visualization::ImageViewer::Ptr(new pcl::visualization::ImageViewer);
            viewerDepth_ = pcl::visualization::ImageViewer::Ptr(new pcl::visualization::ImageViewer);

            viewerScene_->setWindowTitle ("View3D from ray tracing");
            viewerScene_->setPosition (0, 0);
            viewerDepth_->setWindowTitle ("Kinect Depth stream");
            viewerDepth_->setPosition (width, 0);
            //viewerColor_.setWindowTitle ("Kinect RGB stream");
        }
    }

    void
    showScene (KinfuTracker& kinfu, const PtrStepSz<const KinfuTracker::PixelRGB>& rgb24, bool registration, Eigen::Affine3f* pose_ptr = 0)
    {
        if (pose_ptr)
        {
            raycaster_ptr_->run(kinfu.volume(), *pose_ptr);
            raycaster_ptr_->generateSceneView(view_device_);
        }
        else
            kinfu.getImage (view_device_);

        if (paint_image_ && registration && !pose_ptr)
        {
            colors_device_.upload (rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);
            paint3DView (colors_device_, view_device_);
        }


        int cols;
        view_device_.download (view_host_, cols);
        if (viz_)
            viewerScene_->showRGBImage (reinterpret_cast<unsigned char*> (&view_host_[0]), view_device_.cols (), view_device_.rows ());

        //viewerColor_.showRGBImage ((unsigned char*)&rgb24.data, rgb24.cols, rgb24.rows);

#ifdef HAVE_OPENCV
        if (accumulate_views_)
        {
            views_.push_back (cv::Mat ());
            cv::cvtColor (cv::Mat (height, width, CV_8UC3, (void*)&view_host_[0]), views_.back (), CV_RGB2GRAY);
            //cv::copy(cv::Mat(height, width, CV_8UC3, (void*)&view_host_[0]), views_.back());
        }
#endif
    }

    void
    showDepth (const PtrStepSz<const unsigned short>& depth)
    {
        if (viz_)
            viewerDepth_->showShortImage (depth.data, depth.cols, depth.rows, 0, 5000, true);
    }

    void
    showGeneratedDepth (KinfuTracker& kinfu, const Eigen::Affine3f& pose)
    {
        raycaster_ptr_->run(kinfu.volume(), pose);
        raycaster_ptr_->generateDepthImage(generated_depth_);

        int c;
        vector<unsigned short> data;
        generated_depth_.download(data, c);

        if (viz_)
            viewerDepth_->showShortImage (&data[0], generated_depth_.cols(), generated_depth_.rows(), 0, 5000, true);
    }

    void
    toggleImagePaint()
    {
        paint_image_ = !paint_image_;
        cout << "Paint image: " << (paint_image_ ? "On   (requires registration mode)" : "Off") << endl;
    }

    void
    setWindowName(std::string windowname)
    {
        viewerScene_->setWindowTitle (windowname + " : View3D from ray tracing");
        viewerDepth_->setWindowTitle (windowname + " : Kinect Depth stream");
    }

    int viz_;
    bool paint_image_;
    bool accumulate_views_;

    visualization::ImageViewer::Ptr viewerScene_;
    visualization::ImageViewer::Ptr viewerDepth_;
    //visualization::ImageViewer viewerColor_;

    KinfuTracker::View view_device_;
    KinfuTracker::View colors_device_;
    vector<KinfuTracker::PixelRGB> view_host_;

    RayCaster::Ptr raycaster_ptr_;

    KinfuTracker::DepthMap generated_depth_;

#ifdef HAVE_OPENCV
    vector<cv::Mat> views_;
#endif
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SceneCloudView
{
    enum { GPU_Connected6 = 0, CPU_Connected6 = 1, CPU_Connected26 = 2 };

    SceneCloudView(int viz) : viz_(viz), extraction_mode_ (GPU_Connected6), compute_normals_ (false), valid_combined_ (false), cube_added_(false)
    {
        cloud_ptr_ = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
        normals_ptr_ = PointCloud<Normal>::Ptr (new PointCloud<Normal>);
        combined_ptr_ = PointCloud<PointNormal>::Ptr (new PointCloud<PointNormal>);
        point_colors_ptr_ = PointCloud<RGB>::Ptr (new PointCloud<RGB>);

        if (viz_)
        {
            cloud_viewer_ = pcl::visualization::PCLVisualizer::Ptr( new pcl::visualization::PCLVisualizer("Scene Cloud Viewer") );

            cloud_viewer_->setBackgroundColor (0, 0, 0);
            cloud_viewer_->addCoordinateSystem (1.0, "global");
            cloud_viewer_->initCameraParameters ();
            cloud_viewer_->setPosition (0, 500);
            cloud_viewer_->setSize (width, height);
            cloud_viewer_->setCameraClipDistances (0.01, 10.01);

            cloud_viewer_->addText ("H: print help", 2, 15, 20, 34, 135, 246);
        }
    }

    void
    show (KinfuTracker& kinfu, bool integrate_colors)
    {
        viewer_pose_ = kinfu.getCameraPose();

        ScopeTimeT time ("PointCloud Extraction");
        cout << "\nGetting cloud... " << flush;

        valid_combined_ = false;

        if (extraction_mode_ != GPU_Connected6)     // So use CPU
        {
            kinfu.volume().fetchCloudHost (*cloud_ptr_, extraction_mode_ == CPU_Connected26);
        }
        else
        {
            DeviceArray<PointXYZ> extracted = kinfu.volume().fetchCloud (cloud_buffer_device_);

            if (compute_normals_)
            {
                kinfu.volume().fetchNormals (extracted, normals_device_);
                pcl::gpu::mergePointNormal (extracted, normals_device_, combined_device_);
                combined_device_.download (combined_ptr_->points);
                combined_ptr_->width = (int)combined_ptr_->points.size ();
                combined_ptr_->height = 1;

                valid_combined_ = true;
            }
            else
            {
                extracted.download (cloud_ptr_->points);
                cloud_ptr_->width = (int)cloud_ptr_->points.size ();
                cloud_ptr_->height = 1;
            }

            if (integrate_colors)
            {
                kinfu.colorVolume().fetchColors(extracted, point_colors_device_);
                point_colors_device_.download(point_colors_ptr_->points);
                point_colors_ptr_->width = (int)point_colors_ptr_->points.size ();
                point_colors_ptr_->height = 1;
            }
            else
                point_colors_ptr_->points.clear();
        }
        size_t points_size = valid_combined_ ? combined_ptr_->points.size () : cloud_ptr_->points.size ();
        cout << "Done.  Cloud size: " << points_size / 1000 << "K" << endl;

        if (viz_)
        {
            cloud_viewer_->removeAllPointClouds ();
            if (valid_combined_)
            {
                visualization::PointCloudColorHandlerRGBCloud<PointNormal> rgb(combined_ptr_, point_colors_ptr_);
                cloud_viewer_->addPointCloud<PointNormal> (combined_ptr_, rgb, "Cloud");
                cloud_viewer_->addPointCloudNormals<PointNormal>(combined_ptr_, 50);
            }
            else
            {
                visualization::PointCloudColorHandlerRGBCloud<PointXYZ> rgb(cloud_ptr_, point_colors_ptr_);
                cloud_viewer_->addPointCloud<PointXYZ> (cloud_ptr_, rgb);
            }
        }
    }

    void
    toggleCube(const Eigen::Vector3f& size)
    {
        if (!viz_)
            return;

        if (cube_added_)
            cloud_viewer_->removeShape("cube");
        else
            cloud_viewer_->addCube(size*0.5, Eigen::Quaternionf::Identity(), size(0), size(1), size(2));

        cube_added_ = !cube_added_;
    }

    void
    toggleExtractionMode ()
    {
        extraction_mode_ = (extraction_mode_ + 1) % 3;

        switch (extraction_mode_)
        {
        case 0: cout << "Cloud exctraction mode: GPU, Connected-6" << endl; break;
        case 1: cout << "Cloud exctraction mode: CPU, Connected-6    (requires a lot of memory)" << endl; break;
        case 2: cout << "Cloud exctraction mode: CPU, Connected-26   (requires a lot of memory)" << endl; break;
        }
        ;
    }

    void
    toggleNormals ()
    {
        compute_normals_ = !compute_normals_;
        cout << "Compute normals: " << (compute_normals_ ? "On" : "Off") << endl;
    }

    void
    clearClouds (bool print_message = false)
    {
        if (!viz_)
            return;

        cloud_viewer_->removeAllPointClouds ();
        cloud_ptr_->points.clear ();
        normals_ptr_->points.clear ();
        if (print_message)
            cout << "Clouds/Meshes were cleared" << endl;
    }

    void
    showMesh(KinfuTracker& kinfu, bool /*integrate_colors*/)
    {
        if (!viz_)
            return;

        ScopeTimeT time ("Mesh Extraction");
        cout << "\nGetting mesh... " << flush;

        if (!marching_cubes_)
            marching_cubes_ = MarchingCubes::Ptr( new MarchingCubes() );

        DeviceArray<PointXYZ> triangles_device = marching_cubes_->run(kinfu.volume(), triangles_buffer_device_);
        mesh_ptr_ = convertToMesh(triangles_device);

        cloud_viewer_->removeAllPointClouds ();
        if (mesh_ptr_)
            cloud_viewer_->addPolygonMesh(*mesh_ptr_);

        cout << "Done.  Triangles number: " << triangles_device.size() / MarchingCubes::POINTS_PER_TRIANGLE / 1000 << "K" << endl;
    }

    int viz_;
    int extraction_mode_;
    bool valid_combined_;
    bool cube_added_;
    bool compute_normals_;

    Eigen::Affine3f viewer_pose_;

    visualization::PCLVisualizer::Ptr cloud_viewer_;

    PointCloud<PointXYZ>::Ptr cloud_ptr_;
    PointCloud<Normal>::Ptr normals_ptr_;

    DeviceArray<PointXYZ> cloud_buffer_device_;
    DeviceArray<Normal> normals_device_;

    PointCloud<PointNormal>::Ptr combined_ptr_;
    DeviceArray<PointNormal> combined_device_;

    DeviceArray<RGB> point_colors_device_;
    PointCloud<RGB>::Ptr point_colors_ptr_;

    MarchingCubes::Ptr marching_cubes_;
    DeviceArray<PointXYZ> triangles_buffer_device_;

    boost::shared_ptr<pcl::PolygonMesh> mesh_ptr_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename CloudPtr> void
writeCloudFile (int format, const CloudPtr& cloud_prt)
{
    if (format == KinFuApp::PCD_BIN)
    {
        cout << "Saving point cloud to 'cloud_bin.pcd' (binary)... " << flush;
        pcl::io::savePCDFile ("cloud_bin.pcd", *cloud_prt, true);
    }
    else
        if (format == KinFuApp::PCD_ASCII)
        {
            cout << "Saving point cloud to 'cloud.pcd' (ASCII)... " << flush;
            pcl::io::savePCDFile ("cloud.pcd", *cloud_prt, false);
        }
        else   /* if (format == KinFuApp::PLY) */
        {
            cout << "Saving point cloud to 'cloud.ply' (ASCII)... " << flush;
            pcl::io::savePLYFileASCII ("cloud.ply", *cloud_prt);

        }
    cout << "Done" << endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
writePolygonMeshFile (int format, const pcl::PolygonMesh& mesh)
{
    if (format == KinFuApp::MESH_PLY)
    {
        cout << "Saving mesh to to 'mesh.ply'... " << flush;
        pcl::io::savePLYFile("mesh.ply", mesh);
    }
    else /* if (format == KinFuApp::MESH_VTK) */
    {
        cout << "Saving mesh to to 'mesh.vtk'... " << flush;
        pcl::io::saveVTKFile("mesh.vtk", mesh);
    }
    cout << "Done" << endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int
print_cli_help ()
{
    cout << "\nKinFu parameters:" << endl;
    cout << "    --help, -h                                         : print help" << endl;
    cout << "    --viz <bool> (default:1)                           : enable visualisation" << endl;
    cout << "    --volume_size <size_in_meters> (default:3)         : define integration volume size" << endl;
    cout << "    --resolution <voxel_number_per_side> (default:512) : define voxel resolution" << endl;
    cout << "    --icp1 <iteration number> (default:10)             : define coarsest icp iteration number" << endl;
    cout << "    --icp2 <iteration number> (default:5)              : define intermediate icp iteration number" << endl;
    cout << "    --icp3 <iteration number> (default:4)              : define fine icp iteration number" << endl;
    cout << "    --IMU <bool> (default:0)                           : enable IMU" << endl;

    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <QApplication>
int
main (int argc, char* argv[])
{
    QApplication qapp(argc, argv);

    // Print cli help
    if (pc::find_switch (argc, argv, "--help") || pc::find_switch (argc, argv, "-h"))
        return print_cli_help ();

    // Choose your GPU
    int device = 0;
    pc::parse_argument (argc, argv, "-gpu", device);
    pcl::gpu::setDevice (device);
    pcl::gpu::printShortCudaDeviceInfo (device);

    if (checkIfPreFermiGPU(device))
        return cout << endl << "Kinfu is supported only for Fermi and Kepler arhitectures. It is not even compiled for pre-Fermi by default. Exiting..." << endl, 1;


    // Set the grabber
    boost::shared_ptr<pcl::Grabber> capture;

    bool triggered_capture = false;
    bool pcd_input = false;

    std::string eval_folder, match_file, openni_device, oni_file, pcd_dir;
    try
    {
        // OpenNI device
        if (pc::parse_argument (argc, argv, "-dev", openni_device) > 0)
        {
            capture.reset (new pcl::OpenNIGrabber (openni_device));
        }
        // OpenNI file
        else if (pc::parse_argument (argc, argv, "-oni", oni_file) > 0)
        {
            triggered_capture = true;
            bool repeat = false; // Only run ONI file once
            capture.reset (new pcl::ONIGrabber (oni_file, repeat, ! triggered_capture));
        }
        // PCD files device (skip frames, not robust)
        else if (pc::parse_argument (argc, argv, "-pcd", pcd_dir) > 0)
        {
            float fps_pcd = 15.0f;
            pc::parse_argument (argc, argv, "-pcd_fps", fps_pcd);

            vector<string> pcd_files = getPcdFilesInDir(pcd_dir);

            // Sort the read files by name
            sort (pcd_files.begin (), pcd_files.end ());
            capture.reset (new pcl::PCDGrabber<pcl::PointXYZRGBA> (pcd_files, fps_pcd, false));
            triggered_capture = true;
            pcd_input = true;
        }
        // Default: OpenNI device
        else
        {
            capture.reset( new pcl::OpenNIGrabber() );
        }
    }
    catch (const pcl::PCLException& /*e*/) { return cout << "Can't open depth source" << endl, -1; }


    // Initial values for KinFu App
    float volume_size = 3.f; // 3-meters side TSDF cube
    int resolution = 512; // 512 voxel resolution along each axis
    int icp1 = 10, icp2 = 5, icp3 = 4;  // (10,5,4) coarse to fine ICP iterations
    int visualization = 1, IMU = 0; // Enable visualization and disable IMU

    pc::parse_argument (argc, argv, "--volume_size", volume_size);
    pc::parse_argument (argc, argv, "--icp1", icp1);
    pc::parse_argument (argc, argv, "--icp2", icp2);
    pc::parse_argument (argc, argv, "--icp3", icp3);
    pc::parse_argument (argc, argv, "--viz", visualization);
    pc::parse_argument (argc, argv, "--IMU", IMU);
    pc::parse_argument (argc, argv, "--resolution", resolution);

    // Save trajectory
    std::string camera_pose_file;
    boost::shared_ptr<CameraPoseProcessor> pose_processor;
    if (pc::parse_argument (argc, argv, "-save_pose", camera_pose_file) && camera_pose_file.size () > 0)
    {
        pose_processor.reset (new CameraPoseWriter (camera_pose_file));
    }


    // Instantiating
    KinFuApp app (*capture, volume_size, icp1, icp2, icp3, resolution, visualization, IMU, pose_processor);


    // executing
    try { app.startMainLoop (triggered_capture); }
    catch (const pcl::PCLException& ) { cout << "PCLException" << endl; }
    catch (const std::bad_alloc& ) { cout << "Bad alloc" << endl; }
    catch (const std::exception& ) { cout << "Exception" << endl; }


    return 0;
}

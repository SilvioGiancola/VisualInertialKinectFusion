
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/gpu/containers/initialization.h>

#include "camera_pose.h"

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

struct KinFuApp
{
    enum { PCD_BIN = 1, PCD_ASCII = 2, PLY = 3, MESH_PLY = 7, MESH_VTK = 8 };

    KinFuApp(pcl::Grabber& source, float volume_size, int icp1, int icp2, int icp3, int resolution, int viz, int dev, boost::shared_ptr<CameraPoseProcessor> pose_processor=boost::shared_ptr<CameraPoseProcessor> ()) :
        exit_ (false), scan_ (false), scan_mesh_(false), scan_volume_ (false), independent_camera_ (false), integrate_(true), registration_ (false), integrate_colors_ (false), focal_length_(-1.f),
        volume_size_(volume_size), resolution_(resolution), icp1_(icp1), icp2_(icp2), icp3_(icp3),
        capture_ (source), image_view_(viz), time_ms_(0), viz_(viz), useAdafruit_ (dev),
        scene_cloud_view_(viz), pose_processor_ (pose_processor)
    {
        // Open an Adafruit sensor
        if (myAda.open() == SUCCESS)
            myAda.init();

        myAda.setPlayMode(false);

        // Open a Calibration for the Sensor
        Eigen::Matrix4f _PoseAdaOnKin = Eigen::Matrix4f::Identity();
        loadMatrix("../PCViewer/KinFu/PoseAdafruit.txt", _PoseAdaOnKin);
        std::cout << _PoseAdaOnKin << std::endl;
        Eigen::Quaternionf q2 = Eigen::Quaternionf(_PoseAdaOnKin.block<3,3>(0,0));
        std::cout << q2.matrix() << std::endl;

        myAda.setCalibPose(q2);





        // Set KinFu Parameters
        kinfu_.setICPiteration(icp1_,icp2_,icp3_);
        //   kinfu_.setICPiteration(3,2,2);


        kinfu_.tsdf_volume_ =  TsdfVolume::Ptr( new TsdfVolume(Vector3i(resolution_, resolution_, resolution_)) );


        kinfu_.volume().setSize (Vector3f::Constant (volume_size_));


        kinfu_.setInitalCameraPose (Eigen::Translation3f (volume_size_/2, volume_size_/2, volume_size_/2) * Eigen::AngleAxisf (Eigen::Matrix3f::Identity ()));
        //  kinfuDEV_.setInitalCameraPose (Eigen::Translation3f (1.5f, 1.5f, 1.5f) * Eigen::AngleAxisf (myAda.returnPose().matrix()));

        //  toggleRestart();
        kinfu_.volume().setTsdfTruncDist (0.030f/*meters*/);
        //  kinfu_.setIcpCorespFilteringParams (0.1f/*meters*/, sin ( pcl::deg2rad(5.f) ));
        kinfu_.setDepthTruncationForICP(5.f/*meters*/);
        kinfu_.setCameraMovementThreshold(0.001f);

        toggleAdafruit(useAdafruit_);
        toggleRestart();



        //Init KinfuApp
        tsdf_cloud_ptr_ = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
        image_view_.raycaster_ptr_ = RayCaster::Ptr( new RayCaster(kinfu_.rows (), kinfu_.cols ()) );

        if (viz_)
        {
            scene_cloud_view_.cloud_viewer_->registerKeyboardCallback (keyboard_callback, (void*)this);
            image_view_.viewerScene_->registerKeyboardCallback (keyboard_callback, (void*)this);
            image_view_.viewerDepth_->registerKeyboardCallback (keyboard_callback, (void*)this);
            image_view_.setWindowName("No Device");


            // scene_cloud_view_.toggleCube(volume_size);
        }


    }

    ~KinFuApp()
    {
        myAda.close();
        //  if (evaluation_ptr_)
        //     evaluation_ptr_->saveAllPoses(kinfu_);
    }

    void
    initCurrentFrameView ()
    {
        current_frame_cloud_view_ = boost::shared_ptr<CurrentFrameCloudView>(new CurrentFrameCloudView ());
        current_frame_cloud_view_->cloud_viewer_.registerKeyboardCallback (keyboard_callback, (void*)this);
        current_frame_cloud_view_->setViewerPose (kinfu_.getCameraPose ());


    }

    void
    initRegistration ()
    {
        registration_ = capture_.providesCallback<pcl::ONIGrabber::sig_cb_openni_image_depth_image> ();
        cout << "Registration mode: " << (registration_ ? "On" : "Off (not supported by source)") << endl;
        if (registration_)
        {
            //  kinfu_.setDepthIntrinsics(365.907, 365.907, 261.007, 206.482);
            kinfu_.setDepthIntrinsics(KINFU_DEFAULT_RGB_FOCAL_X, KINFU_DEFAULT_RGB_FOCAL_Y);
            //  kinfuDEV_.setDepthIntrinsics(KINFU_DEFAULT_RGB_FOCAL_X, KINFU_DEFAULT_RGB_FOCAL_Y);
        }
    }

    void
    setDepthIntrinsics(std::vector<float> depth_intrinsics)
    {
        float fx = depth_intrinsics[0];
        float fy = depth_intrinsics[1];

        if (depth_intrinsics.size() == 4)
        {
            float cx = depth_intrinsics[2];
            float cy = depth_intrinsics[3];
            kinfu_.setDepthIntrinsics(fx, fy, cx, cy);
            //  kinfuDEV_.setDepthIntrinsics(fx, fy, cx, cy);
            cout << "Depth intrinsics changed to fx="<< fx << " fy=" << fy << " cx=" << cx << " cy=" << cy << endl;
        }
        else {
            kinfu_.setDepthIntrinsics(fx, fy);
            //     kinfuDEV_.setDepthIntrinsics(fx, fy);
            cout << "Depth intrinsics changed to fx="<< fx << " fy=" << fy << endl;
        }
    }

    void
    toggleColorIntegration()
    {
        if(registration_)
        {
            const int max_color_integration_weight = 2;
            kinfu_.initColorIntegration(max_color_integration_weight);
            //   kinfuDEV_.initColorIntegration(max_color_integration_weight);
            integrate_colors_ = true;
        }
        cout << "Color integration: " << (integrate_colors_ ? "On" : "Off ( requires registration mode )") << endl;
    }

    void
    enableTruncationScaling()
    {
        kinfu_.volume().setTsdfTruncDist (kinfu_.volume().getSize()(0) / 100.0f);
        //    kinfuDEV_.volume().setTsdfTruncDist (kinfuDEV_.volume().getSize()(0) / 100.0f);
    }

    void
    toggleIndependentCamera()
    {
        independent_camera_ = !independent_camera_;
        cout << "Camera mode: " << (independent_camera_ ?  "Independent" : "Bound to Kinect pose") << endl;
    }

    void
    toggleIntegrate()
    {
        integrate_ = !integrate_;
        cout << "integration: " << (integrate_ ?  "yes" : "no") << endl;
    }

    void
    toggleAdafruit (bool value)
    {
        useAdafruit_ = value;
        cout << "useAdafruit: " << (useAdafruit_ ? "On" : "Off") << endl;
        if (useAdafruit_)
        {
            kinfu_.setInitalCameraPose (Eigen::Translation3f (volume_size_/2, volume_size_/2, volume_size_/2) * Eigen::AngleAxisf (myAda.returnPose().matrix()));
            //  kinfu_.setInitalCameraPose (Eigen::Translation3f (volume_size_/2, volume_size_/2, volume_size_/2) * Eigen::AngleAxisf (Eigen::Matrix3f::Identity ()));
            kinfu_.setIcpCorespFilteringParams (0.5f/*meters*/, sin ( pcl::deg2rad(10.f) ));
         //   kinfu_.setICPiteration(3,2,2);
            //    kinfu_.setIcpCorespFilteringParams (0.1f/*meters*/, sin ( pcl::deg2rad(5.f) ));
            //    kinfu_.setICPiteration(10,5,4);
        }
        else
        {
            kinfu_.setInitalCameraPose (Eigen::Translation3f (volume_size_/2, volume_size_/2, volume_size_/2) * Eigen::AngleAxisf (Eigen::Matrix3f::Identity ()));
            kinfu_.setIcpCorespFilteringParams (0.5f/*meters*/, sin ( pcl::deg2rad(20.f) ));
          //  kinfu_.setICPiteration(10,5,4);
        }
       }

    void
    toggleRestart()
    {
        Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
        Eigen::Vector3f t = Vector3f (volume_size_/2, volume_size_/2, 0);

        if (useAdafruit_)
            q = myAda.returnPose();

        kinfu_.setInitalCameraPose (Eigen::Translation3f(t) * Eigen::AngleAxisf (q.matrix()));


        // pose = ;
        //    kinfuDEV_.setInitalCameraPose (pose);
    }

    int iter =0;
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    void execute(const PtrStepSz<const unsigned short>& depth, const PtrStepSz<const KinfuTracker::PixelRGB>& rgb24, bool has_data)
    {
        bool has_image = false;
        //   bool has_imageDEV = false;

        if (has_data)
        {
            depth_device_.upload (depth.data, depth.step, depth.rows, depth.cols);
            if (integrate_colors_)
            {
                image_view_.colors_device_.upload (rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);
                //        image_viewDEV_.colors_device_.upload (rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);
            }
            SampledScopeTime fps(time_ms_);


            //run kinfu algorithm
            if (integrate_colors_)
            {
                has_image = kinfu_ (depth_device_, image_view_.colors_device_);
            }
            else
            {

                if (useAdafruit_)
                {
                    q = myAda.returnPose();
                    //   cout << q.w() << "  /  " << q.x() << "  /  " << q.y() << "  /  " << q.z() << endl;
                    Eigen::Affine3f Aff = Eigen::Affine3f(q.matrix());
                    has_image = kinfu_ (depth_device_, depth, &Aff);



                }
                else
                {
                    has_image = kinfu_ (depth_device_, depth);

                }

            }
            iter++;
            if (!has_image && iter>1)
            {
                iter = 0;
                //      toggleRestart();
            }


            // process camera pose
            if (pose_processor_)
                pose_processor_->processPose (kinfu_.getCameraPose ());

            /*  if (pose_processorDEV_)
                pose_processorDEV_->processPose (kinfuDEV_.getCameraPose ());*/


            image_view_.showDepth (depth);
            //   image_viewDEV_.showDepth (depth);
            //image_view_.showGeneratedDepth(kinfu_, kinfu_.getCameraPose());
        }

        if (scan_)
        {
            scan_ = false;
            scene_cloud_view_.show (kinfu_, integrate_colors_);
            // scene_cloud_viewDEV_.show (kinfuDEV_, integrate_colors_);

            if (scan_volume_)
            {
                cout << "Downloading TSDF volume from device ... " << flush;
                kinfu_.volume().downloadTsdfAndWeighs (tsdf_volume_.volumeWriteable (), tsdf_volume_.weightsWriteable ());
                tsdf_volume_.setHeader (Eigen::Vector3i (kinfu_.getResolution(), kinfu_.getResolution(), kinfu_.getResolution()), kinfu_.volume().getSize ());
                cout << "done [" << tsdf_volume_.size () << " voxels]" << endl << endl;

                cout << "Converting volume to TSDF cloud ... " << flush;
                tsdf_volume_.convertToTsdfCloud (tsdf_cloud_ptr_);
                cout << "done [" << tsdf_cloud_ptr_->size () << " points]" << endl << endl;



                /*   cout << "Downloading TSDF volume from device ... " << flush;
                kinfuDEV_.volume().downloadTsdfAndWeighs (tsdf_volumeDEV_.volumeWriteable (), tsdf_volumeDEV_.weightsWriteable ());
                tsdf_volumeDEV_.setHeader (Eigen::Vector3i (kinfuDEV_.getResolution(), kinfuDEV_.getResolution(), kinfuDEV_.getResolution()), kinfuDEV_.volume().getSize ());
                cout << "done [" << tsdf_volumeDEV_.size () << " voxels]" << endl << endl;

                cout << "Converting volume to TSDF cloud ... " << flush;
                tsdf_volumeDEV_.convertToTsdfCloud (tsdf_cloud_ptrDEV_);
                cout << "done [" << tsdf_cloud_ptrDEV_->size () << " points]" << endl << endl;*/
            }
            else
                cout << "[!] tsdf volume download is disabled" << endl << endl;
        }

        if (scan_mesh_)
        {
            scan_mesh_ = false;
            scene_cloud_view_.showMesh(kinfu_, integrate_colors_);
            //scene_cloud_viewDEV_.showMesh(kinfuDEV_, integrate_colors_);
        }

        if (viz_ && has_image)
        {
            Eigen::Affine3f viewer_pose = getViewerPose(*scene_cloud_view_.cloud_viewer_);
            image_view_.showScene (kinfu_, rgb24, registration_, independent_camera_ ? &viewer_pose : 0);

            //     viewer_pose = getViewerPose(*scene_cloud_viewDEV_.cloud_viewer_);
            //     image_viewDEV_.showScene (kinfuDEV_, rgb24, registration_, independent_camera_ ? &viewer_pose : 0);
        }

        if (current_frame_cloud_view_)
            current_frame_cloud_view_->show (kinfu_);

        // if (current_frame_cloud_viewDEV_)
        //       current_frame_cloud_viewDEV_->show (kinfuDEV_);


        if (viz_ && !independent_camera_)
            setViewerPose (*scene_cloud_view_.cloud_viewer_, kinfu_.getCameraPose());

        //    if (viz_ && !independent_camera_)
        //          setViewerPose (*scene_cloud_viewDEV_.cloud_viewer_, kinfuDEV_.getCameraPose());
    }

    void source_cb1_device(const boost::shared_ptr<openni_wrapper::DepthImage>& depth_wrapper)
    {
        {
            boost::mutex::scoped_try_lock lock(data_ready_mutex_);
            if (exit_ || !lock)
                return;

            depth_.cols = depth_wrapper->getWidth();
            depth_.rows = depth_wrapper->getHeight();
            depth_.step = depth_.cols * depth_.elemSize();

            source_depth_data_.resize(depth_.cols * depth_.rows);
            depth_wrapper->fillDepthImageRaw(depth_.cols, depth_.rows, &source_depth_data_[0]);
            depth_.data = &source_depth_data_[0];
        }
        data_ready_cond_.notify_one();
    }

    void source_cb2_device(const boost::shared_ptr<openni_wrapper::Image>& image_wrapper, const boost::shared_ptr<openni_wrapper::DepthImage>& depth_wrapper, float)
    {
        {
            boost::mutex::scoped_try_lock lock(data_ready_mutex_);
            if (exit_ || !lock)
                return;

            depth_.cols = depth_wrapper->getWidth();
            depth_.rows = depth_wrapper->getHeight();
            depth_.step = depth_.cols * depth_.elemSize();

            source_depth_data_.resize(depth_.cols * depth_.rows);
            depth_wrapper->fillDepthImageRaw(depth_.cols, depth_.rows, &source_depth_data_[0]);
            depth_.data = &source_depth_data_[0];

            rgb24_.cols = image_wrapper->getWidth();
            rgb24_.rows = image_wrapper->getHeight();
            rgb24_.step = rgb24_.cols * rgb24_.elemSize();

            source_image_data_.resize(rgb24_.cols * rgb24_.rows);
            image_wrapper->fillRGB(rgb24_.cols, rgb24_.rows, (unsigned char*)&source_image_data_[0]);
            rgb24_.data = &source_image_data_[0];
        }
        data_ready_cond_.notify_one();
    }


    void source_cb1_oni(const boost::shared_ptr<openni_wrapper::DepthImage>& depth_wrapper)
    {
        {
            boost::mutex::scoped_lock lock(data_ready_mutex_);
            if (exit_)
                return;

            depth_.cols = depth_wrapper->getWidth();
            depth_.rows = depth_wrapper->getHeight();
            depth_.step = depth_.cols * depth_.elemSize();

            source_depth_data_.resize(depth_.cols * depth_.rows);
            depth_wrapper->fillDepthImageRaw(depth_.cols, depth_.rows, &source_depth_data_[0]);
            depth_.data = &source_depth_data_[0];
        }
        data_ready_cond_.notify_one();
    }

    void source_cb2_oni(const boost::shared_ptr<openni_wrapper::Image>& image_wrapper, const boost::shared_ptr<openni_wrapper::DepthImage>& depth_wrapper, float)
    {
        {
            boost::mutex::scoped_lock lock(data_ready_mutex_);
            if (exit_)
                return;

            depth_.cols = depth_wrapper->getWidth();
            depth_.rows = depth_wrapper->getHeight();
            depth_.step = depth_.cols * depth_.elemSize();

            source_depth_data_.resize(depth_.cols * depth_.rows);
            depth_wrapper->fillDepthImageRaw(depth_.cols, depth_.rows, &source_depth_data_[0]);
            depth_.data = &source_depth_data_[0];

            rgb24_.cols = image_wrapper->getWidth();
            rgb24_.rows = image_wrapper->getHeight();
            rgb24_.step = rgb24_.cols * rgb24_.elemSize();

            source_image_data_.resize(rgb24_.cols * rgb24_.rows);
            image_wrapper->fillRGB(rgb24_.cols, rgb24_.rows, (unsigned char*)&source_image_data_[0]);
            rgb24_.data = &source_image_data_[0];
        }
        data_ready_cond_.notify_one();
    }

    void
    source_cb3 (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & DC3)
    {
        {
            boost::mutex::scoped_try_lock lock(data_ready_mutex_);
            if (exit_ || !lock)
                return;
            int width  = DC3->width;
            int height = DC3->height;
            depth_.cols = width;
            depth_.rows = height;
            depth_.step = depth_.cols * depth_.elemSize ();
            source_depth_data_.resize (depth_.cols * depth_.rows);

            rgb24_.cols = width;
            rgb24_.rows = height;
            rgb24_.step = rgb24_.cols * rgb24_.elemSize ();
            source_image_data_.resize (rgb24_.cols * rgb24_.rows);

            unsigned char *rgb    = (unsigned char *)  &source_image_data_[0];
            unsigned short *depth = (unsigned short *) &source_depth_data_[0];

            for (int i=0; i < width*height; i++)
            {
                PointXYZRGBA pt = DC3->at (i);
                rgb[3*i +0] = pt.r;
                rgb[3*i +1] = pt.g;
                rgb[3*i +2] = pt.b;
                depth[i]    = pt.z/0.001;
            }
            rgb24_.data = &source_image_data_[0];
            depth_.data = &source_depth_data_[0];
        }
        data_ready_cond_.notify_one ();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
    startMainLoop (bool triggered_capture)
    {
        using namespace openni_wrapper;
        typedef boost::shared_ptr<DepthImage> DepthImagePtr;
        typedef boost::shared_ptr<Image> ImagePtr;

        boost::function<void (const ImagePtr&, const DepthImagePtr&, float constant)> func1_dev = boost::bind (&KinFuApp::source_cb2_device, this, _1, _2, _3);
        boost::function<void (const DepthImagePtr&)> func2_dev = boost::bind (&KinFuApp::source_cb1_device, this, _1);

        boost::function<void (const ImagePtr&, const DepthImagePtr&, float constant)> func1_oni = boost::bind (&KinFuApp::source_cb2_oni, this, _1, _2, _3);
        boost::function<void (const DepthImagePtr&)> func2_oni = boost::bind (&KinFuApp::source_cb1_oni, this, _1);

        bool is_oni = dynamic_cast<pcl::ONIGrabber*>(&capture_) != 0;
        boost::function<void (const ImagePtr&, const DepthImagePtr&, float constant)> func1 = is_oni ? func1_oni : func1_dev;
        boost::function<void (const DepthImagePtr&)> func2 = is_oni ? func2_oni : func2_dev;

        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) > func3 = boost::bind (&KinFuApp::source_cb3, this, _1);

        bool need_colors = integrate_colors_ || registration_;
        if ( pcd_source_ && !capture_.providesCallback<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)>() )
        {
            std::cout << "grabber doesn't provide pcl::PointCloud<pcl::PointXYZRGBA> callback !\n";
        }
        boost::signals2::connection c = pcd_source_? capture_.registerCallback (func3) : need_colors ? capture_.registerCallback (func1) : capture_.registerCallback (func2);


        boost::unique_lock<boost::mutex> lock(data_ready_mutex_);

        if (!triggered_capture)
            capture_.start (); // Start stream

        bool scene_view_not_stopped= viz_ ? !scene_cloud_view_.cloud_viewer_->wasStopped () : true;
        bool image_view_not_stopped= viz_ ? !image_view_.viewerScene_->wasStopped () : true;

        while (!exit_ && scene_view_not_stopped && image_view_not_stopped)
        {
            if (triggered_capture)
                capture_.start(); // Triggers new frame
            bool has_data = data_ready_cond_.timed_wait (lock, boost::posix_time::millisec(100)) ;

            try { this->execute (depth_, rgb24_, has_data && integrate_); }
            catch (const std::bad_alloc& /*e*/) { cout << "Bad alloc" << endl; break; }
            catch (const std::exception& /*e*/) { cout << "Exception" << endl; break; }

            if (viz_)
                scene_cloud_view_.cloud_viewer_->spinOnce (3);
        }

        if (!triggered_capture)
            capture_.stop (); // Stop stream

        c.disconnect();
    }



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
    writeCloud (int format) const
    {
        const SceneCloudView& view = scene_cloud_view_;

        // Points to export are either in cloud_ptr_ or combined_ptr_.
        // If none have points, we have nothing to export.
        if (view.cloud_ptr_->points.empty () && view.combined_ptr_->points.empty ())
        {
            cout << "Not writing cloud: Cloud is empty" << endl;
        }
        else
        {
            if(view.point_colors_ptr_->points.empty()) // no colors
            {
                if (view.valid_combined_)
                    writeCloudFile (format, view.combined_ptr_);
                else
                    writeCloudFile (format, view.cloud_ptr_);
            }
            else
            {
                if (view.valid_combined_)
                    writeCloudFile (format, merge<PointXYZRGBNormal>(*view.combined_ptr_, *view.point_colors_ptr_));
                else
                    writeCloudFile (format, merge<PointXYZRGB>(*view.cloud_ptr_, *view.point_colors_ptr_));
            }
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
    writeMesh(int format) const
    {
        if (scene_cloud_view_.mesh_ptr_)
            writePolygonMeshFile(format, *scene_cloud_view_.mesh_ptr_);
        //   if (scene_cloud_viewDEV_.mesh_ptr_)
        //     writePolygonMeshFile(format, *scene_cloud_viewDEV_.mesh_ptr_);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
    printHelp ()
    {
        cout << endl;
        cout << "KinFu app hotkeys" << endl;
        cout << "=================" << endl;
        cout << "    H    : print this help" << endl;
        cout << "   Esc   : exit" << endl;
        cout << "    T    : take cloud" << endl;
        cout << "    A    : take mesh" << endl;
        cout << "    M    : toggle cloud exctraction mode" << endl;
        cout << "    N    : toggle normals exctraction" << endl;
        cout << "    I    : toggle independent camera mode" << endl;
        cout << "    B    : toggle volume bounds" << endl;
        cout << "    *    : toggle scene view painting ( requires registration mode )" << endl;
        cout << "    C    : clear clouds" << endl;
        cout << "   1,2,3 : save cloud to PCD(binary), PCD(ASCII), PLY(ASCII)" << endl;
        cout << "    7,8  : save mesh to PLY, VTK" << endl;
        cout << "   X, V  : TSDF volume utility" << endl;
        cout << "    D    : use AdaFruit" << endl;
        cout << endl;
    }

    bool exit_;
    bool scan_;
    bool scan_mesh_;
    bool scan_volume_;
    bool useAdafruit_ = false;
    bool independent_camera_;
    bool integrate_;

    float volume_size_;
    int resolution_;
    int icp1_, icp2_, icp3_;

    bool registration_;
    bool integrate_colors_;
    bool pcd_source_;
    float focal_length_;

    pcl::Grabber& capture_;
    ImageView image_view_;

    KinfuTracker kinfu_;
    SceneCloudView scene_cloud_view_;
    boost::shared_ptr<CurrentFrameCloudView> current_frame_cloud_view_;


    KinfuTracker::DepthMap depth_device_;

    pcl::TSDFVolume<float, short> tsdf_volume_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_ptr_;


    boost::mutex data_ready_mutex_;
    boost::condition_variable data_ready_cond_;

    std::vector<KinfuTracker::PixelRGB> source_image_data_;
    std::vector<unsigned short> source_depth_data_;
    PtrStepSz<const unsigned short> depth_;
    PtrStepSz<const KinfuTracker::PixelRGB> rgb24_;

    Adafruit_UART myAda;

    int time_ms_;
    int viz_;

    boost::shared_ptr<CameraPoseProcessor> pose_processor_;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    static void
    keyboard_callback (const visualization::KeyboardEvent &e, void *cookie)
    {
        KinFuApp* app = reinterpret_cast<KinFuApp*> (cookie);

        int key = e.getKeyCode ();

        if (e.keyUp ())
            switch (key)
            {
            case 27: app->exit_ = true; break;
            case (int)'t': case (int)'T': app->scan_ = true; break;
            case (int)'a': case (int)'A': app->scan_mesh_ = true; break;
            case (int)'h': case (int)'H': app->printHelp (); break;
            case (int)'m': case (int)'M': app->scene_cloud_view_.toggleExtractionMode (); break;
            case (int)'n': case (int)'N': app->scene_cloud_view_.toggleNormals ();                break;
            case (int)'c': case (int)'C': app->kinfu_.printposes();                break;
            case (int)'i': case (int)'I': app->toggleIndependentCamera (); break;
            case (int)'p': case (int)'P': app->toggleIntegrate (); break;
            case (int)'b': case (int)'B': app->scene_cloud_view_.toggleCube(app->kinfu_.volume().getSize());                break;
            case (int)'7': case (int)'8': app->writeMesh (key - (int)'0'); break;
            case (int)'1': case (int)'2': case (int)'3': app->writeCloud (key - (int)'0'); break;
            case (int)'d': case (int)'D': app->toggleAdafruit(!app->useAdafruit_); break;
            case (int)'r': case (int)'R': app->toggleRestart(); break;
            case (int)'s': case (int)'S': app->kinfu_.getMetricsOnNextFrame(); break;
            case '*': app->image_view_.toggleImagePaint (); break;

            case (int)'x': case (int)'X':
                app->scan_volume_ = !app->scan_volume_;
                cout << endl << "Volume scan: " << (app->scan_volume_ ? "enabled" : "disabled") << endl << endl;
                break;
            case (int)'v': case (int)'V':
                cout << "Saving TSDF volume to tsdf_volume.dat ... " << flush;
                app->tsdf_volume_.save ("tsdf_volume.dat", true);
                cout << "done [" << app->tsdf_volume_.size () << " voxels]" << endl;
                cout << "Saving TSDF volume cloud to tsdf_cloud.pcd ... " << flush;
                pcl::io::savePCDFile<pcl::PointXYZI> ("tsdf_cloud.pcd", *app->tsdf_cloud_ptr_, true);
                cout << "done [" << app->tsdf_cloud_ptr_->size () << " points]" << endl;
                break;

            default:
                break;
            }
    }
};

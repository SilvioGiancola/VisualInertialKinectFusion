#include "KinFuWindowLight.h"
#include "ui_KinFuWindowLight.h"


KinFuWindowLight::KinFuWindowLight (QWidget *parent) : QMainWindow (parent), ui (new Ui::KinFuWindowLight)
{
    ui->setupUi (this);
    this->setWindowTitle ("Kinect viewer");

    KinectCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

    // Timer for 3D/UI update
    tmrTimer = new QTimer(this);
    connect(tmrTimer,SIGNAL(timeout()),this,SLOT(processFrameAndUpdateGUI()));
    tmrTimer->start(10); // msec

    scnTimer = new QTimer(this);
    connect(scnTimer,SIGNAL(timeout()),this,SLOT(on_btnUpdateScene_clicked()));




    kinfu_.setDepthIntrinsics(525, 525);

    kinfu_.setICPiteration(10,5,4);

    int resolution = 512;

    kinfu_.tsdf_volume_ =  pcl::gpu::TsdfVolume::Ptr( new pcl::gpu::TsdfVolume(Eigen::Vector3i(resolution, resolution, resolution)) );

    int size_cube = 3.0f;


    kinfu_.volume().setSize (Eigen::Vector3f::Constant (size_cube));


    kinfu_.setInitalCameraPose (Eigen::Translation3f (size_cube/2, size_cube/2, size_cube/2) * Eigen::AngleAxisf (Eigen::Matrix3f::Identity ()));
    //  kinfuDEV_.setInitalCameraPose (Eigen::Translation3f (1.5f, 1.5f, 1.5f) * Eigen::AngleAxisf (myAda.returnPose().matrix()));

    //  toggleRestart();
    kinfu_.volume().setTsdfTruncDist (0.030f/*meters*/);
    //  kinfu_.setIcpCorespFilteringParams (0.1f/*meters*/, sin ( pcl::deg2rad(5.f) ));
    kinfu_.setDepthTruncationForICP(5.f/*meters*/);
    kinfu_.setCameraMovementThreshold(0.001f);



    //Init KinfuApp
    //  tsdf_cloud_ptr_ = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
    //   tsdf_cloud_ptrDEV_ = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
    //image_view_.raycaster_ptr_ = pcl::gpu::RayCaster::Ptr( new pcl::gpu::RayCaster(kinfu_.rows (), kinfu_.cols ()) );
    //   image_viewDEV_.raycaster_ptr_ = RayCaster::Ptr( new RayCaster(kinfuDEV_.rows (), kinfuDEV_.cols ()) );


    // Set up the QVTK window
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer3D", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    viewer->addCoordinateSystem(1.0, "Main Corrodinate System");
    viewer->setBackgroundColor(0.5,0.5,0.5);
    ui->qvtkWidget->update ();




    //    // Set up the QVTK Depth window
    //    viewerDepth_.reset (new pcl::visualization::PCLVisualizer ("viewerDepth", false));
    //    ui->QVTK_Depth->SetRenderWindow (viewerDepth_->getRenderWindow ());
    //    viewerDepth_->setupInteractor (ui->QVTK_Depth->GetInteractor (), ui->QVTK_Depth->GetRenderWindow ());
    //    ui->QVTK_Depth->update ();


    // Set up the QVTK Depth window
    //   viewerScene_.reset (new pcl::visualization::PCLVisualizer ("viewerScene", false));
    //    ui->QVTK_Depth->SetRenderWindow (viewerScene_->getRenderWindow ());
    //   viewerScene_->setupInteractor (ui->QVTK_Depth->GetInteractor (), ui->QVTK_Depth->GetRenderWindow ());
    //    ui->QVTK_Depth->update ();





    // Setup Kinect
    interface = NULL;
    // Show stream
    StopStream = false;

    // Run Kinect grabber
    interface = new pcl::OpenNIGrabber("",pcl::OpenNIGrabber::Mode::OpenNI_VGA_30Hz, pcl::OpenNIGrabber::Mode::OpenNI_VGA_30Hz);

    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&KinFuWindowLight::cloud_cb_, this, _1);
    //boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> f = boost::bind (&KinFuWindowLight::cloud_cb_, this, _1);

    interface->registerCallback(f);

    // Start interface
    interface->start();



       scnTimer->start(1000); // msec
}
/*
KinFuWindowLight::KinFuWindowLight (KinFuApp myKinFuApp, QWidget *parent = 0) : QMainWindow (parent), ui (new Ui::KinFuWindowLight);
{

}*/
KinFuWindowLight::~KinFuWindowLight ()
{
    if(interface->isRunning())
        interface->stop();

    delete ui;
}

void KinFuWindowLight::on_btnStopStream_toggled(bool checked)
{
    if(checked == true)
    {
        StopStream = true;
        ui->btnStopStream->setText("Stream stopped");
    }
    else
    {
        StopStream = false;
        ui->btnStopStream->setText("Stream running");
    }
}

void KinFuWindowLight::on_btnResetCamera_clicked()
{
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    Eigen::Vector3f t = Eigen::Vector3f (3.0/2, 3.0/2, 3.0/2);



    kinfu_.setInitalCameraPose (Eigen::Translation3f(t) * Eigen::AngleAxisf (q.matrix()));

    viewer->resetCamera();
    ui->qvtkWidget->update();
}


void KinFuWindowLight::processFrameAndUpdateGUI()
{

    QMutexLocker locker(&mex);

    if(StopStream == false)
    {
        // Add point cloud on first call
        if(!viewer->contains("cloud"))
        {
            viewer->addPointCloud(KinectCloud,"cloud");
            viewer->resetCamera();
        }
        // Update point cloud
        else
        {
            viewer->updatePointCloud (KinectCloud,"cloud");
            viewer->updatePointCloudPose("cloud", kinfu_.getCameraPose());
        }
        ui->qvtkWidget->update ();
    }




    int width  = KinectCloud->width;
    int height = KinectCloud->height;
    depth_.cols = width;
    depth_.rows = height;
    depth_.step = depth_.cols * depth_.elemSize ();
    source_depth_data_.resize (depth_.cols * depth_.rows);

    rgb24_.cols = width;
    rgb24_.rows = height;
    rgb24_.step = rgb24_.cols * rgb24_.elemSize ();
    source_image_data_.resize (rgb24_.cols * rgb24_.rows);

    unsigned char  *rgb   = (unsigned char *)  &source_image_data_[0];
    unsigned short *depth = (unsigned short *) &source_depth_data_[0];

    for (int i=0; i < width*height; i++)
    {
        pcl::PointXYZRGBA pt = KinectCloud->at (i);
        rgb[3*i +0] = pt.r;
        rgb[3*i +1] = pt.g;
        rgb[3*i +2] = pt.b;
        depth[i]    = pt.z/0.001;
    }
    rgb24_.data = &source_image_data_[0];
    depth_.data = &source_depth_data_[0];


    depth_device_.upload (depth_.data, depth_.step, depth_.rows, depth_.cols);

    bool  has_image = kinfu_ (depth_device_, depth_);



}


void KinFuWindowLight::on_btnUpdateScene_clicked()
{
    // QMutexLocker locker(&mex);
    // ScopeTimeT time ("Mesh Extraction");
    //  cout << "\nGetting mesh... " << flush;

    if (!marching_cubes_)
        marching_cubes_ = pcl::gpu::MarchingCubes::Ptr( new pcl::gpu::MarchingCubes() );

    pcl::gpu::DeviceArray<pcl::PointXYZ> triangles_device = marching_cubes_->run(kinfu_.volume(), triangles_buffer_device_);
    //  mesh_ptr_ = convertToMesh(triangles_device);

    if (triangles_device.empty())
        mesh_ptr_ = boost::shared_ptr<pcl::PolygonMesh>();
    else
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.width  = (int)triangles_device.size();
        cloud.height = 1;
        triangles_device.download(cloud.points);

        mesh_ptr_ = pcl::PolygonMesh::Ptr( new pcl::PolygonMesh() );
        pcl::toPCLPointCloud2(cloud, mesh_ptr_->cloud);

        mesh_ptr_->polygons.resize (triangles_device.size() / 3);
        for (size_t i = 0; i < mesh_ptr_->polygons.size (); ++i)
        {
            pcl::Vertices v;
            v.vertices.push_back(i*3+0);
            v.vertices.push_back(i*3+2);
            v.vertices.push_back(i*3+1);
            mesh_ptr_->polygons[i] = v;
        }
    }



    viewer->removePolygonMesh("Model");
    if (mesh_ptr_)
        viewer->addPolygonMesh(*mesh_ptr_,"Model");
}



// Point cloud callback
void KinFuWindowLight::cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & DC3)
{
    QMutexLocker locker(&mex);
    KinectCloud = DC3;
    return;
}






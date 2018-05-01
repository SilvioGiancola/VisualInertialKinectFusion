#include "KinFuWindow.h"
#include "ui_KinFuWindow.h"


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    KinFuWindow* viewer = static_cast<KinFuWindow*> (viewer_void);

    if (event.keyDown())
    {
        cout << "KeyboardEvent: " << event.getKeySym ()  << endl;

        if (event.getKeySym () == "space" )
        {

            if (viewer->is_KinectV1_active())
                viewer->on_KinectV1_toggle_clicked();

            else if (viewer->is_Files_active())
                viewer->on_Files_toggle_clicked();

            /*else if (viewer->is_RealSense_active())
                viewer->on_RealSense_toggle_clicked();*/

        }
    }
    return;
}

void KinFuWindow::keyPressEvent(QKeyEvent *e)
{
    std::cout << "keyPressEvent" << e->text().toStdString() << std::endl;

    if (e->key() == Qt::Key_Space)
    {

        if (is_KinectV1_active())
            on_KinectV1_toggle_clicked();

        else if (is_Files_active())
            on_Files_toggle_clicked();

        /*else if (is_RealSense_active())
            on_RealSense_toggle_clicked();*/

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

KinFuWindow::KinFuWindow (QWidget *parent) : QMainWindow (parent), ui (new Ui::KinFuWindow)
{
    ui->setupUi (this);
    this->setWindowTitle ("Kinect viewer");

    this->setFocusPolicy(Qt::StrongFocus);
    // ui->centralWidget->hide();

    // KinectCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    KinectCloudAll.clear();

    // Timer for 3D/UI update
    tmrTimer = new QTimer(this);
    connect(tmrTimer,SIGNAL(timeout()),this,SLOT(UpdatePointCloud()));
    //

    scnTimer = new QTimer(this);
    connect(scnTimer,SIGNAL(timeout()),this,SLOT(UpdateModel()));


    kinFuTimer = new QTimer(this);
    connect(kinFuTimer,SIGNAL(timeout()),this,SLOT(CallBackFiles()));



    KinectCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    /*
    kinfu_.setDepthIntrinsics(525, 525);

    kinfu_.setICPiteration(10,5,4);



    int resolution = 512;
    kinfu_.tsdf_volume_ =  pcl::gpu::TsdfVolume::Ptr( new pcl::gpu::TsdfVolume(Eigen::Vector3i(resolution, resolution, resolution)) );

    int size_cube = 3.0f;
    kinfu_.volume().setSize (Eigen::Vector3f::Constant (size_cube));
    kinfu_.setInitalCameraPose (Eigen::Translation3f (size_cube/2, size_cube/2, size_cube/2) * Eigen::AngleAxisf (Eigen::Matrix3f::Identity ()));
    //  kinfuDEV_.setInitalCameraPose (Eigen::Translation3f (1.5f, 1.5f, 1.5f) * Eigen::AngleAxisf (myAda.returnPose().matrix()));
*/
    //  toggleRestart();
    // kinfu_.volume().setTsdfTruncDist (0.030f);/*meters*/
    //  kinfu_.setIcpCorespFilteringParams (0.1f/*meters*/, sin ( pcl::deg2rad(5.f) ));
    //  kinfu_.setDepthTruncationForICP(5.f/*meters*/);
    //  kinfu_.setCameraMovementThreshold(0.001f);



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
    viewer->setBackgroundColor(0.7,0.7,0.7);
    ui->qvtkWidget->update ();


    viewer->setCameraPosition(ui->doubleSpinBox_Size_X->value()*1.5,-15,ui->doubleSpinBox_Size_Z->value()*2, // mi posiziono dietro ad un Kinect
                              ui->doubleSpinBox_Init_Tx->value(),ui->doubleSpinBox_Init_Ty->value(),ui->doubleSpinBox_Init_Tz->value(), // guardo un punto centrale
                              0,0,1);   // orientato con la z verso l'alto
    viewer->setCameraClipDistances(-30,30);


    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)this);

    /*

    viewerScene_ = pcl::visualization::ImageViewer::Ptr(new pcl::visualization::ImageViewer);
    viewerDepth_ = pcl::visualization::ImageViewer::Ptr(new pcl::visualization::ImageViewer);*/

    //   ui->qvtkWidget_Scene->SetRenderWindow (viewerScene_->->getRenderWindow ());

    /*viewerScene_->setWindowTitle ("View3D from ray tracing");
    viewerScene_->setPosition (0, 0);
    viewerDepth_->setWindowTitle ("Kinect Depth stream");
    viewerDepth_->setPosition (width, 0);*/


    // Files_init();
    KinectV1_init();
    //RealSense_init();
    Adafruit_init();

    on_btnResetKinFu_clicked();



    showWorkingCube(ui->checkBox_ShowWorkingCube->isChecked());
    showTrajectory(ui->checkBox_ShowTrajectory->isChecked());
    showInitialPose(ui->checkBox_ShowInitialPose->isChecked());

    myListofPointCloudFiles.clear();


    //   QString RootFolder = QString("/media/giancos/NewVolume/TUMdatasets/");
    RefreshTUMdatasets(ui->lineEdit_DatasetsRootPath->text());


    ParamsKinFuFile = QDir::homePath() + "/PointClouds/ParamKinFu.ini";
    if (!QFile(ParamsKinFuFile).exists())
        saveKinFuParameters(ParamsKinFuFile);
    else
        openKinFuParameters(ParamsKinFuFile);

/*

    for (float lambda = 0; lambda<=4; lambda = lambda+0.5)
    {
        ui->tableWidget_testList->insertRow(ui->tableWidget_testList->rowCount());
        ui->tableWidget_testList->item(0,ui->tableWidget_testList->rowCount()-1)->text() = 1;
        ui->tableWidget_testList->item(1,ui->tableWidget_testList->rowCount()-1)->text() = lambda;
        ui->tableWidget_testList->item(2,ui->tableWidget_testList->rowCount()-1)->text() = "0";

        ui->tableWidget_testList->insertRow(ui->tableWidget_testList->rowCount());
        ui->tableWidget_testList->item(0,ui->tableWidget_testList->rowCount()-1)->text() = 1;
        ui->tableWidget_testList->item(1,ui->tableWidget_testList->rowCount()-1)->text() = lambda;
        ui->tableWidget_testList->item(2,ui->tableWidget_testList->rowCount()-1)->text() = 1;
    }
*/


}

KinFuWindow::~KinFuWindow ()
{
    if(interface_kinect)
        if(interface_kinect->isRunning())
            interface_kinect->stop();

    on_Files_stop_clicked();

    /*if(interface_files)
        if(interface_files->isRunning())
            interface_files->stop();
*/

    /*if(RealSenseDevice)
        if(RealSenseDevice->is_streaming())
            RealSenseDevice->stop();*/

    if (myAda.isOpen())
        myAda.close();

    delete ui;
}




// Visual purposes
void KinFuWindow::UpdatePointCloud()
{

    QMutexLocker locker(&mex);
    //  if (!KinectCloudAll.isEmpty())
    //  {
    // Add point cloud on first call
    if(!viewer->contains("cloud"))
    {
        viewer->addPointCloud(KinectCloud,"cloud");
        viewer->updatePointCloudPose("cloud", kinfu_.getCameraPose());
    }
    // Update point cloud
    else
    {
        viewer->updatePointCloud (KinectCloud,"cloud");
        viewer->updatePointCloudPose("cloud", kinfu_.getCameraPose());
    }
    viewer->removeCoordinateSystem("currentKinectPose");
    viewer->addCoordinateSystem(0.2, kinfu_.getCameraPose(), "currentKinectPose");
    // }

  /*  if (ui->checkBox_FollowKinectPOV->isChecked())
    {
        pcl::visualization::Camera thisCam;
        viewer->getCameraParameters(thisCam);

        Eigen::Affine3f aff = kinfu_.getCameraPose();

      //  settings.beginGroup("Viewer Point of View");
        thisCam.focal[0] = aff.matrix().
        thisCam.focal[1] = settings.value("focal_Y").toDouble();
        thisCam.focal[2] = settings.value("focal_Z").toDouble();
        thisCam.pos[0] = settings.value("pos_X").toDouble();
        thisCam.pos[1] = settings.value("pos_Y").toDouble();
        thisCam.pos[2] = settings.value("pos_Z").toDouble();
        thisCam.view[0] = settings.value("view_X").toDouble();
        thisCam.view[1] = settings.value("view_Y").toDouble();
        thisCam.view[2] = settings.value("view_Z").toDouble();
        thisCam.clip[0] = settings.value("clip_front").toDouble();
        thisCam.clip[1] = settings.value("clip_back").toDouble();
      //  settings.endGroup();

        viewer->setCameraPosition(ui->doubleSpinBox_Init_Tx->value(),0,-12, // mi posiziono dietro ad un Kinect
                                  ui->doubleSpinBox_Init_Tx->value(),ui->doubleSpinBox_Init_Ty->value(),ui->doubleSpinBox_Init_Tz->value(), // guardo un punto centrale
                                  0,-1,0);   // orientato con la y verso l'alto


        viewer->setCameraParameters(thisCam);
    }*/
    ui->qvtkWidget->update ();


}

void KinFuWindow::UpdateModel()
{
    QMutexLocker locker(&mex);
    pcl::ScopeTime time ("Mesh Extraction");
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


    // Add point cloud on first call
    if (mesh_ptr_)
    {
        if(!viewer->contains("Model"))
            viewer->addPolygonMesh(*mesh_ptr_,"Model");

        // Update point cloud
        else
            viewer->updatePolygonMesh (*mesh_ptr_,"Model");

        ui->qvtkWidget->update ();
    }

}

void KinFuWindow::on_btnStopStream_clicked(bool checked)
{
    if(checked == true)
        tmrTimer->start(1000.0f/ui->doubleSpinBox_FreqStream->value()); // msec

    else
        tmrTimer->stop();

    return;
}

void KinFuWindow::on_btnUpdateScene_clicked(bool checked)
{
    if(checked == true)
        scnTimer->start(1000.0f/ui->doubleSpinBox_FreqModel->value()); // msec

    else
        scnTimer->stop();

    return;
}




// Reset
void KinFuWindow::on_btnResetKinFu_clicked()
{
    kinfu_.tsdf_volume_ =  pcl::gpu::TsdfVolume::Ptr( new pcl::gpu::TsdfVolume(Eigen::Vector3i(512,512,512)) );
    int resolution = ui->spinBox_Resolution->value();

    kinfu_.tsdf_volume_ =  pcl::gpu::TsdfVolume::Ptr( new pcl::gpu::TsdfVolume(Eigen::Vector3i(resolution, resolution, resolution)) );

    /*
    rs::intrinsics depth_intrin     = RealSenseDevice->get_stream_intrinsics( rs::stream::depth );
    qDebug () << "RS depth_intrin.fx" << depth_intrin.fx;
    qDebug () << "RS depth_intrin.fy" << depth_intrin.fy;
     kinfu_.setDepthIntrinsics(depth_intrin.fx, depth_intrin.fy);*/

    //Depth Intrinsic parameters (525 for KinV1)
    kinfu_.setDepthIntrinsics(525, 525);
    // ICP parameters
    kinfu_.setICPiteration(ui->spinBox_ICPfine->value(),ui->spinBox_ICPmedium->value(),ui->spinBox_ICPcoarse->value());

    //lambda regularization
    kinfu_.setRegularization(ui->doubleSpinBox_lambda->value());






    setWorkingCube();
    setInitialPose();
    // setIcpCorespFilteringParams();





    // TDSF cube parameters
    kinfu_.volume().setTsdfTruncDist (0.030f/*meters*/);
    //  kinfu_.setIcpCorespFilteringParams (0.1f/*meters*/, sin ( pcl::deg2rad(5.f) ));
    kinfu_.setDepthTruncationForICP(5.f/*meters*/);
    kinfu_.setCameraMovementThreshold(0.001f);



    viewer->removeAllPointClouds();
    viewer->removePolygonMesh("Model");



    //showInitialPose(ui->checkBox_ShowInitialPose->value());


    /*  QString CurrentTab = ui->tabWidget->tabText(ui->tabWidget->currentIndex());
    if (CurrentTab.compare("Kinect V1") == 0)
    {
        qDebug () << "KinectV1 mode on";*/
    KinectV1_init();
    /*  }
    else if (CurrentTab.compare("Files") == 0)
    {
        qDebug () << "Files mode on";*/
    // Files_init();
    //  RealSense_init();
    // }


    KinectCloudAll.clear();
    current_indexFile = 0;

    ui->qvtkWidget->update();
}





// IMU settings
void KinFuWindow::Adafruit_init()
{
    try
    {
        if (!myAda.open() == SUCCESS)
            //myAda.init();
            //else
            throw std::exception();

        myAda.setPlayMode(false);


        // Open a Calibration for the Sensor
        Eigen::Matrix4f _PoseAdaOnKin = Eigen::Matrix4f::Identity();
        //  loadMatrix("../PCViewer/KinFu/PoseAdafruit.txt", _PoseAdaOnKin);



        std::cout << "Adafruit calib pose is : \n" << _PoseAdaOnKin.format(Eigen::IOFormat(4,0,", ", "\n","[","]","[","]")) << std::endl;
        Eigen::Quaternionf q2 = Eigen::Quaternionf(_PoseAdaOnKin.block<3,3>(0,0));
        //   std::cout << q2.matrix() << std::endl;

        myAda.setCalibPose(q2);


        qDebug() << "Adafruit is Opened";

    }
    catch(std::exception e)
    {
        qDebug() << "exception caught in opening Adafruit";
    }

}


// File CallBack

void KinFuWindow::Files_init(QString DirPath)
{

    QDir myDir (DirPath);
    QStringList allFiles = myDir.entryList(QDir::Files);

    qDebug() << "Files_init";

    if (allFiles.length() < 1)
        return;

    allFiles.sort(); // order Alphabetically

    myListofPointCloudFiles.clear();


    // std::vector<std::string> result;
    foreach (QString file, allFiles)
    {
        QString FullPath = DirPath + "/" + file;

        if (FullPath.endsWith(".pcd"))
        {
            //qDebug () << "Point Cloud opened :" << FullPath;
            myListofPointCloudFiles.push_back (FullPath.toStdString());
        }
        else if (FullPath.endsWith(".ini"))
        {
            qDebug() << "   -> Ini Paramaters files opened : " << FullPath;
            openKinFuParameters(FullPath);
        }
    }

    //  Files_init();

    on_btnResetKinFu_clicked();

    current_indexFile = 0;
    KinectCloudAll.clear();
    ui->Files_status->setText(QString(" %1 files are opened").arg(myListofPointCloudFiles.size()));
    ui->Files_start->setEnabled(true);
    ui->Files_stop->setEnabled(true);
}
void KinFuWindow::on_Files_openFiles_clicked()
{

    /*  QStringList allFiles = QFileDialog::getOpenFileNames(this, tr("Select Point Clouds"),
                                                         QDir::homePath() + "/PointClouds/",
                                                         tr("All files (*.*)"));*/

    QString Folder = QFileDialog::getExistingDirectory(this, tr("Select Folder"),
                                                       QDir::homePath() + "/PointClouds/");

    Files_init(Folder);

}

void KinFuWindow::CallBackFiles()
{
    QMutexLocker locker(&mex);

    pcl::ScopeTime CallBack("CallBack");
    std::cout << "Calling CallBack form file" <<std::endl;

    if (current_indexFile >= myListofPointCloudFiles.size())
    {
        qDebug() << "No file selected";
        kinFuTimer->stop();
        return;
    }

    KinectCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    std::cout << "Point cloud with index " << current_indexFile <<  std::endl;
    pcl::io::loadPCDFile(myListofPointCloudFiles.at(current_indexFile), *KinectCloud);

    ui->Files_status->setText(QString(" Processing PC %1 over %2").arg(current_indexFile).arg(myListofPointCloudFiles.size()));
    ElabSinglePC(KinectCloud);
    /* if (current_indexFile >= myListofPointCloudFiles.size())
        kinFuTimer->stop();*/

    return;
}


void KinFuWindow::on_Files_grabsingle_clicked()
{
    CallBackFiles();
}

void KinFuWindow::on_Files_start_clicked(bool checked)
{
    if(checked)
    {
        float period = 1000.0f/ui->Files_FPS->value();
        kinFuTimer->start(period);
    }
    else
    {
        kinFuTimer->stop();
    }
}

void KinFuWindow::on_Files_stop_clicked()
{
    std::cout << "on_Files_stop_clicked" << std::endl;
    ui->Files_start->setChecked(false);
    kinFuTimer->stop();
    current_indexFile = 0;
    ui->Files_status->setText(QString(" Processing PC %1 over %2").arg(current_indexFile).arg(myListofPointCloudFiles.size()));
}

void KinFuWindow::on_Files_toggle_clicked()
{
}

bool KinFuWindow::is_Files_active()
{
    return (ui->tabWidget->tabText( ui->tabWidget->currentIndex()).compare("Files") == 0);
}

// Kinect callback
void KinFuWindow::KinectV1_init()
{
    qDebug() << "KinectV1_init";
    try
    {
        //   if (interface_kinect == NULL)
        // {   // Run Kinect grabber
        interface_kinect = new pcl::OpenNIGrabber("",pcl::OpenNIGrabber::Mode::OpenNI_VGA_30Hz, pcl::OpenNIGrabber::Mode::OpenNI_VGA_30Hz);

        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f_device = boost::bind (&KinFuWindow::CallBackKinectV1, this, _1);

        interface_kinect->registerCallback(f_device);
        //  }

        /* if (ui->groupBox_useAdafruit->isChecked())
            on_groupBox_useAdafruit_clicked(true);
*/
        ui->tab_KinectV1->setEnabled(true);
        ui->KinectV1_status->setText("Kinect is connected");
    }
    catch (std::exception e)
    {
        //   ui->tab_KinectV1->setEnabled(false);
        QString err = QString::fromStdString(e.what());
        ui->KinectV1_status->setText(QString("No Kinect connected: \n %1").arg(err));
    }
}

void KinFuWindow::CallBackKinectV1 (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & DC3)
{
    QMutexLocker locker(&mex);
    if (interface_kinect->isRunning())
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr KinectCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::copyPointCloud(*DC3, *KinectCloud);


        if (myAda.isOpen())
            KinectCloud->sensor_orientation_ = myAda.returnPose();



        KinectCloud->header.frame_id = QString("%2.pcd")
                .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd-HH-mm-ss-zzz"))
                .toStdString();

        KinectCloudAll.append(KinectCloud);


        ui->KinectV1_status->setText(QString(" Processing PC %1").arg(KinectCloudAll.size()));

        ElabSinglePC(KinectCloud);
    }

    return;
}

void KinFuWindow::on_KinectV1_save_clicked()
{
    // foreach(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr thisPC, KinectCloudAll)

    QString dir = QFileDialog::getExistingDirectory(this, tr("Select Directory to save"),
                                                    QDir::homePath() + "/PointClouds/",
                                                    QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    for (int i = 0; i < KinectCloudAll.length(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr thisPC = KinectCloudAll.at(i);
        bool binary = true;
        std::string path = QString("%1/%2").arg(dir)
                .arg(thisPC->header.frame_id.c_str()).toStdString();

        qDebug() << i << " actually saving " << path.c_str();
        pcl::io::savePCDFile(path, *thisPC, binary);

    }
}

void KinFuWindow::on_KinectV1_start_clicked()
{
    if(interface_kinect)
    {
        ui->dockWidget_Parameters->setEnabled(false);
        interface_kinect->start();
    }
}

void KinFuWindow::on_KinectV1_stop_clicked()
{
    if(interface_kinect)
    {
        interface_kinect->stop();
        ui->dockWidget_Parameters->setEnabled(true);
    }
}

void KinFuWindow::on_KinectV1_toggle_clicked()
{
    if (interface_kinect)
    {
        if (interface_kinect->isRunning())
            on_KinectV1_stop_clicked();
        else
            on_KinectV1_start_clicked();
    }
}

bool KinFuWindow::is_KinectV1_active()
{
    return (ui->tabWidget->tabText( ui->tabWidget->currentIndex()).compare("Kinect V1") == 0);
}







bool KinFuWindow::ElabSinglePC(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & KinectCloud)
{


    int width  = KinectCloud->width;
    int height = KinectCloud->height;
    depth_.cols = width;
    depth_.rows = height;
    depth_.step = depth_.cols * depth_.elemSize ();
    source_depth_data_.resize (depth_.cols * depth_.rows);

    // rgb24_.cols = width;
    // rgb24_.rows = height;
    //rgb24_.step = rgb24_.cols * rgb24_.elemSize ();
    //  source_image_data_.resize (rgb24_.cols * rgb24_.rows);

    // unsigned char  *rgb   = (unsigned char *)  &source_image_data_[0];
    unsigned short *depth = (unsigned short *) &source_depth_data_[0];

    for (int i=0; i < width*height; i++)
    {
        pcl::PointXYZRGBA pt = KinectCloud->at (i);
        //    rgb[3*i +0] = pt.r;
        //  rgb[3*i +1] = pt.g;
        //  rgb[3*i +2] = pt.b;
        depth[i]    = pt.z/0.001;
    }
    //  rgb24_.data = &source_image_data_[0];
    depth_.data = &source_depth_data_[0];


    depth_device_.upload (depth_.data, depth_.step, depth_.rows, depth_.cols);

    current_indexFile++;
    bool has_image;
    if (ui->groupBox_useAdafruit->isChecked())
    {
        // Eigen::AngleAxisf yawAngle(ui->doubleSpinBox_mu_Z->value()*PI/180.0, Eigen::Vector3f::UnitZ());
        // Eigen::AngleAxisf pitchAngle(ui->doubleSpinBox_mu_Y->value()*PI/180.0, Eigen::Vector3f::UnitY());
        //  Eigen::AngleAxisf rollAngle(ui->doubleSpinBox_mu_X->value()*PI/180.0, Eigen::Vector3f::UnitX());

        //  Eigen::Quaternionf errorQuat = rollAngle * pitchAngle * yawAngle;

        //  KinectCloud->sensor_orientation_ = Eigen::Quaternionf(errorQuat.matrix() * KinectCloud->sensor_orientation_.matrix());
        if (ui->checkBox_useNoiseModel->isChecked())
            KinectCloud->sensor_orientation_ = applyNoiseModel(KinectCloud->sensor_orientation_);

        Eigen::Quaternionf q = KinectCloud->sensor_orientation_;

        //  transformationMatrix * currentPose

        ui->Adafruit_status->setText(QString(" Using Adafruit %1, %2, %3, %4")
                                     .arg(QString::number(q.w(), 'f', 2))
                                     .arg(QString::number(q.x(), 'f', 2))
                                     .arg(QString::number(q.y(), 'f', 2))
                                     .arg(QString::number(q.z(), 'f', 2)));
        //  qDebug() << current_indexFile << "=?=" << KinectCloudAll.length() << " - I am using the prior orientation : " << q.w() << q.x() << q.y() << q.z();
        Eigen::Affine3f IMUhint(q);
        has_image = kinfu_ (depth_device_, &IMUhint);
    }
    else
    {
        ui->Adafruit_status->setText(QString(" Adafruit not used"));
        //    qDebug() << current_indexFile << "=?=" << KinectCloudAll.length() << " - no prior";
        has_image = kinfu_ (depth_device_);
    }
    return has_image;
}



// Stream Handlers
void KinFuWindow::on_tabWidget_currentChanged(int index)
{
    // QString objectName = ui->tabWidget->tabText(index);

    if(interface_kinect)
        if(interface_kinect->isRunning())
            interface_kinect->stop();


    /*if(interface_files)
        if(interface_files->isRunning())
            interface_files->stop();*/

    current_indexFile = 0;
    KinectCloudAll.clear();


    if (is_KinectV1_active())
        KinectV1_init();

    /*  else if (is_Files_active())
        Files_init();*/

    /*  else if (is_RealSense_active())
        RealSense_init();*/

}



// ICP Params
void KinFuWindow::on_spinBox_ICPcoarse_valueChanged(int arg1) { kinfu_.setICPiteration(ui->spinBox_ICPfine->value(), ui->spinBox_ICPmedium->value(), arg1); }
void KinFuWindow::on_spinBox_ICPmedium_valueChanged(int arg1) { kinfu_.setICPiteration(ui->spinBox_ICPfine->value(), arg1, ui->spinBox_ICPcoarse->value()); }
void KinFuWindow::on_spinBox_ICPfine_valueChanged(int arg1)   { kinfu_.setICPiteration(arg1, ui->spinBox_ICPmedium->value(), ui->spinBox_ICPcoarse->value()); }

void KinFuWindow::on_doubleSpinBox_lambda_valueChanged(double arg1) { kinfu_.setRegularization(arg1); }






void KinFuWindow::setInitialPose()
{
    Eigen::AngleAxisf rollAngle(ui->doubleSpinBox_Init_Rx->value()*PI/180.0, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(ui->doubleSpinBox_Init_Ry->value()*PI/180.0, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(ui->doubleSpinBox_Init_Rz->value()*PI/180.0, Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q = rollAngle * pitchAngle * yawAngle;

    Eigen::Vector3f t = Eigen::Vector3f (ui->doubleSpinBox_Init_Tx->value(),
                                         ui->doubleSpinBox_Init_Ty->value(),
                                         ui->doubleSpinBox_Init_Tz->value());


    kinfu_.setInitalCameraPose (Eigen::Translation3f(t) * Eigen::AngleAxisf (q.matrix()));

    if (ui->checkBox_ShowInitialPose->isChecked())
        showInitialPose(true);

}



void KinFuWindow::setWorkingCube()
{
    kinfu_.volume().setSize (Eigen::Vector3f(ui->doubleSpinBox_Size_X->value(),
                                             ui->doubleSpinBox_Size_Y->value(),
                                             ui->doubleSpinBox_Size_Z->value()));

    if (ui->checkBox_ShowWorkingCube->isChecked())
        showWorkingCube(true);
}

void KinFuWindow::setTrajectory()
{

}

void KinFuWindow::setIcpCorespFilteringParams()
{

    float distThres_ = ui->doubleSpinBox_distThresh->value();
    float angleThres_ = sin ( ui->doubleSpinBox_angleThresh->value() * 3.14159254f / 180.f );
    kinfu_.setIcpCorespFilteringParams ( distThres_,angleThres_);
}


//



// Vsiualization
void KinFuWindow::showInitialPose(bool checked)
{
    if (checked)
    {
        // Show Initial Position
        Eigen::AngleAxisf rollAngle (ui->doubleSpinBox_Init_Rx->value()*PI/180.0, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(ui->doubleSpinBox_Init_Ry->value()*PI/180.0, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle  (ui->doubleSpinBox_Init_Rz->value()*PI/180.0, Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf q = rollAngle * pitchAngle * yawAngle;

        Eigen::Affine3f aff;
        aff.linear()= q.matrix();
        aff.translation () = Eigen::Vector3f(ui->doubleSpinBox_Init_Tx->value(),
                                             ui->doubleSpinBox_Init_Ty->value(),
                                             ui->doubleSpinBox_Init_Tz->value());

        if(!viewer->contains("InitialPosition"))
            viewer->addCoordinateSystem(0.3, "InitialPosition");

        viewer->updateCoordinateSystemPose("InitialPosition", aff);
    }
    else
    {
        viewer->removeCoordinateSystem("InitialPosition");
    }
    ui->qvtkWidget->update();
    return;
}


void KinFuWindow::showWorkingCube(bool checked)
{
    if (checked)
    {
        std::vector<double> size = {
            ui->doubleSpinBox_Size_X->value(),
            ui->doubleSpinBox_Size_Y->value(),
            ui->doubleSpinBox_Size_Z->value()};

        pcl::ModelCoefficients line_coeff;
        line_coeff.values.resize (6);    // We need 6 values

        // Add X Lines
        for (int dim = 0; dim < 3; dim++)
            for (int y = 0; y < 2; y++)
                for (int z = 0; z < 2; z++)
                {
                    line_coeff.values[(dim + 0)%3] = 0;
                    line_coeff.values[(dim + 1)%3] = y*size[(dim + 1)%3];
                    line_coeff.values[(dim + 2)%3] = z*size[(dim + 2)%3];

                    line_coeff.values[(dim + 3)%3 + 3] = size[(dim + 3)%3];
                    line_coeff.values[(dim + 4)%3 + 3] = 0;//y*size;
                    line_coeff.values[(dim + 5)%3 + 3] = 0;//z*size;

                    if (viewer->contains(QString("line %1%2%3").arg(dim).arg(y).arg(z).toStdString()))
                        viewer->removeShape(QString("line %1%2%3").arg(dim).arg(y).arg(z).toStdString());
                    viewer->addLine (line_coeff, QString("line %1%2%3").arg(dim).arg(y).arg(z).toStdString());

                }
    }
    else
    {
        for (int dim = 0; dim < 3; dim++)
            for (int y = 0; y < 2; y++)
                for (int z = 0; z < 2; z++)
                    if (viewer->contains(QString("line %1%2%3").arg(dim).arg(y).arg(z).toStdString()))
                        viewer->removeShape(QString("line %1%2%3").arg(dim).arg(y).arg(z).toStdString());



    }
    ui->qvtkWidget->update();
}

void KinFuWindow::showTrajectory(bool checked)
{
    std::string prefix = "Traj_";

    if (checked)
    {
        Eigen::Affine3f Aff, Aff_prev;
        for( int i = 0; i < kinfu_.getNumberOfPoses(); i++)
        {

            Aff_prev = Aff;
            Aff = kinfu_.getCameraPose(i);


            if (i % ui->spinBox_SRSfreq->value() == 0)
            {
                if(!viewer->contains(prefix + std::to_string(i)))
                    viewer->addCoordinateSystem(0.1, prefix + std::to_string(i));

                viewer->updateCoordinateSystemPose(prefix + std::to_string(i), Aff);
            }

            if (i>0)
            {

                pcl::ModelCoefficients line_coeff;
                line_coeff.values.resize (6);    // We need 6 values

                line_coeff.values[0] = Aff_prev.translation().x();
                line_coeff.values[1] = Aff_prev.translation().y();
                line_coeff.values[2] = Aff_prev.translation().z();

                line_coeff.values[3] = Aff.translation().x() - Aff_prev.translation().x();
                line_coeff.values[4] = Aff.translation().y() - Aff_prev.translation().y();
                line_coeff.values[5] = Aff.translation().z() - Aff_prev.translation().z();
                viewer->addLine (line_coeff, QString("traj_from_%1_to_%2").arg(i-1).arg(i).toStdString());
            }

        }
    }
    else
    {
        for( int i = 0; i < 99999; i++)
        {
            if (viewer->contains(prefix + std::to_string(i)))
                viewer->removeCoordinateSystem(prefix + std::to_string(i));

            if (viewer->contains(QString("traj_from_%1_to_%2").arg(i-1).arg(i).toStdString()))
                viewer->removeShape(QString("traj_from_%1_to_%2").arg(i-1).arg(i).toStdString());

        }
    }
    ui->qvtkWidget->update();
}

void KinFuWindow::on_pushButton_InitAdafruit_clicked()
{
    myAda.init();
}

void KinFuWindow::on_doubleSpinBox_Size_X_valueChanged(double arg1) { setWorkingCube(); }
void KinFuWindow::on_doubleSpinBox_Size_Y_valueChanged(double arg1) { setWorkingCube(); }
void KinFuWindow::on_doubleSpinBox_Size_Z_valueChanged(double arg1) { setWorkingCube(); }
void KinFuWindow::on_doubleSpinBox_distThresh_valueChanged(double arg1) { setIcpCorespFilteringParams(); }
void KinFuWindow::on_doubleSpinBox_angleThresh_valueChanged(double arg1) { setIcpCorespFilteringParams(); }


void KinFuWindow::on_pushButton_ResetViewer_clicked()
{

    QSettings settings("Silvio Giancola", "testKinFuGUI");

    pcl::visualization::Camera thisCam;
    viewer->getCameraParameters(thisCam);

    settings.beginGroup("Viewer Point of View");
    thisCam.focal[0] = settings.value("focal_X").toDouble();
    thisCam.focal[1] = settings.value("focal_Y").toDouble();
    thisCam.focal[2] = settings.value("focal_Z").toDouble();
    thisCam.pos[0] = settings.value("pos_X").toDouble();
    thisCam.pos[1] = settings.value("pos_Y").toDouble();
    thisCam.pos[2] = settings.value("pos_Z").toDouble();
    thisCam.view[0] = settings.value("view_X").toDouble();
    thisCam.view[1] = settings.value("view_Y").toDouble();
    thisCam.view[2] = settings.value("view_Z").toDouble();
    thisCam.clip[0] = settings.value("clip_front").toDouble();
    thisCam.clip[1] = settings.value("clip_back").toDouble();
    settings.endGroup();

    /*   viewer->setCameraPosition(ui->doubleSpinBox_Init_Tx->value(),0,-12, // mi posiziono dietro ad un Kinect
                              ui->doubleSpinBox_Init_Tx->value(),ui->doubleSpinBox_Init_Ty->value(),ui->doubleSpinBox_Init_Tz->value(), // guardo un punto centrale
                              0,-1,0);   // orientato con la y verso l'alto
    */

    viewer->setCameraParameters(thisCam);

    ui->qvtkWidget->update();
}

void KinFuWindow::on_pushButton_SavePointofView_clicked()
{
    QSettings settings("Silvio Giancola", "testKinFuGUI");

    pcl::visualization::Camera thisCam;
    viewer->getCameraParameters(thisCam);
    //  thisCam.

    settings.beginGroup("Viewer Point of View");
    settings.setValue("focal_X", thisCam.focal[0]);
    settings.setValue("focal_Y", thisCam.focal[1]);
    settings.setValue("focal_Z", thisCam.focal[2]);
    settings.setValue("pos_X", thisCam.pos[0]);
    settings.setValue("pos_Y", thisCam.pos[1]);
    settings.setValue("pos_Z", thisCam.pos[2]);
    settings.setValue("view_X", thisCam.view[0]);
    settings.setValue("view_Y", thisCam.view[1]);
    settings.setValue("view_Z", thisCam.view[2]);
    settings.setValue("clip_front", thisCam.clip[0]);
    settings.setValue("clip_back", thisCam.clip[1]);
    settings.endGroup();
}

void KinFuWindow::on_pushButton_SaveParams_clicked()
{
    saveKinFuParameters(ParamsKinFuFile);
}

void KinFuWindow::on_pushButton_SaveParamsAs_clicked()
{
    QString m_settingfile = QFileDialog::getSaveFileName(this, tr("Saving path"),
                                                         QDir::homePath() + "/PointClouds/");

    if (!m_settingfile.isEmpty())
        saveKinFuParameters(m_settingfile);
}

void KinFuWindow::on_pushButton_OpenParams_clicked()
{
    QString m_settingfile = QFileDialog::getOpenFileName(this, tr("Saving path"),
                                                         QDir::homePath() + "/PointClouds/KinFuParams.ini");

    if (!m_settingfile.isEmpty())
        openKinFuParameters(m_settingfile);
}


void KinFuWindow::saveKinFuParameters(QString path)
{
    QSettings settings(path, QSettings::IniFormat);

    settings.setValue("Resolution", ui->spinBox_Resolution->value());
    settings.setValue("Size_X", ui->doubleSpinBox_Size_X->value());
    settings.setValue("Size_Y", ui->doubleSpinBox_Size_Y->value());
    settings.setValue("Size_Z", ui->doubleSpinBox_Size_Z->value());

    settings.setValue("ICPcoarse", ui->spinBox_ICPcoarse->value());
    settings.setValue("ICPmedium", ui->spinBox_ICPmedium->value());
    settings.setValue("ICPfine", ui->spinBox_ICPfine->value());

    settings.setValue("R_X", ui->doubleSpinBox_Init_Rx->value());
    settings.setValue("R_Y", ui->doubleSpinBox_Init_Ry->value());
    settings.setValue("R_Z", ui->doubleSpinBox_Init_Rz->value());

    settings.setValue("T_X", ui->doubleSpinBox_Init_Tx->value());
    settings.setValue("T_Y", ui->doubleSpinBox_Init_Ty->value());
    settings.setValue("T_Z", ui->doubleSpinBox_Init_Tz->value());
}

void KinFuWindow::openKinFuParameters(QString path)
{
    ParamsKinFuFile = path;
    QSettings settings(path, QSettings::IniFormat);


    int resolution = settings.value("Resolution", "").toInt();
    qDebug() << "ini: set resolution to " << resolution;
    ui->spinBox_Resolution->setValue(resolution);



    double sizeX = settings.value("Size_X", "").toDouble();
    qDebug() << "ini: set Size_X to " << sizeX;
    ui->doubleSpinBox_Size_X->setValue(sizeX);

    double sizeY = settings.value("Size_Y", "").toDouble();
    qDebug() << "ini: set Size_Y to " << sizeY;
    ui->doubleSpinBox_Size_Y->setValue(sizeY);

    double sizeZ = settings.value("Size_Z", "").toDouble();
    qDebug() << "ini: set Size_Z to " << sizeZ;
    ui->doubleSpinBox_Size_Z->setValue(sizeZ);


    int ICPcoarse = settings.value("ICPcoarse", "").toInt();
    qDebug() << "ini: set ICPcoarse to " << ICPcoarse;
    ui->spinBox_ICPcoarse->setValue(ICPcoarse);

    int ICPmedium = settings.value("ICPmedium", "").toInt();
    qDebug() << "ini: set ICPmedium to " << ICPmedium;
    ui->spinBox_ICPmedium->setValue(ICPmedium);

    int ICPfine = settings.value("ICPfine", "").toInt();
    qDebug() << "ini: set ICPfine to " << ICPfine;
    ui->spinBox_ICPfine->setValue(ICPfine);


    double R_X = settings.value("R_X", "").toDouble();
    qDebug() << "ini: set R_X to " << R_X;
    ui->doubleSpinBox_Init_Rx->setValue(R_X);

    double R_Y = settings.value("R_Y", "").toDouble();
    qDebug() << "ini: set R_Y to " << R_Y;
    ui->doubleSpinBox_Init_Ry->setValue(R_Y);

    double R_Z = settings.value("R_Z", "").toDouble();
    qDebug() << "ini: set R_Z to " << R_Z;
    ui->doubleSpinBox_Init_Rz->setValue(R_Z);

    double T_X = settings.value("T_X", "").toDouble();
    qDebug() << "ini: set T_X to " << T_X;
    ui->doubleSpinBox_Init_Tx->setValue(T_X);

    double T_Y = settings.value("T_Y", "").toDouble();
    qDebug() << "ini: set T_Y to " << T_Y;
    ui->doubleSpinBox_Init_Ty->setValue(T_Y);

    double T_Z = settings.value("T_Z", "").toDouble();
    qDebug() << "ini: set T_Z to " << T_Z;
    ui->doubleSpinBox_Init_Tz->setValue(T_Z);


}



void KinFuWindow::on_btnDownloadModel_clicked()
{
    UpdateModel();
}

void KinFuWindow::on_btnSaveModel_clicked()
{
    if (mesh_ptr_)
    {
        QString path = QFileDialog::getSaveFileName(this, tr("Select a File to Save"),
                                                    QDir::homePath() + "/PointClouds/mesh.ply");
        if (!path.isEmpty())        SaveModel(path);
    }
}

void KinFuWindow::SaveModel(QString path)
{
    if (mesh_ptr_)
    {
        cout << "Saving mesh to " << path.toStdString() << endl;
        pcl::io::savePLYFile(path.toStdString(), *mesh_ptr_);
    }
    else
        cout << "Cannot save mesh" << endl;
}



void KinFuWindow::on_btnDownloadTrajectory_clicked()
{
    std::string prefix = "Traj_";

    for( int i = 0; i < 99999; i++)
    {
        if (viewer->contains(prefix + std::to_string(i)))
            viewer->removeCoordinateSystem(prefix + std::to_string(i));

        if (viewer->contains(QString("traj_from_%1_to_%2").arg(i-1).arg(i).toStdString()))
            viewer->removeShape(QString("traj_from_%1_to_%2").arg(i-1).arg(i).toStdString());

    }


    Eigen::Affine3f Aff, Aff_prev;
    for( int i = 0; i < kinfu_.getNumberOfPoses(); i++)
    {

        Aff_prev = Aff;
        Aff = kinfu_.getCameraPose(i);


        if (i % ui->spinBox_SRSfreq->value() == 0)
        {
            if(!viewer->contains(prefix + std::to_string(i)))
                viewer->addCoordinateSystem(0.1, prefix + std::to_string(i));

            viewer->updateCoordinateSystemPose(prefix + std::to_string(i), Aff);
        }

        if (i>0)
        {

            pcl::ModelCoefficients line_coeff;
            line_coeff.values.resize (6);    // We need 6 values

            line_coeff.values[0] = Aff_prev.translation().x();
            line_coeff.values[1] = Aff_prev.translation().y();
            line_coeff.values[2] = Aff_prev.translation().z();

            line_coeff.values[3] = Aff.translation().x() - Aff_prev.translation().x();
            line_coeff.values[4] = Aff.translation().y() - Aff_prev.translation().y();
            line_coeff.values[5] = Aff.translation().z() - Aff_prev.translation().z();
            viewer->addLine (line_coeff, QString("traj_from_%1_to_%2").arg(i-1).arg(i).toStdString());
        }
    }
}

void KinFuWindow::on_btnSavePoses_clicked()
{
    QString path = QFileDialog::getSaveFileName(this, tr("Select a File to Save"),
                                                QDir::homePath() + "/PointClouds/traj.txt");
    if (!path.isEmpty()) SavePoses(path);



}

void KinFuWindow::SavePoses(QString Path)
{
    /*  std::ofstream out_stream_;
    out_stream_.open (Path.toStdString().c_str () );

    for( int i = 0; i < kinfu_.getNumberOfPoses(); i++)
    {

        Eigen::Affine3f pose = kinfu_.getCameraPose(i);

        Eigen::Quaternionf q (pose.rotation ());
        Eigen::Vector3f t (pose.translation ());
        // write translation , quaternion in a row
        out_stream_ << t[0] << "," << t[1] << "," << t[2]
                            << "," << q.w () << "," << q.x ()
                            << "," << q.y ()<< ","  << q.z () << std::endl;
    }
    out_stream_.close ();
    */

    std::ofstream out_stream_;
    out_stream_.open (Path.toStdString().c_str () );

    for( int i = 0; i < kinfu_.getNumberOfPoses(); i++)
    {
        QString path = QString::fromStdString(myListofPointCloudFiles.at(i));
        QString timestamp = path.split("/").last().mid(3,17);


        Eigen::Affine3f pose = kinfu_.getCameraPose(i);

        Eigen::Quaternionf q (pose.rotation ());
        Eigen::Vector3f t (pose.translation ());
        // write translation , quaternion in a row
        out_stream_ << timestamp.toStdString() << " " <<
                       t[0] << " " << t[1] << " " << t[2]
                    << " " << q.w () << " " << q.x ()
                    << " " << q.y ()<< " "  << q.z () << std::endl;
    }
    out_stream_.close ();


    return;
}




void KinFuWindow::on_pushButton_BatchElab_clicked()
{
    QList<float> LambdaList = {-1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2.0};
    //    QList<float> LambdaList = {-1.0,0.0,1.0,2.0};
    //    QList<float> LambdaList = {-1.0,0.0,0.5,1.0,1.5,2.0};
    QString RootFolder = QString("/media/giancos/NewVolume/KinFuAcq/");

    QStringList allEnvironnement = QDir(RootFolder).entryList(QDir::AllDirs | QDir::NoDotAndDotDot);

    foreach (QString Environnement, allEnvironnement)
    {
        QStringList allDataSet = QDir(RootFolder + Environnement).entryList(QDir::AllDirs | QDir::NoDotAndDotDot);

        foreach (QString DataSet, allDataSet)
        {
            QString Folder = RootFolder + Environnement + "/" +  DataSet ;
            qDebug () << Folder;

            foreach (float lambda, LambdaList)
            {
                qDebug () << "";

                QString ModelPath = QString("%1/lambda_%2_mesh.ply").arg(Folder).arg(lambda);
                QString TrajPath = QString("%1/lambda_%2_traj.csv").arg(Folder).arg(lambda);
                QString LogPath = QString("%1/lambda_%2_log.txt").arg(Folder).arg(lambda);

                /*  if (QFile(TrajPath).exists())
                {
                    qDebug() << "I have already solved for this dataset " << TrajPath;
                }
               // else if (QFile(ModelPath).exists())
              //  {
              //      qDebug() << "I have already solved for this dataset " << ModelPath;
              //  }
               else
                {*/
                try
                {


                    if (lambda < 0)
                    {
                        ui->doubleSpinBox_lambda->setValue(0);
                        ui->groupBox_useAdafruit->setChecked(false);
                    }
                    else
                    {
                        ui->groupBox_useAdafruit->setChecked(true);
                        ui->doubleSpinBox_lambda->setValue(lambda);
                    }


                    qDebug() << "Casting Out the Log";
                    // brind cout to file
                    std::ofstream  out(LogPath.toStdString());
                    std::streambuf *coutbuf = std::cout.rdbuf();
                    std::cout.rdbuf(out.rdbuf());


                    qDebug() << "Init the Point Clouds and Ini Files";
                    Files_init(Folder);



                    // build the filter
                    pcl::ConditionAnd<pcl::PointXYZRGBA>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGBA> ());
                    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGBA> ("z", pcl::ComparisonOps::GT, 0.01)));
                    pcl::ConditionalRemoval<pcl::PointXYZRGBA> condrem(true);
                    condrem.setCondition(range_cond);

                    for (int i = 0; i < myListofPointCloudFiles.size(); i++)
                        //foreach (std::string file, myListofPointCloudFiles)
                    {
                        std::string file = myListofPointCloudFiles.at(i);
                        qDebug() << "now Processing : " << QString::fromStdString(file) << " with lambda = " << lambda;
                        KinectCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
                        pcl::io::loadPCDFile(file, *KinectCloud);

                        ////////////
                        /// Remove point which Z= 0 in local reference system\
                        ///


                        condrem.setInputCloud (KinectCloud);
                        condrem.setKeepOrganized(true);


                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGBA>);
                        condrem.filter (*temp);

                        pcl::PointIndices::Ptr ind (new pcl::PointIndices());
                        condrem.getRemovedIndices(*ind);
                        //condrem.setUserFilterValue();

                        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
                        extract.setInputCloud (KinectCloud);
                        extract.setIndices (ind);
                        extract.setNegative (true);
                        extract.setKeepOrganized(KinectCloud->isOrganized());
                        extract.filter (*KinectCloud);





                        ///
                        QString log = QString("Processing PC %1 over %2").arg(current_indexFile).arg(myListofPointCloudFiles.size());
                        ui->Files_status->setText(log);
                        std::cout << std::endl << log.toStdString() << std::endl;
                        bool res = ElabSinglePC(KinectCloud);
                        if (i > 0 && !res)
                            break;
                    }

                    qDebug() << "Saving the Trajectory";
                    SavePoses(TrajPath );

                    /* qDebug() << "Updating the Model";
                        UpdateModel();
                        qDebug() << "Saving the Model";
                        SaveModel(ModelPath);*/


                    // reset cout to console
                    std::cout.rdbuf(coutbuf);

                    QThread::sleep(5);
                }
                catch( pcl::PCLException e)
                {
                    qDebug () << "Ho Ho, I catched a PCLException";
                    qDebug () << QString::fromStdString(e.detailedMessage());
                }
                catch( std::exception e)
                {
                    qDebug () << "Ho Ho, I catched an std::exception";
                    qDebug () << QString::fromStdString(e.what());
                }
                catch( std::bad_alloc e)
                {
                    qDebug () << "Ho Ho, I catched a bad alloc";
                    qDebug () << QString::fromStdString(e.what());
                }
                catch( std::invalid_argument e)
                {
                    qDebug () << "Ho Ho, I catched an std::invalid_argument";
                    qDebug () << QString::fromStdString(e.what());
                }
                catch( ... )
                {
                    qDebug () << "Ho Ho, I catched something but I don't know what!";
                }
                // }
            }
        }
    }
}

void KinFuWindow::on_pushButton_BatchElab_TUM_clicked()
{

    QList<float> LambdaList;
    foreach (QString str, ui->lineEdit_lambdaStr->text().split(','))
        LambdaList.push_back(str.toFloat());



    for (int i = 0; i < ui->listWidget_Datasets->count(); ++i)
    {

        QString Folder = ui->lineEdit_DatasetsRootPath->text() + "/"
                + ui->listWidget_Datasets->item(i)->text() ;

        qDebug () << Folder;




        if (ui->listWidget_Datasets->item(i)->checkState() != Qt::Checked)
        {
            qDebug () << "Folder not selected";
        }
        else
        {

            // NOT WORKING!
            QDir resultsDir(Folder);
            if (resultsDir.exists("resultsBatch5"))
            {
                qDebug() << "folder already exists";
            }
            else
            {
                qDebug() << "folder do not exists";
                resultsDir.mkdir("resultsBatch5");
            }


            foreach (float lambda, LambdaList)
            {

                bool useIMU = (lambda >= 0);


                QList<bool> NoiseModelList = {true,false};

             //   if (useIMU) NoiseModelList = {true,false};
             //   else NoiseModelList = {false};


                foreach (bool useNoiseModel, NoiseModelList)

                {
                    qDebug () << "";

                    QString ModelPath = QString("%1/resultsBatch5/lambda_%2_%3mesh.ply").arg(Folder).arg(lambda).arg(useNoiseModel?"noisy_":"");
                    QString TrajPath = QString("%1/resultsBatch5/lambda_%2_%3traj.txt").arg(Folder ).arg(lambda).arg(useNoiseModel?"noisy_":"");
                    QString LogPath = QString("%1/resultsBatch5/lambda_%2_%3log.txt").arg(Folder).arg(lambda).arg(useNoiseModel?"noisy_":"");
                    qDebug() << "ModelPath : " << ModelPath;
                    qDebug() << "TrajPath : " << TrajPath;
                    qDebug() << "LogPath : " << LogPath;





                    if (QFile(LogPath).exists() && !ui->checkBox_OverWrite->isChecked())
                    {
                        qDebug() << "I have already solved for this dataset " << LogPath;
                    }
                    else if (QFile(TrajPath).exists() && !ui->checkBox_OverWrite->isChecked())
                    {
                        qDebug() << "I have already solved for this dataset " << TrajPath;
                    }
                    else
                    {


                        ui->groupBox_useAdafruit->setChecked(useIMU);
                        ui->doubleSpinBox_lambda->setValue(lambda);
                        ui->checkBox_useNoiseModel->setChecked(useNoiseModel);





                        try
                        {

                            qDebug() << "Casting Out the Log";
                            // bring cout out to file
                            std::ofstream  out(LogPath.toStdString());
                            std::streambuf *coutbuf = std::cout.rdbuf();
                            std::cout.rdbuf(out.rdbuf());


                            qDebug() << "Init the Point Clouds and Ini Files";
                            Files_init(Folder +  "/pointclouds");



                            // build the filter
                            pcl::ConditionAnd<pcl::PointXYZRGBA>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGBA> ());
                            range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGBA> ("z", pcl::ComparisonOps::GT, 0.01)));
                            pcl::ConditionalRemoval<pcl::PointXYZRGBA> condrem(true);
                            condrem.setCondition(range_cond);

                            for (int i = 0; i < myListofPointCloudFiles.size(); i++)
                                //foreach (std::string file, myListofPointCloudFiles)
                            {
                                std::string file = myListofPointCloudFiles.at(i);
                                qDebug() << "now Processing : " << QString::fromStdString(file) << " with lambda = " << lambda;
                                KinectCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
                                pcl::io::loadPCDFile(file, *KinectCloud);

                                ////////////
                                /// Remove point which Z= 0 in local reference system\
                                ///


                                condrem.setInputCloud (KinectCloud);
                                condrem.setKeepOrganized(true);


                                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGBA>);
                                condrem.filter (*temp);

                                pcl::PointIndices::Ptr ind (new pcl::PointIndices());
                                condrem.getRemovedIndices(*ind);
                                //condrem.setUserFilterValue();

                                pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
                                extract.setInputCloud (KinectCloud);
                                extract.setIndices (ind);
                                extract.setNegative (true);
                                extract.setKeepOrganized(KinectCloud->isOrganized());
                                extract.filter (*KinectCloud);





                                ///
                                QString log = QString("Processing PC %1 over %2").arg(current_indexFile).arg(myListofPointCloudFiles.size());
                                ui->Files_status->setText(log);
                                std::cout << std::endl << log.toStdString() << std::endl;
                                /*bool res =*/ ElabSinglePC(KinectCloud);
                                // if (i > 0 && !res)
                                //     break;
                            }

                            qDebug() << "Saving the Trajectory";
                            SavePoses(TrajPath);


                            /* qDebug() << "Updating the Model";
                            UpdateModel();
                            qDebug() << "Saving the Model";
                            SaveModel(ModelPath);*/


                            // reset cout to console
                            std::cout.rdbuf(coutbuf);

                            QThread::sleep(5);
                        }
                        catch( pcl::PCLException e)
                        {
                            qDebug () << "Ho Ho, I catched a PCLException";
                            qDebug () << QString::fromStdString(e.detailedMessage());
                        }
                        catch( std::exception e)
                        {
                            qDebug () << "Ho Ho, I catched an std::exception";
                            qDebug () << QString::fromStdString(e.what());
                        }
                        catch( std::bad_alloc e)
                        {
                            qDebug () << "Ho Ho, I catched a bad alloc";
                            qDebug () << QString::fromStdString(e.what());
                        }
                        catch( std::invalid_argument e)
                        {
                            qDebug () << "Ho Ho, I catched an std::invalid_argument";
                            qDebug () << QString::fromStdString(e.what());
                        }
                        catch( ... )
                        {
                            qDebug () << "Ho Ho, I catched something but I don't know what!";
                        }
                    }
                }
            }
        }
    }
}


void KinFuWindow::on_pushButton_CurrentPCPose_clicked()
{
    /*
    std::string file = myListofPointCloudFiles.at(current_indexFile);
    KinectCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile(file, *KinectCloud);
    */
    auto euler = KinectCloud->sensor_orientation_.toRotationMatrix().eulerAngles(0,1,2);
    ui->doubleSpinBox_Init_Rx->setValue(((double)euler(0))*180.f/3.14159);
    ui->doubleSpinBox_Init_Ry->setValue(((double)euler(1))*180.f/3.14159);
    ui->doubleSpinBox_Init_Rz->setValue(((double)euler(2))*180.f/3.14159);
    setInitialPose();
}


void KinFuWindow::RefreshTUMdatasets(QString RootFolder)
{
    QStringList allDataSet = QDir(RootFolder).entryList(QDir::AllDirs | QDir::NoDotAndDotDot);
    ui->listWidget_Datasets->clear();

    foreach (QString DataSet, allDataSet)
    {
        QListWidgetItem* item = new QListWidgetItem(DataSet, ui->listWidget_Datasets);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable); // set checkable flag
        item->setCheckState(Qt::Unchecked); // AND initialize check state
    }
}


void KinFuWindow::on_pushButton_RefreshDatasets_clicked()
{
    RefreshTUMdatasets(ui->lineEdit_DatasetsRootPath->text());
}



Eigen::Quaternionf KinFuWindow::applyNoiseModel(Eigen::Quaternionf quat)
{
    std::vector<float> polyx = {5.0336e-19,-4.5614e-17,-4.205e-14,3.3053e-12,1.2105e-09,-7.4391e-08,-1.4335e-05,0.00045265,1.0721,0.49059};
    std::vector<float> polyy = {4.4603e-19,8.6866e-17,-3.9541e-14,-5.9939e-12,1.2096e-09,1.3383e-07,-1.4708e-05,-0.00097595,1.0586,0.37616};
    std::vector<float> polyz = {9.0759e-20,-8.6837e-17,-8.9757e-15,5.6317e-12,2.8428e-10,-1.2174e-07,-1.5651e-06,0.0010182,0.94418,0.85568};


    Eigen::Vector3f rpy = quat.matrix().eulerAngles(0,1,2)*180/PI;


  //  std::cout << "rpy before : " << rpy << std::endl;

    Eigen::Vector3f res = Eigen::Vector3f::Zero();

    for (int i = 0; i < polyx.size(); i++)
    {
        res[0] = res[0]*rpy[0] + polyx[i];
        res[1] = res[1]*rpy[1] + polyx[i];
        res[2] = res[2]*rpy[2] + polyx[i];
    }

 //   std::cout << "rpy after : " << res << std::endl;

    quat = Eigen::AngleAxisf(res[0]*PI/180.0, Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(res[1]*PI/180.0, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(res[2]*PI/180.0, Eigen::Vector3f::UnitZ());
    return quat;
}

void KinFuWindow::on_pushButton_clicked()
{
    ui->tableWidget_testList->insertRow(ui->tableWidget_testList->rowCount());
}

void KinFuWindow::on_pushButton_2_clicked()
{
    ui->tableWidget_testList->removeRow(ui->tableWidget_testList->currentRow());
}

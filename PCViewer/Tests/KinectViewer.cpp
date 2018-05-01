#include "KinectViewer.h"
#include "ui_KinectViewer.h"


KinectViewer::KinectViewer (QWidget *parent) : QMainWindow (parent), ui (new Ui::KinectViewer)
{
    ui->setupUi (this);
    this->setWindowTitle ("Kinect viewer");

 /*   QString defaultSavingPath = QString("%1/PointClouds/%2-AcqKinFu/")
            .arg(QDir::homePath())
            .arg(QDate::currentDate().toString("yyyy-MM-dd"));
*/
    //ui->lineEdit_savingPath->setText(defaultSavingPath);

    /*if (!QDir(defaultSavingPath).exists())
        QDir().mkdir(defaultSavingPath);
*/

    KinectCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

    // Timer for 3D/UI update
    tmrTimer = new QTimer(this);
    connect(tmrTimer,SIGNAL(timeout()),this,SLOT(processFrameAndUpdateGUI()));
    tmrTimer->start(10); // msec

    KinFreqTimer = new QTimer(this);
    connect(KinFreqTimer,SIGNAL(timeout()),this,SLOT(restartKinFreqCounter()));
    KinFreqTimer->start(1000); // msec
    KinFreqCounter = 0;

    WinFreqTimer = new QTimer(this);
    connect(WinFreqTimer,SIGNAL(timeout()),this,SLOT(restartWinFreqCounter()));
    WinFreqTimer->start(1000); // msec
    WinFreqCounter = 0;

    // Setup Kinect
    // bRun = false;
    interface = NULL;

    // Run Kinect grabber
    run();

    // Set up the QVTK window
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    viewer->addCoordinateSystem(1.0, "Main Corrodinate System");
    ui->qvtkWidget->update ();


    viewer->addText ("window frequency is " + std::to_string(WinFreqCounter) + "Hz", 2, 30, 15, 34, 135, 246, "WinFreqCounter");
    viewer->addText ("call back frequency is " + std::to_string(KinFreqCounter) + "Hz", 2, 15, 15, 34, 135, 246, "KinFreqCounter");


    // Show stream
    // StopStream = false;
}


void KinectViewer::processFrameAndUpdateGUI()
{
    QMutexLocker locker(&mex);

    if(/*bCopying == false &&*/ ui->checkBox_startStream->isChecked())
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
            viewer->updatePointCloudPose("cloud", Eigen::Affine3f(KinectCloud->sensor_orientation_));
        }
        ui->qvtkWidget->update ();

        WinFreqCounter++;

    }
}

KinectViewer::~KinectViewer ()
{
    stop();
    myAda.close();
    delete ui;
}


void KinectViewer::on_checkBox_startStream_clicked(bool checked)
{
    //checkBox_startStream
}



void KinectViewer::on_checkBox_savePointCloud_clicked(bool checked)
{
    if (checked)
    {
        qDebug() << "Starting to Save PC " ;
    }
    else
    {
        //  QMutexLocker locker(&mex);

        qDebug() << "saving is still checked? (should NOT be, ie [false]) : "  << ui->checkBox_savePointCloud->isChecked();
        qDebug() << "Can now save " << PCtoSave.length() << " PCs :";

      //  PCtoSave.clear();
    }
}



void KinectViewer::on_btnResetCamera_clicked()
{
    viewer->resetCamera();
    ui->qvtkWidget->update();
}


void KinectViewer::restartKinFreqCounter()
{
    viewer->updateText("call back frequency is " + std::to_string(KinFreqCounter) + "Hz", 2, 15, 15, 34, 135, 246, "KinFreqCounter");
  //  std::cout << "call back frequency is " << KinFreqCounter << "Hz"<< std::endl;
    KinFreqCounter = 0;
}

void KinectViewer::restartWinFreqCounter()
{
    viewer->updateText ("window frequency is " + std::to_string(WinFreqCounter) + "Hz", 2, 30, 15, 34, 135, 246, "WinFreqCounter");
   // std::cout << "window frequency is " << WinFreqCounter << "Hz" << std::endl;
    WinFreqCounter = 0;
}


// Point cloud callback
void KinectViewer::cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
    //  QTime t;
    //   t.start();
    QMutexLocker locker(&mex);

    //bCopying = true;
    KinFreqCounter++;
    pcl::copyPointCloud(*cloud, *KinectCloud);
    // KinectCloud = cloud;
    // bCopying = false;
    //std::cout << t.elapsed() << std::endl;



    if (ui->checkBox_includeOrientation->isChecked())
        KinectCloud->sensor_orientation_ = myAda.returnPose();

    KinectCloud->header.frame_id = QDateTime::currentDateTime().toString("yyyy-MM-dd-HH-mm-ss-zzz").toStdString();
    //   bool binary = true;

    //  qDebug() << QString::fromStdString(KinectCloud->header.frame_id);


    if (ui->checkBox_savePointCloud->isChecked())
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr newPCtoSave(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::copyPointCloud(*KinectCloud, *newPCtoSave);
        PCtoSave.append(newPCtoSave);
    }

    // pcl::io::savePCDFile(KinectCloud->header.frame_id, *KinectCloud, binary);
}

// Run Kinect
int KinectViewer::run() {
    if (interface != NULL)
        if (interface->isRunning() == true)
            return -1;

    interface = new pcl::OpenNIGrabber("",pcl::OpenNIGrabber::Mode::OpenNI_VGA_30Hz, pcl::OpenNIGrabber::Mode::OpenNI_VGA_30Hz);

    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&KinectViewer::cloud_cb_, this, _1);

    interface->registerCallback(f);

    // Start interface
    interface->start();
    // Running
    // bRun = true;
    return 0;
}

// Stop Kinect
int KinectViewer::stop() {
    if(interface->isRunning() == false)
        return -1;
    // Not running
    //   bRun = false;
    // Stop interface
    interface->stop();
    return 0;
}





void KinectViewer::on_checkBox_includeOrientation_clicked(bool checked)
{
    if (checked)
    {
        stop();
        if (myAda.open() == SUCCESS)
            myAda.init();
        myAda.setPlayMode(false);
        run();
    }
    else
    {
        myAda.close();
    }
}

void KinectViewer::on_pushButton_save_clicked()
{

    QString fulldir = QString("%1/PointClouds/%2-AcqKinFu")
            .arg(QDir::homePath())
            .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd-HH-mm"));

    if (!QDir(fulldir).exists())
        QDir().mkdir(fulldir);

    foreach (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC, PCtoSave)
    {
        bool binary = true;
        QString fullpath = QString("%1/%2.pcd").arg(fulldir)
                .arg(QString::fromStdString(PC->header.frame_id));
        qDebug() << fullpath;
        pcl::io::savePCDFile(fullpath.toStdString(), *PC, binary);
    }
}

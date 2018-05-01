#include "kinecttab.h"
#include "ui_kinecttab.h"


KinectTab::KinectTab(std::string serial) : ui(new Ui::KinectTab)
{
    ui->setupUi(this);

    ada = new Adafruit_UART();

    //_PoseAHRSOnKin = Eigen::Matrix4f::Identity();
   // _PoseNexCaveOnKin = Eigen::Matrix4f::Identity();
  //  _PoseAdaOnKin = Eigen::Matrix4f::Identity();

    ui->rB_Pipeline_GL->setEnabled(Kin.isGLcapable());
 //   ui->rB_Pipeline_GL->setChecked(Kin.isGLcapable());
    ui->rB_Pipeline_CL->setEnabled(Kin.isCLcapable());
   // ui->rB_Pipeline_CL->setChecked(Kin.isCLcapable());

}

KinectTab::~KinectTab()
{
    delete ui;
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

void KinectTab::on_pushButton_Open_clicked()
{
    if (Kin.isOpen())
        return;

    Kin.Open();


    ui->label_Firmware->setText(QString("Firmware: %1").arg(Kin.getFirmwareVersion()));
    ui->label_Status->setText("Status: O - Connected");

    ui->groupBox_Device->setEnabled(true);
    ui->groupBox_Pose->setEnabled(true);

    on_pushButton_getPose_clicked();


    loadMatrix("../PCViewer/NexCaveOnKin.txt", _PoseNexCaveOnKin);
    loadMatrix("../PCViewer/AHRSOnKin.txt", _PoseAHRSOnKin);

    return;
}

void KinectTab::on_pushButton_Close_clicked()
{
    if (!Kin.isOpen())
        return;

    Kin.Close();


    ui->label_Status->setText("Status: X - Disconnected");
    ui->groupBox_Device->setEnabled(false);
    ui->groupBox_Pose->setEnabled(false);

    return;
}

#include <QDir>
void KinectTab::on_pushButton_Grab_clicked()
{
    if (!Kin.isOpen())
        return;


    PointCloudT::Ptr PC(new PointCloudT);
    Kin.GrabPointCloud(PC);


/*
    if (useNexCave)
    {
        tkr->mainloop();
        tkr->mainloop();
        std::cout << "real start" << std::endl;
        tc1->grabbed = false ;
        int i = 0;
        QTime t;
        t.start();
        while (tc1->grabbed == false && i < 100)
        {
            i++;
            tkr->mainloop();
            vrpn_SleepMsecs(1); // Sleep for 1ms so we don't eat the CPU
            std::cout << "i = " << i << " / Time : "<< t.elapsed() << std::endl;

        }

        Eigen::Matrix4f NexCavePose = tc1->Pose * _PoseNexCaveOnKin;

        PC->sensor_orientation_ = Eigen::Quaternionf(NexCavePose.block<3,3>(0,0));
        PC->sensor_origin_ = NexCavePose.block<4,1>(0,3);
    }
*/

    if ( useAda && ada->isOpen() )
    {
     //   Eigen::Matrix4f _AdaPose = Eigen::Matrix4f::Identity();
     //   Eigen::Quaternionf quat = ada->returnPose();

   /*     if (ada->GetQuat(&quat) != SUCCESS)
            if (ada->GetQuat(&quat) != SUCCESS)
                ada->GetQuat(&quat);
        /*    if (ada->GetQuat(&quat) != SUCCESS)
                    if (ada->GetQuat(&quat) != SUCCESS)*/
        // return ERROR;
        //

     //  std::cout << quat.matrix() << std::endl;
      //  _AdaPose.block(0,0,3,3) = quat.matrix();

       std::cout << ada->getCalibPose().matrix() << std::endl;
        Eigen::Matrix4f _PoseKinect = Eigen::Matrix4f::Identity();//_AdaPose * _PoseAdaOnKin;
       // _PoseKinect.block(0,0,3,3) = ada->returnPose().matrix();

        PC->sensor_orientation_ = ada->returnPose(); //Eigen::Quaternionf(_PoseKinect.block<3,3>(0,0));
        PC->sensor_origin_ = _PoseKinect.block<4,1>(0,3);

    }

    if (useAHRS)
    {
        Eigen::Matrix4f _AHRSPose = Eigen::Matrix4f::Identity();
        Eigen::Quaternionf quat = Eigen::Quaternionf::Identity();

        WithRobot::SensorData sensor_data;

        std::cout << "waiting..." << std::endl;
        if(AHRS.wait_data() == true)
        { // waiting for new data

            std::cout << "grabbed" << std::endl;
            AHRS.get_data(sensor_data);

            WithRobot::Quaternion& q = sensor_data.quaternion;


            quat.w() = q.w;
            quat.x() = q.x;
            quat.y() = q.y;
            quat.z() = q.z;


            std::cout << quat.matrix() << std::endl;
            _AHRSPose.block(0,0,3,3) = quat.matrix();

            Eigen::Matrix4f  _PoseKinect = _AHRSPose * _PoseAHRSOnKin;

            PC->sensor_orientation_ = Eigen::Quaternionf(_PoseKinect.block<3,3>(0,0));
            PC->sensor_origin_ = _PoseKinect.block<4,1>(0,3);
        }
    }






    emit PCgrabbed(PC);

    return;
}




/*Pose Kinect*/
void KinectTab::on_pushButton_setPose_clicked()
{
    std::cout << " setPose clicked" << std::endl;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose = ui->TrasfMat->getMatrix();

    Kin.setPose(pose);
    return;
}

void KinectTab::on_pushButton_getPose_clicked()
{
    std::cout << " getPose clicked" << std::endl;
    ui->TrasfMat->setMatrix(Kin.getPose());
    return;
}

void KinectTab::on_pushButton_resetPose_clicked()
{
    std::cout << " resetPose clicked" << std::endl;

    Kin.setPose(Eigen::Matrix4f::Identity());

    on_pushButton_getPose_clicked();

    return;
}

void KinectTab::on_pushButton_savePose_clicked()
{
    std::cout << " savePose clicked" << std::endl;

    QFile file(QString("%1/PointClouds/ParamKinect/%2_Pose.txt").arg(QDir::homePath()).arg(Kin.getSerialNumber()) );

    file.remove();

    Eigen::Matrix4f _KinectPose = Kin.getPose();

    if (file.open(QIODevice::ReadWrite))
    {
        QTextStream stream(&file);
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                stream << _KinectPose(i,j);
                if (j==3) stream << endl;
                else stream << " ";
            }
        }
    }
    else
    {
        std::cout << "non si apre" << std::endl;
    }

    file.close();

    return;
}

void KinectTab::on_pushButton_registerPose_clicked()
{
    //  emit Registration(_LinkedKinect->getKinectIndex(), ui->spinBox_registerPose->value());
    return;
}






bool KinectTab::Open_KinectPose()
{

    std::cout << "Open - KinectPose" << std::endl;
    QFile file(QString("%1/PointClouds/ParamKinect/%2_Pose.txt").arg(QDir::homePath()).arg(Kin.getSerialNumber()) );


    if(!file.exists() || !file.open(QIODevice::ReadOnly))
        return true;

    Eigen::Matrix4f _KinectPose = Eigen::Matrix4f::Identity();

    QTextStream in(&file);
    int i = 0;
    while(!(in.atEnd()))
    {
        QString line = in.readLine();
        QStringList fields = line.split(" ");

        for (int j = 0; j < fields.size(); j++)
            _KinectPose(i,j) = fields.at(j).toFloat();

        i++;
    }

    Kin.setPose(_KinectPose);

    file.close();
    return false;
}



//Eigen::Matrix4f _PoseNexCaveOnKin;
/*
void handle_tracker_pos_quat(void *userdata, const vrpn_TRACKERCB t)
{
    std::cout << "init Callback" << std::endl;
    t_user_callback *t_data = static_cast<t_user_callback *>(userdata);

    Eigen::Vector3f trans;
    trans[0] = t.pos[0]; trans[1] = t.pos[1]; trans[2] = t.pos[2];

    Eigen::Quaternionf quat;
    quat.x() = t.quat[0]; quat.y() = t.quat[1]; quat.z() = t.quat[2]; quat.w() = t.quat[3];

    Eigen::Matrix4f Pose = Eigen::Matrix4f::Identity();
    Pose.block(0,0,3,3) = quat.matrix();
    Pose.block(0,3,3,1) = trans;

    //Pose = Pose * _PoseNexCaveOnKin;

    t_data->Pose = Pose;
    t_data->grabbed = true;
    std::cout << "end Callback" << std::endl;

}*/



void KinectTab::on_checkBox_Adafruit_clicked(bool checked)
{
    if (checked)
    {
        ada = new Adafruit_UART();


        Eigen::Matrix4f _PoseAdaOnKin;
        loadMatrix("../PCViewer/AdaOnKin.txt", _PoseAdaOnKin);
        std::cout << _PoseAdaOnKin << std::endl;
        Eigen::Quaternionf q = Eigen::Quaternionf(_PoseAdaOnKin.block<3,3>(0,0));
        std::cout << q.matrix() << std::endl;
        ada->setCalibPose(q);
        std::cout << ada->getCalibPose().matrix() << std::endl;


        if(ada->open() != SUCCESS)
            ui->checkBox_Adafruit->setChecked(false);

        else
            useAda = true;
    }
    else
    {
        useAda = false;
        ada->close();
    }
}

void KinectTab::on_pushButton_RotationSensor_Init_clicked()
{
    if (ada->isOpen())
        ada->init();
}



void KinectTab::on_checkBox_myAHRS_clicked(bool checked)
{
    if (checked)
    {
        if(AHRS.start("/dev/ttyACM0", 115200) == false)
        {
            std::cout << "start() returns false" << std::endl;
            return;
        }

        /*
           *  set binary output format
           *   - select Quaternion and IMU data
           */
        if(AHRS.cmd_binary_data_format("QUATERNION") == false)
        {
            std::cout << "cmd_binary_data_format() returns false" << std::endl;
            //     handle_error("cmd_binary_data_format() returns false");
            return;
        }

        /*
           *  set divider
           *   - output rate(Hz) = max_rate/divider
           */
        if(AHRS.cmd_divider("1") ==false)
        {
            std::cout << "cmd_divider() returns false" << std::endl;
            //   handle_error("cmd_divider() returns false");
            return;
        }

        /*
           *  set transfer mode
           *   - BC : Binary Message & Continuous mode
           */
        if(AHRS.cmd_mode("BC") ==false)
        {
            std::cout << "cmd_mode() returns false" << std::endl;
            //  handle_error("cmd_mode() returns false");
            return;
        }

        useAHRS = true;
    }
    else
    {
        AHRS.stop();
        useAHRS = false;
    }
}


void KinectTab::on_checkBox_NexCave_clicked(bool checked)
{
    if (checked)
    {
/*
        loadMatrix("../PCViewer/NexCaveOnKin.txt", _PoseNexCaveOnKin);

        QString myName("dtrackbody@109.171.138.177");

        tkr = new vrpn_Tracker_Remote(myName.toStdString().c_str());

        tc1 = new t_user_callback;
        if (tc1 == NULL)
            fprintf(stderr, "Out of memory\n");

        tc1->Pose = Eigen::Matrix4f::Identity();

        tkr->register_change_handler(tc1, handle_tracker_pos_quat, vrpn_ALL_SENSORS);



        tkr->mainloop();

        useNexCave = true;*/
    }
    else
    {
        useNexCave = false;
    }
}



void KinectTab::on_rB_Pipeline_CPU_clicked(bool checked)
{
    if (checked)
        Kin.setPipeline("CPU");
    return;
}

void KinectTab::on_rB_Pipeline_GL_clicked(bool checked)
{
    if (checked)
        Kin.setPipeline("GL");
    return;
}

void KinectTab::on_rB_Pipeline_CL_clicked(bool checked)
{
    if (checked)
        Kin.setPipeline("CL");
    return;
}

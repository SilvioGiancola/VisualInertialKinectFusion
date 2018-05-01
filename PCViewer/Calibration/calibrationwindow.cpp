#include "calibrationwindow.h"
#include "ui_calibrationwindow.h"


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    CalibrationWindow w;
    w.showMaximized();

    return a.exec();
}

CalibrationWindow::CalibrationWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::CalibrationWindow)
{
    ui->setupUi(this);
    ui->centralwidget->hide();
}

CalibrationWindow::~CalibrationWindow()
{
    ada.close();
    delete ui;
}






///////////////////////
/// \brief CalibrationWindow::on_Adafruit_Open_clicked
/// Open a connection to the adafruit sensor
void CalibrationWindow::on_Adafruit_Open_clicked()
{
    if (ada.open() == SUCCESS)
        ui->statusbar->showMessage(QString("Opening OK"));
    else
        ui->statusbar->showMessage(QString("Opening ERROR"));


}

////////////////////////
/// \brief CalibrationWindow::on_Adafruit_Init_clicked
/// Initialize the REGISTER for the Adafruit Sensor
void CalibrationWindow::on_Adafruit_Init_clicked()
{
    if (ada.init() == SUCCESS)
        ui->statusbar->showMessage(QString("Initialization OK"));
    else
        ui->statusbar->showMessage(QString("Initialization ERROR"));


}

////////////////////////
/// \brief CalibrationWindow::on_AdaFruit_Close_clicked
/// Close the connection with the adafruit sensor
void CalibrationWindow::on_AdaFruit_Close_clicked()
{
    ada.close();
}

////////////////////////////
/// \brief CalibrationWindow::on_Adafruit_Grab_clicked
/// Grad an Quaternion from the adafruit sensor
void CalibrationWindow::on_Adafruit_Grab_clicked()
{
    Eigen::Quaternionf * quat = new Eigen::Quaternionf();
    if(ada.GetQuat(quat) == SUCCESS)
        ui->statusbar->showMessage(QString("Grabbing OK"));
    else
        ui->statusbar->showMessage(QString("Grabbing ERROR"));

    emit ShowCoordinateSystem(0.5, Eigen::Affine3f(quat->matrix()), "Adafruit");

    if (ui->Adafruit_SavePoses->isChecked())
    {
        Eigen::Matrix4f Pose = Eigen::Matrix4f::Identity();
        Pose.block(0,0,3,3) = quat->matrix();
        PosesAdafruit.append(Pose);
        QString Text = QString("\n%1 %2 %3 %4 %5 %6 %7")
                .arg(0).arg(0).arg(0)
                .arg(quat->w()).arg(quat->x()).arg(quat->y()).arg(quat->z());
        QString Message = ui->Adafruit_ListPoses->text().append(Text);
        ui->Adafruit_ListPoses->setText(Message);

    }
}

void CalibrationWindow::on_Adafruit_EmptyList_clicked()
{
    ui->Adafruit_ListPoses->setText(QString("X, Y, Z, qW, qX, qY, qZ"));
}




void CalibrationWindow::on_Adafruit_getCal_clicked()
{
    //ada.GetQuat()
}








void handle_tracker_pos_quat(void *userdata, const vrpn_TRACKERCB t)
{
   t_user_callback *t_data = static_cast<t_user_callback *>(userdata);

    Eigen::Vector3f trans;
    trans[0] = t.pos[0]; trans[1] = t.pos[1]; trans[2] = t.pos[2];

    Eigen::Quaternionf quat;
    quat.x() = t.quat[0]; quat.y() = t.quat[1]; quat.z() = t.quat[2]; quat.w() = t.quat[3];

    Eigen::Matrix4f Pose = Eigen::Matrix4f::Identity();
    Pose.block(0,0,3,3) = quat.matrix();
    Pose.block(0,3,3,1) = trans;

    t_data->Pose = Pose;

}

void CalibrationWindow::on_NexCave_Open_clicked()
{
    QString myName("dtrackbody@109.171.138.177");

    tkr = new vrpn_Tracker_Remote(myName.toStdString().c_str());

    tc1 = new t_user_callback;
    if (tc1 == NULL)
        fprintf(stderr, "Out of memory\n");

    tkr->register_change_handler(tc1, handle_tracker_pos_quat, vrpn_ALL_SENSORS);

    tkr->mainloop();
}

void CalibrationWindow::on_NexCave_Grab_clicked()
{
    tkr->mainloop();
    tkr->mainloop();
    tkr->mainloop();
    tkr->mainloop();
    emit ShowCoordinateSystem(0.5, Eigen::Affine3f(tc1->Pose), "NexCave");

    if (ui->NexCave_SavePoses->isChecked())
    {
        PosesNexCave.append(tc1->Pose);
        Eigen::Vector4f T = tc1->Pose.block<4,1>(0,3);
        Eigen::Quaternionf quat = Eigen::Quaternionf(tc1->Pose.block<3,3>(0,0));
        QString Pose = QString("\n%1 %2 %3 %4 %5 %6 %7")
                .arg(T[0]).arg(T[1]).arg(T[2])
                .arg(quat.w()).arg(quat.x()).arg(quat.y()).arg(quat.z());
        QString Message = ui->NexCave_ListPoses->text().append(Pose);
        ui->NexCave_ListPoses->setText(Message);

    }
}

void CalibrationWindow::on_NexCave_EmptyList_clicked()
{
    ui->NexCave_ListPoses->setText(QString("X, Y, Z, qW, qX, qY, qZ"));
}






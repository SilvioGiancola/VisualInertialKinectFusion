#include "calibration_adafruit.h"
#include "ui_calibration_adafruit.h"

Calibration_Adafruit::Calibration_Adafruit(QWidget *parent) :
    QWidget(parent),ui(new Ui::Calibration_Adafruit)
{
    ui->setupUi(this);
}

Calibration_Adafruit::~Calibration_Adafruit()
{
    delete ui;
}

using namespace std;

void Calibration_Adafruit::on_pushButton_Save_AdaPoses_clicked()
{
    AdaPoseList.clear();
    QString LabelText = QString("X, Y, Z, qW, qX, qY,qZ");

    for (int i = 0; i< CurrentPCList.size(); i++)
    {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block(0,0,3,3) = CurrentPCList.at(i)->sensor_orientation_.matrix();
        pose.block(0,3,4,1) = CurrentPCList.at(i)->sensor_origin_;
        AdaPoseList.append(pose);
        LabelText.append(QString("\n%1 %2 %3 %4 %5 %6 %7")
                         .arg(CurrentPCList.at(i)->sensor_origin_.x())
                         .arg(CurrentPCList.at(i)->sensor_origin_.y())
                         .arg(CurrentPCList.at(i)->sensor_origin_.z())
                         .arg(CurrentPCList.at(i)->sensor_orientation_.w())
                         .arg(CurrentPCList.at(i)->sensor_orientation_.x())
                         .arg(CurrentPCList.at(i)->sensor_orientation_.y())
                         .arg(CurrentPCList.at(i)->sensor_orientation_.z()));
    }


    ui->label_AdaPoses->setText(LabelText);
}

void Calibration_Adafruit::on_pushButton_Save_RegPoses_clicked()
{
    RegPoseList.clear();
    QString LabelText = QString("X, Y, Z, qW, qX, qY,qZ");

    for (int i = 0; i< CurrentPCList.size(); i++)
    {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block(0,0,3,3) = CurrentPCList.at(i)->sensor_orientation_.matrix();
        pose.block(0,3,4,1) = CurrentPCList.at(i)->sensor_origin_;
        RegPoseList.append(pose);
        LabelText.append(QString("\n%1 %2 %3 %4 %5 %6 %7")
                         .arg(CurrentPCList.at(i)->sensor_origin_.x())
                         .arg(CurrentPCList.at(i)->sensor_origin_.y())
                         .arg(CurrentPCList.at(i)->sensor_origin_.z())
                         .arg(CurrentPCList.at(i)->sensor_orientation_.w())
                         .arg(CurrentPCList.at(i)->sensor_orientation_.x())
                         .arg(CurrentPCList.at(i)->sensor_orientation_.y())
                         .arg(CurrentPCList.at(i)->sensor_orientation_.z()));
    }


    ui->label_RegPoses->setText(LabelText);
}


//#include <engine.h>
void Calibration_Adafruit::on_pushButton_Calibrate_Ada_clicked()
{



    /*

%     bHg - pose of gripper relative to the robot base..
%           (Gripper center is at: g0 = Hbg * [0;0;0;1] )
%           Matrix dimensions are 4x4xM, where M is ..
%           .. number of camera positions.
%           Algorithm gives a non-singular solution when ..
%           .. at least 3 positions are given
%           Hbg(:,:,i) is i-th homogeneous transformation matrix
%     wHc - pose of camera relative to the world ..
%           (relative to the calibration block)
%           Dimension: size(Hwc) = size(Hbg)
%     gHc - 4x4 homogeneous transformation from gripper to camera
%           , that is the camera position relative to the gripper.
%           Focal point of the camera is positioned, ..
%           .. relative to the gripper, at
%                 f = gHc*[0;0;0;1];

    int M = RegPoseList.size();
    // M = size(bHg,3);

    int K = (M*M-M)/2; //              % Number of unique camera position pairs
    Eigen::MatrixX3f A = Eigen::Matrix3Xf::Zero(3*K,3);    //        % will store: skew(Pgij+Pcij)
    Eigen::MatrixX3f B = Eigen::Matrix3Xf::Zero(3*K,1);      //      % will store: Pcij - Pgij
    int k = 0;


    //   % Now convert from wHc notation to Hc notation used in Tsai paper.
    //Hg = bHg;
    //% Hc = cHw = inv(wHc); We do it in a loop because wHc is given, not cHw
    //  Hc = zeros(4,4,M);
    QList<Eigen::Matrix4f> Hc;
    for (int i = 0; i < M; i++)// = 1:M,
    {
        Hc.append(AdaPoseList.at(i).inverse());
    }

    //  for i = 1:M,
    //      for j = i+1:M;
    for (int i = 0; i < M; i++)
    {
        for (int j = i+1; j < M; j++)
        {
            Eigen::Matrix4f Hgij = RegPoseList.at(j).inverse() * RegPoseList.at(i);
            Eigen::Quaternionf Pgij = 2*Eigen::Quaternionf(Hgij);

            Eigen::Matrix4f Hcij = AdaPoseList.at(j) * AdaPoseList.at(i).inverse();
            Eigen::Quaternionf Pcij = 2*Eigen::Quaternionf(Hcij);
            //  Hgij = inv(Hg(:,:,j))*Hg(:,:,i);    % Transformation from i-th to j-th gripper pose
            //         Pgij = 2*rot2quat(Hgij);            % ... and the corresponding quaternion

          //  Hcij = Hc(:,:,j)*inv(Hc(:,:,i));    % Transformation from i-th to j-th camera pose
                //    Pcij = 2*rot2quat(Hcij);            % ... and the corresponding quaternion

                    k = k+1; //                           % Form linear system of equations
                    m(0, 1) = 1;
                  m(0, 2) = 2;
                   m(0, 3) = 3;

                    A((3*k-3)+(1:3), 1:3) = skew(Pgij+Pcij); % left-hand side
                    B((3*k-3)+(1:3))      = Pcij - Pgij;     % right-hand side
        }
    }
    //     end;
    //  end;

    % Rotation from camera to gripper is obtained from the set of equations:
    %    skew(Pgij+Pcij) * Pcg_ = Pcij - Pgij
    % Gripper with camera is first moved to M different poses, then the gripper
    % .. and camera poses are obtained for all poses. The above equation uses
    % .. invariances present between each pair of i-th and j-th pose.

    Pcg_ = A \ B;                % Solve the equation A*Pcg_ = B

    % Obtained non-unit quaternin is scaled back to unit value that
    % .. designates camera-gripper rotation
    Pcg = 2 * Pcg_ / sqrt(1 + Pcg_'*Pcg_);

    Rcg = quat2rot(Pcg/2);         % Rotation matrix


    % Calculate translational component
    k = 0;
    for i = 1:M,
        for j = i+1:M;
            Hgij = inv(Hg(:,:,j))*Hg(:,:,i);    % Transformation from i-th to j-th gripper pose
            Hcij = Hc(:,:,j)*inv(Hc(:,:,i));    % Transformation from i-th to j-th camera pose

            k = k+1;                            % Form linear system of equations
            A((3*k-3)+(1:3), 1:3) = Hgij(1:3,1:3)-eye(3); % left-hand side
            B((3*k-3)+(1:3))      = Rcg(1:3,1:3)*Hcij(1:3,4) - Hgij(1:3,4);     % right-hand side

        end;
    end;

    Tcg = A \ B;

    gHc = transl(Tcg) * Rcg;	% incorporate translation with rotation


    return*/
}

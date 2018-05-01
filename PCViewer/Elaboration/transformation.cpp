#include "transformation.h"
#include "ui_transformation.h"

Transformation::Transformation(QWidget *parent) :
    QWidget(parent), ui(new Ui::Transformation)
{
    ui->setupUi(this);
}

Transformation::~Transformation()
{
    delete ui;
}




/*
 * Global Transformation click handling
 */
void Transformation::on_Transform_setMat_clicked()
{
    // check consistency
    if (CurrentPC == NULL)
        return;

    // define rotation and point clouds
    PointCloudT::Ptr input(new PointCloudT);
    pcl::copyPointCloud(*CurrentPC, *input);
    PointCloudT::Ptr output(new PointCloudT);
    pcl::copyPointCloud(*input, *output);

    Eigen::Matrix4f trans = ui->Transform_RT->getMatrix();

    output->sensor_origin_ = trans.block<4,1>(0,3);
    output->sensor_orientation_ = Eigen::Quaternionf(trans.block<3,3>(0,0));

    emit replace(input, output, QString("Set Pose"));

    return;
}

void Transformation::on_Transform_getMat_clicked()
{
    // check consistency
    if (CurrentPC == NULL)
        return;

    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans.block(0,0,3,3) = CurrentPC->sensor_orientation_.matrix();
    trans.block(0,3,4,1) = CurrentPC->sensor_origin_;
    ui->Transform_RT->setMatrix(trans);

    return;
}

void Transformation::on_Transform_reset_clicked()
{
    // check consistency
    if (CurrentPC == NULL)
        return;

    // define rotation and point clouds
    PointCloudT::Ptr input(new PointCloudT);
    pcl::copyPointCloud(*CurrentPC, *input);
    PointCloudT::Ptr output(new PointCloudT);
    pcl::copyPointCloud(*input, *output);

    // Reset PC
    output->sensor_orientation_ = Eigen::Quaternionf::Identity();
    output->sensor_origin_ = Eigen::Vector4f::Zero();

    // Reset Rt Matrix Spin Box
    ui->Transform_RT->setMatrix(Eigen::Matrix4f::Identity());

    // update Viewer
    emit replace(input, output, QString("Reset Pose"));

    return;
}

void Transformation::on_Transform_solidify_clicked()
{
    // check consistency
    if (CurrentPC == NULL)
        return;

    // define rotation and point clouds
    PointCloudT::Ptr input(new PointCloudT);
    pcl::copyPointCloud(*CurrentPC, *input);
    PointCloudT::Ptr output(new PointCloudT);
    pcl::copyPointCloud(*input, *output);

    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans.block(0,0,3,3) = input->sensor_orientation_.matrix();
    trans.block(0,3,3,1) = input->sensor_origin_.head(3);

    pcl::transformPointCloud(*input, *output, trans);

    output->sensor_orientation_ = Eigen::Quaternionf::Identity();
    output->sensor_origin_ = Eigen::Vector4f::Zero();

    // update Viewer
    emit replace(input, output, QString("Solidify Pose"));

    // show currrent matrix
    on_Transform_getMat_clicked();

    return;
}




/*
 * Rotation Transformation click handling
 */
void Transformation::on_Transform_Rxneg_clicked()
{
    if (ui->checkBox_ALL->isChecked())
    {
        for (int i = 0; i < CurrentPCList.size(); i++)
        {
            if (CurrentPCList.at(i) == NULL)
                continue;

            // define rotation and point clouds
            PointCloudT::Ptr input(new PointCloudT);
            pcl::copyPointCloud(*CurrentPCList.at(i), *input);
            PointCloudT::Ptr output(new PointCloudT);

            // process
            Rotate(input, output, Eigen::Quaternionf( Eigen::AngleAxisf(ui->Transform_RStep->value()*std::acos(-1)/180, Eigen::Vector3f(-1,0,0) ) ) );

        }
    }
    else
    {
        // check consistency
        if (CurrentPC == NULL)
            return;

        // define rotation and point clouds
        PointCloudT::Ptr input(new PointCloudT);
        pcl::copyPointCloud(*CurrentPC, *input);
        PointCloudT::Ptr output(new PointCloudT);

        // process
        Rotate(input, output, Eigen::Quaternionf( Eigen::AngleAxisf(ui->Transform_RStep->value()*std::acos(-1)/180, Eigen::Vector3f(-1,0,0) ) ) );
    }
    return;
}

void Transformation::on_Transform_Rxpos_clicked()
{
    if (ui->checkBox_ALL->isChecked())
    {
        for (int i = 0; i < CurrentPCList.size(); i++)
        {
            if (CurrentPCList.at(i) == NULL)
                continue;

            // define rotation and point clouds
            PointCloudT::Ptr input(new PointCloudT);
            pcl::copyPointCloud(*CurrentPCList.at(i), *input);
            PointCloudT::Ptr output(new PointCloudT);

            // process
            Rotate(input, output, Eigen::Quaternionf( Eigen::AngleAxisf(ui->Transform_RStep->value()*std::acos(-1)/180, Eigen::Vector3f(1,0,0) ) ) );

        }
    }
    else
    {
        // check consistency
        if (CurrentPC == NULL)
            return;

        // define rotation and point clouds
        PointCloudT::Ptr input(new PointCloudT);
        pcl::copyPointCloud(*CurrentPC, *input);
        PointCloudT::Ptr output(new PointCloudT);

        // process
        Rotate(input, output, Eigen::Quaternionf( Eigen::AngleAxisf(ui->Transform_RStep->value()*std::acos(-1)/180, Eigen::Vector3f(1,0,0) ) ) );

    }
    return;
}

void Transformation::on_Transform_Ryneg_clicked()
{ if (ui->checkBox_ALL->isChecked())
    {
        for (int i = 0; i < CurrentPCList.size(); i++)
        {
            if (CurrentPCList.at(i) == NULL)
                continue;

            // define rotation and point clouds
            PointCloudT::Ptr input(new PointCloudT);
            pcl::copyPointCloud(*CurrentPCList.at(i), *input);
            PointCloudT::Ptr output(new PointCloudT);

            // process
            Rotate(input, output, Eigen::Quaternionf( Eigen::AngleAxisf(ui->Transform_RStep->value()*std::acos(-1)/180, Eigen::Vector3f(0,-1,0) ) ) );

        }
    }
    else
    {
        // check consistency
        if (CurrentPC == NULL)
            return;

        // define rotation and point clouds
        PointCloudT::Ptr input(new PointCloudT);
        pcl::copyPointCloud(*CurrentPC, *input);
        PointCloudT::Ptr output(new PointCloudT);

        // process
        Rotate(input, output, Eigen::Quaternionf( Eigen::AngleAxisf(ui->Transform_RStep->value()*std::acos(-1)/180, Eigen::Vector3f(0,-1,0) ) ) );
    }
    return;
}

void Transformation::on_Transform_Rypos_clicked()
{ if (ui->checkBox_ALL->isChecked())
    {
        for (int i = 0; i < CurrentPCList.size(); i++)
        {
            if (CurrentPCList.at(i) == NULL)
                continue;

            // define rotation and point clouds
            PointCloudT::Ptr input(new PointCloudT);
            pcl::copyPointCloud(*CurrentPCList.at(i), *input);
            PointCloudT::Ptr output(new PointCloudT);

            // process
            Rotate(input, output, Eigen::Quaternionf( Eigen::AngleAxisf(ui->Transform_RStep->value()*std::acos(-1)/180, Eigen::Vector3f(0,1,0) ) ) );

        }
    }
    else
    {
        // check consistency
        if (CurrentPC == NULL)
            return;

        // define rotation and point clouds
        PointCloudT::Ptr input(new PointCloudT);
        pcl::copyPointCloud(*CurrentPC, *input);
        PointCloudT::Ptr output(new PointCloudT);

        // process
        Rotate(input, output, Eigen::Quaternionf( Eigen::AngleAxisf(ui->Transform_RStep->value()*std::acos(-1)/180, Eigen::Vector3f(0,1,0) ) ) );
    }
    return;
}

void Transformation::on_Transform_Rzneg_clicked()
{
    if (ui->checkBox_ALL->isChecked())
    {
        for (int i = 0; i < CurrentPCList.size(); i++)
        {
            if (CurrentPCList.at(i) == NULL)
                continue;

            // define rotation and point clouds
            PointCloudT::Ptr input(new PointCloudT);
            pcl::copyPointCloud(*CurrentPCList.at(i), *input);
            PointCloudT::Ptr output(new PointCloudT);

            // process
            Rotate(input, output, Eigen::Quaternionf( Eigen::AngleAxisf(ui->Transform_RStep->value()*std::acos(-1)/180, Eigen::Vector3f(0,0,-1) ) ) );

        }
    }
    else
    {
        // check consistency
        if (CurrentPC == NULL)
            return;

        // define rotation and point clouds
        PointCloudT::Ptr input(new PointCloudT);
        pcl::copyPointCloud(*CurrentPC, *input);
        PointCloudT::Ptr output(new PointCloudT);

        // process
        Rotate(input, output, Eigen::Quaternionf( Eigen::AngleAxisf(ui->Transform_RStep->value()*std::acos(-1)/180, Eigen::Vector3f(0,0,-1) ) ) );
    }
    return;
}

void Transformation::on_Transform_Rzpos_clicked()
{
    if (ui->checkBox_ALL->isChecked())
    {
        for (int i = 0; i < CurrentPCList.size(); i++)
        {
            if (CurrentPCList.at(i) == NULL)
                continue;

            // define rotation and point clouds
            PointCloudT::Ptr input(new PointCloudT);
            pcl::copyPointCloud(*CurrentPCList.at(i), *input);
            PointCloudT::Ptr output(new PointCloudT);

            // process
            Rotate(input, output, Eigen::Quaternionf( Eigen::AngleAxisf(ui->Transform_RStep->value()*std::acos(-1)/180, Eigen::Vector3f(0,0,1) ) ) );

        }
    }
    else
    {
        // check consistency
        if (CurrentPC == NULL)
            return;

        // define rotation and point clouds
        PointCloudT::Ptr input(new PointCloudT);
        pcl::copyPointCloud(*CurrentPC, *input);
        PointCloudT::Ptr output(new PointCloudT);

        // process
        Rotate(input, output, Eigen::Quaternionf( Eigen::AngleAxisf(ui->Transform_RStep->value()*std::acos(-1)/180, Eigen::Vector3f(0,0,1) ) ) );
    }

    return;
}




/*
 * Translation Transformation click handling
 */
void Transformation::on_Transform_Txneg_clicked()
{
    if (ui->checkBox_ALL->isChecked())
    {
        for (int i = 0; i < CurrentPCList.size(); i++)
        {
            if (CurrentPCList.at(i) == NULL)
                continue;

            // define rotation and point clouds
            PointCloudT::Ptr input(new PointCloudT);
            pcl::copyPointCloud(*CurrentPCList.at(i), *input);
            PointCloudT::Ptr output(new PointCloudT);

            // process
            Translate(input, output, Eigen::Vector3f( -ui->Transform_TStep->value(), 0, 0 ));
        }
    }
    else
    {
        // check consistency
        if (CurrentPC == NULL)
            return;

        // define rotation and point clouds
        PointCloudT::Ptr input(new PointCloudT);
        pcl::copyPointCloud(*CurrentPC, *input);
        PointCloudT::Ptr output(new PointCloudT);

        // process
        Translate(input, output, Eigen::Vector3f( -ui->Transform_TStep->value(), 0, 0 ));
    }
    return;
}

void Transformation::on_Transform_Txpos_clicked()
{
    if (ui->checkBox_ALL->isChecked())
    {
        for (int i = 0; i < CurrentPCList.size(); i++)
        {
            if (CurrentPCList.at(i) == NULL)
                continue;

            // define rotation and point clouds
            PointCloudT::Ptr input(new PointCloudT);
            pcl::copyPointCloud(*CurrentPCList.at(i), *input);
            PointCloudT::Ptr output(new PointCloudT);

            // process
            Translate(input, output, Eigen::Vector3f( ui->Transform_TStep->value(), 0, 0 ));
        }
    }
    else
    {
        // check consistency
        if (CurrentPC == NULL)
            return;

        // define rotation and point clouds
        PointCloudT::Ptr input(new PointCloudT);
        pcl::copyPointCloud(*CurrentPC, *input);
        PointCloudT::Ptr output(new PointCloudT);

        // process
        Translate(input, output, Eigen::Vector3f( ui->Transform_TStep->value(), 0, 0 ));
    }
    return;
}

void Transformation::on_Transform_Tyneg_clicked()
{
    if (ui->checkBox_ALL->isChecked())
    {
        for (int i = 0; i < CurrentPCList.size(); i++)
        {
            if (CurrentPCList.at(i) == NULL)
                continue;

            // define rotation and point clouds
            PointCloudT::Ptr input(new PointCloudT);
            pcl::copyPointCloud(*CurrentPCList.at(i), *input);
            PointCloudT::Ptr output(new PointCloudT);

            // process
            Translate(input, output, Eigen::Vector3f( 0, -ui->Transform_TStep->value(), 0 ));
        }
    }
    else
    {
        // check consistency
        if (CurrentPC == NULL)
            return;

        // define rotation and point clouds
        PointCloudT::Ptr input(new PointCloudT);
        pcl::copyPointCloud(*CurrentPC, *input);
        PointCloudT::Ptr output(new PointCloudT);

        // process
        Translate(input, output, Eigen::Vector3f( 0, -ui->Transform_TStep->value(), 0 ));
    }
    return;
}

void Transformation::on_Transform_Typos_clicked()
{
    if (ui->checkBox_ALL->isChecked())
    {
        for (int i = 0; i < CurrentPCList.size(); i++)
        {
            if (CurrentPCList.at(i) == NULL)
                continue;

            // define rotation and point clouds
            PointCloudT::Ptr input(new PointCloudT);
            pcl::copyPointCloud(*CurrentPCList.at(i), *input);
            PointCloudT::Ptr output(new PointCloudT);

            // process
            Translate(input, output, Eigen::Vector3f( 0, ui->Transform_TStep->value(), 0 ));
        }
    }
    else
    {
        // check consistency
        if (CurrentPC == NULL)
            return;

        // define rotation and point clouds
        PointCloudT::Ptr input(new PointCloudT);
        pcl::copyPointCloud(*CurrentPC, *input);
        PointCloudT::Ptr output(new PointCloudT);

        // process
        Translate(input, output, Eigen::Vector3f( 0, ui->Transform_TStep->value(), 0 ));
    }
    return;
}

void Transformation::on_Transform_Tzneg_clicked()
{
    if (ui->checkBox_ALL->isChecked())
    {
        for (int i = 0; i < CurrentPCList.size(); i++)
        {
            if (CurrentPCList.at(i) == NULL)
                continue;

            // define rotation and point clouds
            PointCloudT::Ptr input(new PointCloudT);
            pcl::copyPointCloud(*CurrentPCList.at(i), *input);
            PointCloudT::Ptr output(new PointCloudT);

            // process
            Translate(input, output, Eigen::Vector3f( 0, 0, -ui->Transform_TStep->value()));
        }
    }
    else
    {
        // check consistency
        if (CurrentPC == NULL)
            return;

        // define rotation and point clouds
        PointCloudT::Ptr input(new PointCloudT);
        pcl::copyPointCloud(*CurrentPC, *input);
        PointCloudT::Ptr output(new PointCloudT);
        // process
        Translate(input, output, Eigen::Vector3f( 0, 0, -ui->Transform_TStep->value()));
    }
    return;
}

void Transformation::on_Transform_Tzpos_clicked()
{
    if (ui->checkBox_ALL->isChecked())
    {
        for (int i = 0; i < CurrentPCList.size(); i++)
        {
            if (CurrentPCList.at(i) == NULL)
                continue;

            // define rotation and point clouds
            PointCloudT::Ptr input(new PointCloudT);
            pcl::copyPointCloud(*CurrentPCList.at(i), *input);
            PointCloudT::Ptr output(new PointCloudT);

            // process
            Translate(input, output, Eigen::Vector3f( 0, 0, ui->Transform_TStep->value()));
        }
    }
    else
    {
        // check consistency
        if (CurrentPC == NULL)
            return;

        // define rotation and point clouds
        PointCloudT::Ptr input(new PointCloudT);
        pcl::copyPointCloud(*CurrentPC, *input);
        PointCloudT::Ptr output(new PointCloudT);

        // process
        Translate(input, output, Eigen::Vector3f( 0, 0, ui->Transform_TStep->value()));
    }
    return;
}


void Transformation::on_Transform_demean_clicked()
{
    // check consistency
    if (CurrentPC == NULL)
        return;

    // define point clouds
    PointCloudT::Ptr input(new PointCloudT);
    pcl::copyPointCloud(*CurrentPC, *input);

    PointCloudT::Ptr tmp(new PointCloudT);
    pcl::copyPointCloud(*input, *tmp);

    PointCloudT::Ptr output(new PointCloudT);
    pcl::copyPointCloud(*input, *output);

    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans.block(0,0,3,3) = tmp->sensor_orientation_.matrix();
    trans.block(0,3,3,1) = tmp->sensor_origin_.head(3);

    pcl::transformPointCloud(*tmp, *tmp, trans);

    tmp->sensor_orientation_ = Eigen::Quaternionf::Identity();
    tmp->sensor_origin_ = Eigen::Vector4f::Zero();

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*tmp, centroid);

    // process
    Translate(input, output, -centroid.head(3));


    return;
}





/*
 * Transformation handling
 */
void Transformation::Rotate(PointCloudT::Ptr input, PointCloudT::Ptr output, Eigen::Quaternionf quaternion)
{
    // Create transf matrix from quaternion
    Eigen::Matrix4f transformationMatrix = Eigen::Matrix4f::Identity();
    transformationMatrix.block(0,0,3,3) = Eigen::Matrix3f(quaternion); //(init row, init col, nb row, nb col)

    ApplyTransformation(input, output, transformationMatrix);

    return;
}

void Transformation::Translate(PointCloudT::Ptr input, PointCloudT::Ptr output, Eigen::Vector3f translation)
{
    Eigen::Matrix4f transformationMatrix = Eigen::Matrix4f::Identity();
    transformationMatrix.block(0,3,3,1) = translation;//(init row, init col, nb row, nb col)

    ApplyTransformation(input, output, transformationMatrix);

    return;
}

void Transformation::ApplyTransformation(PointCloudT::Ptr input, PointCloudT::Ptr output,  Eigen::Matrix4f transformationMatrix)
{
    pcl::copyPointCloud(*input, *output);

    Eigen::Matrix4f currentPose = Eigen::Matrix4f::Identity();
    currentPose.block(0,0,3,3) = input->sensor_orientation_.matrix();
    currentPose.block(0,3,3,1) = input->sensor_origin_.head(3);


    if (ui->Transform_GlobalRefSyst->isChecked())
        transformationMatrix = transformationMatrix * currentPose;
    else if (ui->Transform_LocalRefSyst->isChecked())
        transformationMatrix = currentPose * transformationMatrix;


    output->sensor_origin_ = transformationMatrix.block<4,1>(0,3);
    output->sensor_orientation_ = Eigen::Quaternionf(transformationMatrix.block<3,3>(0,0));


    // update
    emit replace(input, output, QString("Transformation"));
    on_Transform_getMat_clicked();

    return;
}

#include "registration_correspondenceestimation.h"
#include "ui_registration_correspondenceestimation.h"

Registration_CorrespondenceEstimation::Registration_CorrespondenceEstimation(QWidget *parent) :
    QWidget(parent), ui(new Ui::Registration_CorrespondenceEstimation)
{
    ui->setupUi(this);
    myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimation<PointT, PointT>);
}

Registration_CorrespondenceEstimation::~Registration_CorrespondenceEstimation()
{
    delete ui;
}








void Registration_CorrespondenceEstimation::on_Registration_Base_clicked(bool checked)
{
    myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimation<PointT, PointT>);
    ui->Registration_Base->setChecked(checked);
    return;
}

void Registration_CorrespondenceEstimation::on_Registration_BackProjection_clicked(bool checked)
{
    myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimationBackProjection<PointT, PointT, PointT>);
    ui->Registration_BackProjection->setChecked(checked);
    return;
}

void Registration_CorrespondenceEstimation::on_Registration_NormalShooting_clicked(bool checked)
{
    myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimationNormalShooting<PointT, PointT, PointT>);
    ui->Registration_NormalShooting->setChecked(checked);
    return;
}

void Registration_CorrespondenceEstimation::on_Registration_OrganizedProjection_clicked(bool checked)
{
    myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimationOrganizedProjection<PointT, PointT>);
    ui->Registration_OrganizedProjection->setChecked(checked);
    return;
}

void Registration_CorrespondenceEstimation::on_checkBox_Reciprok_clicked(bool checked)
{
    _reciprok = checked;
    ui->checkBox_Reciprok->setChecked(checked);
}

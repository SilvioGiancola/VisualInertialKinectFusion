#include "registration_transformationestimation.h"
#include "ui_registration_transformationestimation.h"

Registration_TransformationEstimation::Registration_TransformationEstimation(QWidget *parent) :
    QWidget(parent), ui(new Ui::Registration_TransformationEstimation)
{
    ui->setupUi(this);

    myTransformationEstimation.reset(new pcl::registration::TransformationEstimationSVD<PointT, PointT>);
    //   myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimation<PointT, PointT>);

}

Registration_TransformationEstimation::~Registration_TransformationEstimation()
{
    delete ui;
}


using namespace std;




/////////////////
/// \brief Registration_TransformationEstimation::on_Registration_SVD_clicked
/// choose the Registration to use
void Registration_TransformationEstimation::on_Registration_SVD_clicked()
{
    myTransformationEstimation.reset(new pcl::registration::TransformationEstimationSVD<PointT, PointT>);
    return;
}

void Registration_TransformationEstimation::on_Registration_LM_clicked()
{
    myTransformationEstimation.reset(new pcl::registration::TransformationEstimationLM<PointT, PointT>);
    return;
}

void Registration_TransformationEstimation::on_Registration_Point2Plane_clicked()
{
    myTransformationEstimation.reset(new pcl::registration::TransformationEstimationPointToPlane<PointT, PointT>);
    return;
}

void Registration_TransformationEstimation::on_Registration_Point2PlaneLLS_clicked()
{
    myTransformationEstimation.reset(new pcl::registration::TransformationEstimationPointToPlaneLLS<PointT, PointT>);
    return;
}

void Registration_TransformationEstimation::on_Registration_TranslationOnly_clicked()
{
    myTransformationEstimation.reset(new pcl::registration::TransformationEstimationTranslation<PointT, PointT>);
    return;
}





// Trasnforamtion estimation

Eigen::Matrix4f Registration_TransformationEstimation::getTransformation(PointCloudT::Ptr Source, PointCloudT::Ptr Target, pcl::CorrespondencesPtr Correspondences)
{
    Eigen::Matrix4f transformation_ = Eigen::Matrix4f::Identity();
    pcl::ScopeTime t ("Transformation estimation");
    myTransformationEstimation->estimateRigidTransformation (*Source, *Target, *Correspondences, transformation_);

    return transformation_;
}

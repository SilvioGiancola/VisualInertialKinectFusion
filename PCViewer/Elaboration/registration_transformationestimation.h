#ifndef Registration_TransformationEstimation_H
#define Registration_TransformationEstimation_H

#include <QWidget>
#include <define.h>

#include <pcl/visualization/pcl_plotter.h>
//ICP
#include <pcl/correspondence.h>

#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_translation.h> // My Transformation Estimation
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>



#include <pcl/common/time.h>

namespace Ui {
class Registration_TransformationEstimation;
}


class Registration_TransformationEstimation : public QWidget
{
    Q_OBJECT

public:
    explicit Registration_TransformationEstimation(QWidget *parent = 0);
    ~Registration_TransformationEstimation();

    Eigen::Matrix4f getTransformation(PointCloudT::Ptr Source, PointCloudT::Ptr Target, pcl::CorrespondencesPtr Correspondences);
    pcl::registration::TransformationEstimation<PointT, PointT>::Ptr myTransformationEstimation;

public slots:
    void on_Registration_SVD_clicked();
    void on_Registration_LM_clicked();
    void on_Registration_Point2Plane_clicked();
    void on_Registration_Point2PlaneLLS_clicked();
    void on_Registration_TranslationOnly_clicked();



private:
    Ui::Registration_TransformationEstimation *ui;
};

#endif // Registration_TransformationEstimation_H

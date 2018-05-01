#ifndef REGISTRATION_KEYPOINTS_H
#define REGISTRATION_KEYPOINTS_H

#include <QWidget>
#include <define.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_organized_projection.h>

#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>

#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_translation.h> // My Transformation Estimation

#include <pcl/registration/icp.h> //RegistrationICP
#include <pcl/registration/icp_nl.h> //RegistrationICP
#include <pcl/registration/gicp.h> //RegistrationICP

#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/filter.h>


#include <pcl/features/pfhrgb.h>
#include <pcl/features/shot.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/pfh.h>

#include <QElapsedTimer>

#include <pcl/common/transforms.h>  // transform point cloud
#include <pcl/common/io.h> // copy point cloud

#include <pcl/visualization/pcl_visualizer.h>
namespace Ui {
class Registration_Keypoints;
}

class Registration_Keypoints : public QWidget
{
    Q_OBJECT

public:
    explicit Registration_Keypoints(QWidget *parent = 0);
    ~Registration_Keypoints();

    void setViewer(pcl::visualization::PCLVisualizer::Ptr myViewer);

public slots:
    void setCurrentPCList(QList<PointCloudT::Ptr> pcList) { CurrentPCList = pcList; updateList();}

signals:
    void replace(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC, QString ElabName);
    void addPC(PointCloudT::Ptr newPC);

private slots:
    void updateList();


    void on_pushButton_clicked();

    // DETECTION
    void DetectAGAST(PointCloudT::Ptr input, PointCloudT::Ptr output, int paramThreshold = 30);
    void DetectBRISK(PointCloudT::Ptr input, PointCloudT::Ptr output, int paramThreshold = 30, int octave = 4);
    void DetectSIFT(PointCloudT::Ptr input, PointCloudT::Ptr output);

    // DESCRIPTION
    void DescribeFPFH(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptor);
    void DescribePFH(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptor);
    void DescribePFHRGB(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descriptor);
    void DescribeSHOT(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::SHOT352>::Ptr descriptor);
    void DescribeCSHOT(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::SHOT1344>::Ptr descriptor);

    // CORRESPONDENCES FINDING
    void FindCorrespondencePoints(PointCloudT::Ptr features_1, PointCloudT::Ptr features_2, pcl::CorrespondencesPtr correspondences);
    void FindCorrespondenceFPFH (pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_1 ,pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_2, pcl::CorrespondencesPtr correspondences);
    void FindCorrespondencePFH (pcl::PointCloud<pcl::PFHSignature125>::Ptr features_1 ,pcl::PointCloud<pcl::PFHSignature125>::Ptr features_2, pcl::CorrespondencesPtr correspondences);
    void FindCorrespondencePFHRGB (pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr features_1 ,pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr features_2, pcl::CorrespondencesPtr correspondences);
    void FindCorrespondenceSHOT (pcl::PointCloud<pcl::SHOT352>::Ptr features_1 ,pcl::PointCloud<pcl::SHOT352>::Ptr features_2, pcl::CorrespondencesPtr correspondences);
    void FindCorrespondenceCSHOT (pcl::PointCloud<pcl::SHOT1344>::Ptr features_1 ,pcl::PointCloud<pcl::SHOT1344>::Ptr features_2, pcl::CorrespondencesPtr correspondences);

    void on_pushButton_Elaboration_All_PC_clicked();

private:
    Ui::Registration_Keypoints *ui;
    QList<PointCloudT::Ptr> CurrentPCList;
    pcl::visualization::PCLVisualizer::Ptr viewer;
};

#endif // REGISTRATION_KEYPOINTS_H

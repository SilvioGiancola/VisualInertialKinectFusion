#ifndef CALIBRATION_ADAFRUIT_H
#define CALIBRATION_ADAFRUIT_H

#include <QWidget>
#include <define.h>

#include <pcl/visualization/pcl_plotter.h>
//ICP
#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_organized_projection.h>

#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>

#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_translation.h>   // My Transformation Estimation

#include <pcl/registration/icp.h> //RegistrationICP
#include <pcl/registration/icp_nl.h> //RegistrationICP
#include <pcl/registration/gicp.h> //RegistrationICP

#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/sample_consensus/sac_model_registration_translation.h>    // My RANSAC Registration


#include <pcl/keypoints/brisk_2d.h>
#include <pcl/keypoints/agast_2d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/brisk_2d.h>



#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/filter.h>

#include <pcl/features/pfhrgb.h>
#include <pcl/features/shot.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/pfh.h>

#include <QElapsedTimer>
#include <QFile>
#include <QFileDialog>
#include <QTextStream>
#include <QString>
#include <QTime>

#include <algorithm>
#include <vector>

#include <pcl/common/transforms.h>  // transform point cloud
#include <pcl/common/io.h> // copy point cloud

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>


namespace Ui {
class Calibration_Adafruit;
}


class Calibration_Adafruit : public QWidget
{
    Q_OBJECT

public:
    explicit Calibration_Adafruit(QWidget *parent = 0);
    ~Calibration_Adafruit();

    void setViewer(pcl::visualization::PCLVisualizer::Ptr myViewer){viewer = myViewer; return;}

public slots:
    void setCurrentPCList(QList<PointCloudT::Ptr> pcList) { CurrentPCList = pcList;}

signals:
    void replace(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC, QString ElabName);


private slots:
    void on_pushButton_Save_AdaPoses_clicked();

    void on_pushButton_Save_RegPoses_clicked();

    void on_pushButton_Calibrate_Ada_clicked();

private:
    Ui::Calibration_Adafruit *ui;
    QList<PointCloudT::Ptr> CurrentPCList;
    pcl::visualization::PCLVisualizer::Ptr viewer;

    QList<Eigen::Matrix4f> AdaPoseList;
    QList<Eigen::Matrix4f> RegPoseList;
};

#endif // CALIBRATION_ADAFRUIT_H

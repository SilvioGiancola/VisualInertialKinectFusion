#ifndef ELABORATION_LF_REC_H
#define ELABORATION_LF_REC_H

#include <QWidget>
#include <define.h>

#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/keypoints/keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/vfh.h>
#include <QElapsedTimer>
#include <pcl/common/common.h>


namespace Ui {
class Elaboration_LF_Rec;
}
typedef pcl::Histogram<153> myhist;

class Elaboration_LF_Rec : public QWidget
{
    Q_OBJECT

public:
    explicit Elaboration_LF_Rec(QWidget *parent = 0);
    ~Elaboration_LF_Rec();
    void setPCList(QList< PointCloudT::Ptr > myPCList);
    void setViewer(pcl::visualization::PCLVisualizer::Ptr myViewer);
    void SetMaxRes ();
    void SetMinDim ();

signals:
    void PCupdated();

public slots:
    void setCurrentPCList(QList<PointCloudT::Ptr> pcList) { PCList = pcList; updateList();}

signals:
    void replace(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC, QString ElabName);
    void addPC(PointCloudT::Ptr newPC);

private slots:
    void on_Start_LF_Rec_clicked();
    void on_ComputeKeypoints_clicked();
    void updateList();

private:
    Ui::Elaboration_LF_Rec *ui;
    QList< PointCloudT::Ptr > PCList;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    PointCloudT::Ptr scene_keysOUT;
    PointCloudT::Ptr model_keysOUT;
    pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptor;
    pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptor;
    pcl::CorrespondencesPtr model_scene_corrs_OUT;
    double maxres;
    double mindim;
    QElapsedTimer * Timer;

    pcl::PointIndices  FindNANindx (pcl::PointCloud<pcl::PFHSignature125>::Ptr PC);
    pcl::PointIndices  FindNANindx (pcl::PointCloud<pcl::FPFHSignature33>::Ptr PC);
    pcl::PointIndices  FindNANindx (pcl::PointCloud<pcl::SHOT352>::Ptr PC);
    pcl::PointIndices  FindNANindx (pcl::PointCloud<myhist>::Ptr PC);

    pcl::PointCloud<pcl::PFHSignature125> cleanNAN (pcl::PointCloud<pcl::PFHSignature125>::Ptr PC, pcl::PointIndices::Ptr indices_NAN);
    pcl::PointCloud<pcl::FPFHSignature33> cleanNAN (pcl::PointCloud<pcl::FPFHSignature33>::Ptr PC, pcl::PointIndices::Ptr indices_NAN);
    pcl::PointCloud<pcl::SHOT352> cleanNAN (pcl::PointCloud<pcl::SHOT352>::Ptr PC, pcl::PointIndices::Ptr indices_NAN);
    pcl::PointCloud<myhist> cleanNAN (pcl::PointCloud<myhist>::Ptr PC, pcl::PointIndices::Ptr indices_NAN);
    PointCloudT cleanNAN (PointCloudT::Ptr PC, pcl::PointIndices::Ptr indices_NAN);

    pcl::Correspondences findCorrs (pcl::PointCloud<pcl::PFHSignature125>::Ptr scene_descr, pcl::PointCloud<pcl::PFHSignature125>::Ptr model_descr);
    pcl::Correspondences findCorrs (pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_descr, pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_descr);
    pcl::Correspondences findCorrs (pcl::PointCloud<pcl::SHOT352>::Ptr scene_descr, pcl::PointCloud<pcl::SHOT352>::Ptr model_descr);
    pcl::Correspondences findCorrs (pcl::PointCloud<myhist>::Ptr scene_descr, pcl::PointCloud<myhist>::Ptr model_descr);

//    void visualize_keys (PointCloudT keypoints, QColor color);
};

#endif // ELABORATION_LF_REC_H

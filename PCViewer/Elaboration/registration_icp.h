#ifndef REGISTRATION_ICP_H
#define REGISTRATION_ICP_H

#include <QWidget>
#include <define.h>

#include <pcl/visualization/pcl_plotter.h>
//ICP
#include <pcl/correspondence.h>

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


#include <iostream>
#include <algorithm>
#include <vector>
#include <iterator>
#include <memory>


#include <pcl/common/transforms.h>  // transform point cloud
#include <pcl/common/io.h> // copy point cloud
#include <pcl/common/time.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>

#include <pcl/PCLPointCloud2.h>


namespace Ui {
class Registration_ICP;
}


class Registration_ICP : public QWidget
{
    Q_OBJECT

public:
    explicit Registration_ICP(QWidget *parent = 0);
    ~Registration_ICP();

    void setViewer(pcl::visualization::PCLVisualizer::Ptr myViewer){viewer = myViewer; return;}

public slots:
    void setCurrentPCList(QList<PointCloudT::Ptr> pcList) { CurrentPCList = pcList; updateList();}

signals:
    void replace(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC, QString ElabName);


private slots:
    void updateList();


    void on_pushButton_ApplyOldTranslation_clicked();
    void on_pushButton_ApplyOldOrientation_clicked();

    void on_pushButton_UseNextPair_clicked();



    void on_Registration_Start_Elab_clicked();
    void on_Registration_Start_Elab_All_clicked();

    Eigen::Matrix4f DoICP(PointCloudT::Ptr PC, PointCloudT::Ptr Model, PointCloudT::Ptr FullPC, PointCloudT::Ptr FullModel);



    void on_pushButton_Calc_Dist_clicked();

    std::vector<double> getDistances(PointCloudT::Ptr PC, PointCloudT::Ptr Model, QString path = QString());






    void on_pushButton_KinFu_clicked();

    void on_Registration_Start_Elab_Ponte_clicked();

    void on_Registration_Start_Elab_PonteBis_clicked();

private:
    Ui::Registration_ICP *ui;
    QList<PointCloudT::Ptr> CurrentPCList;
    pcl::visualization::PCLVisualizer::Ptr viewer;
};
#endif // REGISTRATION_ICP_H

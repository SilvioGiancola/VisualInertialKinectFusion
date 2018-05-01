#ifndef Registration_RANSAC_H
#define Registration_RANSAC_H

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

#include <pcl/filters/conditional_removal.h>



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
class Registration_RANSAC;
}


class Registration_RANSAC : public QWidget
{
    Q_OBJECT

public:
    explicit Registration_RANSAC(QWidget *parent = 0);
    ~Registration_RANSAC();

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



    void on_pushButton_Registration_RANSAC_clicked();
    void on_pushButton_Registration_RANSAC3_clicked();



    void on_pushButton_Calc_Dist_clicked();

    std::vector<double> getDistances(PointCloudT::Ptr PC, PointCloudT::Ptr Model, QString path = QString());






private:
    Ui::Registration_RANSAC *ui;
    QList<PointCloudT::Ptr> CurrentPCList;
    pcl::visualization::PCLVisualizer::Ptr viewer;
};
#endif // Registration_RANSAC_H

#ifndef ELABORATION_PAPAZOV_H
#define ELABORATION_PAPAZOV_H

#include <QWidget>
#include <define.h>

#include <pcl/recognition/ransac_based/obj_rec_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <list>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h> //RegistrationICP
#include <pcl/registration/icp_nl.h> //RegistrationICP
#include <pcl/registration/gicp.h> //RegistrationICP
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>




namespace Ui {
class Elaboration_Papazov;
}

class Elaboration_Papazov : public QWidget
{
    Q_OBJECT

public:
    explicit Elaboration_Papazov(QWidget *parent = 0);
    ~Elaboration_Papazov();

    void setPCList(QList< PointCloudT::Ptr > myPCList);
    void setViewer(pcl::visualization::PCLVisualizer::Ptr myViewer);
    void setMinDim();
    void Initialize();

//    void setPointCloud(PointCloudT::Ptr myPC);

signals:
    void PCupdated();

public slots:
    void setCurrentPCList(QList<PointCloudT::Ptr> pcList) { PCList = pcList; updateList();}

signals:
    void replace(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC, QString ElabName);
    void addPC(PointCloudT::Ptr newPC);

private slots:
    void on_StartRecognition_clicked();

    void on_AutoSet_clicked();
    void updateList();

private:
    Ui::Elaboration_Papazov *ui;

    QList< PointCloudT::Ptr > PCList;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    int StartClickCount;
    double mindim;
    int InitialPCsize;

//    PointCloudT::Ptr PC;
};

#endif // ELABORATION_PAPAZOV_H

#ifndef Registration_CorrespondenceRejection_H
#define Registration_CorrespondenceRejection_H

#include <QWidget>
#include <define.h>

//ICP
#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h> // TODO: ADD IT !!
#include <pcl/registration/correspondence_rejection_organized_boundary.h>  // TODO: ADD IT !!


#include <pcl/common/time.h>


#include <QElapsedTimer>
#include <QFile>
#include <QFileDialog>
#include <QTextStream>
#include <QString>
#include <QTime>

#include <algorithm>
#include <vector>


namespace Ui {
class Registration_CorrespondenceRejection;
}


class Registration_CorrespondenceRejection : public QWidget
{
    Q_OBJECT

public:
    explicit Registration_CorrespondenceRejection(QWidget *parent = 0);
    ~Registration_CorrespondenceRejection();


    pcl::CorrespondencesPtr filterCorrespondences(pcl::CorrespondencesPtr correspondences, pcl::PointCloud<PointT>::Ptr Source, pcl::PointCloud<PointT>::Ptr Target);



public slots:
    void on_Registration_Max_Dist_Correspondences_clicked(bool checked);
    void on_Registration_OnetoOne_clicked(bool checked);
    void on_Registration_Trimmed_clicked(bool checked);
    void on_Registration_VarTrimmed_clicked(bool checked);
    void on_Registration_MedianDistance_clicked(bool checked);
    void on_Registration_SampleConsensus_clicked(bool checked);




private slots:

    void on_pushButton_GetList_clicked();




    void AddCorrespondenceRejectorDistance(double MaximumDistance);
    void AddCorrespondenceRejectorOneToOne();
    void AddCorrespondenceRejectorTrimmed(double OverlapRatio);
    void AddCorrespondenceRejectorVarTrimmed();
    void AddCorrespondenceRejectorMedianDistance(double MedianFactor);
    void AddCorrespondenceRejectorSampleConsensus(double InliersThreshold, int Iteration);



    void on_doubleSpinBox_Max_Dist_Correspondences_valueChanged(double arg);
    void on_Registration_SAC_MaxIteration_valueChanged(int arg);
    void on_Registration_SAC_InlierThreshold_valueChanged(double arg);
    void on_doubleSpinBox_Trimmed_valueChanged(double arg1);
    void on_Registration_MedianFactor_valueChanged(double arg1);

private:
    Ui::Registration_CorrespondenceRejection *ui;
    std::vector<pcl::registration::CorrespondenceRejector::Ptr> correspondence_rejectors_;


};

#endif // Registration_CorrespondenceRejection_H

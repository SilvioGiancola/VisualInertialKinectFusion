#ifndef Registration_CorrespondenceEstimation_H
#define Registration_CorrespondenceEstimation_H

#include <QWidget>
#include <define.h>

//ICP
#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_organized_projection.h>


#include <pcl/common/time.h>


#include <QElapsedTimer>
#include <QFile>
#include <QFileDialog>
#include <QTextStream>
#include <QString>
#include <QTime>

#include <algorithm>
#include <vector>

#include <QStandardItemModel>

namespace Ui {
class Registration_CorrespondenceEstimation;
}


class Registration_CorrespondenceEstimation : public QWidget
{
    Q_OBJECT

public:
    explicit Registration_CorrespondenceEstimation(QWidget *parent = 0);
    ~Registration_CorrespondenceEstimation();


    // template <typename PointT>
    pcl::CorrespondencesPtr getCorrespondences(pcl::PointCloud<PointT>::Ptr Source, pcl::PointCloud<PointT>::Ptr Target)
    {
        pcl::ScopeTime t ("Find Correspondences");

        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
        myCorrespondenceEstimation->setInputSource (Source);
        myCorrespondenceEstimation->setInputTarget (Target);
        if (_reciprok)
            myCorrespondenceEstimation->determineReciprocalCorrespondences (*correspondences);
        else
            myCorrespondenceEstimation->determineCorrespondences (*correspondences);

        return correspondences;
    }

    bool _reciprok = false;

public slots:
    void on_Registration_Base_clicked(bool checked);
    void on_Registration_BackProjection_clicked(bool checked);
    void on_Registration_NormalShooting_clicked(bool checked);
    void on_Registration_OrganizedProjection_clicked(bool checked);
    void on_checkBox_Reciprok_clicked(bool checked);





private:
    Ui::Registration_CorrespondenceEstimation *ui;
    pcl::registration::CorrespondenceEstimationBase<PointT, PointT>::Ptr myCorrespondenceEstimation;


};

#endif // Registration_CorrespondenceEstimation_H

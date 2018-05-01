#ifndef FILTERS_RADIUS_OUTLIERS_H
#define FILTERS_RADIUS_OUTLIERS_H

#include <QWidget>
#include <define.h>
#include <pcl/filters/radius_outlier_removal.h>   // remove radius outliers
#include <pcl/filters/statistical_outlier_removal.h>  //remove statistical outliers
#include <pcl/kdtree/kdtree_flann.h>

namespace Ui {
class Filters_Outliers;
}


class Filters_Outliers : public QWidget
{
    Q_OBJECT

public:
    explicit Filters_Outliers(QWidget *parent = 0);
    ~Filters_Outliers();

public slots:
    void setCurrentPC(PointCloudT::Ptr pc) { CurrentPC = pc; }
    void setCurrentPCList(QList<PointCloudT::Ptr> pcList) { CurrentPCList = pcList;}

signals:
    void replace(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC, QString ElabName);

private slots:
    void on_Remove_Radius_Outliers_clicked();
    void on_Remove_Radius_Outliers_ALL_clicked();

    void on_Remove_Statistical_Outliers_clicked();
    void on_Remove_Statistical_Outliers_ALL_clicked();

    void RemoveRadiusOutliers(PointCloudT::Ptr input, PointCloudT::Ptr output);

    void RemoveStatisticalOutliers(PointCloudT::Ptr input, PointCloudT::Ptr output);


private:
    Ui::Filters_Outliers *ui;
    PointCloudT::Ptr CurrentPC;
    QList<PointCloudT::Ptr> CurrentPCList;
};

#endif // FILTERS_RADIUS_OUTLIERS_H

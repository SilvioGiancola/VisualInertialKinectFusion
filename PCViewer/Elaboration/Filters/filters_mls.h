#ifndef FILTERS_MLS_H
#define FILTERS_MLS_H

#include <QWidget>
#include <define.h>
#include <pcl/surface/mls.h>        //Moving Least Square
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace Ui {
class Filters_MLS;
}


class Filters_MLS : public QWidget
{
    Q_OBJECT

public:
    explicit Filters_MLS(QWidget *parent = 0);
    ~Filters_MLS();

public slots:
    void setCurrentPC(PointCloudT::Ptr pc) { CurrentPC = pc; }
    void setCurrentPCList(QList<PointCloudT::Ptr> pcList) { CurrentPCList = pcList;}

signals:
    void replace(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC, QString ElabName);


private slots:
    void on_MLS_process_clicked();

    void on_MLS_processALL_clicked();

    void MovingLeastSquare(PointCloudT::Ptr input, PointCloudT::Ptr output);

    void on_BF_processALL_clicked();

    void on_BF_process_clicked();

    void BilateralFilter(PointCloudT::Ptr input, PointCloudT::Ptr output);

private:
    Ui::Filters_MLS *ui;
    PointCloudT::Ptr CurrentPC;
    QList<PointCloudT::Ptr> CurrentPCList;
};

#endif // FILTERS_MLS_H

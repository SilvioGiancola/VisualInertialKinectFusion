#ifndef FEATURES_NORMAL_H
#define FEATURES_NORMAL_H

#include <QWidget>
#include <define.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>

namespace Ui {
class Features_Normal;
}


class Features_Normal : public QWidget
{
    Q_OBJECT

public:
    explicit Features_Normal(QWidget *parent = 0);
    ~Features_Normal();

public slots:
    void setCurrentPC(PointCloudT::Ptr pc) { CurrentPC = pc; }
    void setCurrentPCList(QList<PointCloudT::Ptr> pcList) { CurrentPCList = pcList;}

signals:
    void replace(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC, QString ElabName);


private slots:
    void on_pushButton_Compute_clicked();
    void on_pushButton_Comput_All_Normals_clicked();

    void ComputeNormal(PointCloudT::Ptr input, PointCloudT::Ptr output);
private:
    Ui::Features_Normal *ui;
    PointCloudT::Ptr CurrentPC;
    QList<PointCloudT::Ptr> CurrentPCList;
};

#endif // FEATURES_NORMAL_H

#ifndef FILTERS_OTHERS_H
#define FILTERS_OTHERS_H

#include <QWidget>
#include <define.h>
#include <pcl/filters/filter.h>
//#include <pcl/common/transforms.h>
//#include <pcl/filters/conditional_removal.h> // ROI
//include <pcl/filters/extract_indices.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <QColorDialog>

namespace Ui {
class Filters_Others;
}


class Filters_Others : public QWidget
{
    Q_OBJECT

public:
    explicit Filters_Others(QWidget *parent = 0);
    ~Filters_Others();

public slots:
    void setCurrentPC(PointCloudT::Ptr pc) { CurrentPC = pc; }


signals:
    void replace(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC, QString ElabName);

private slots:
    void on_pushButton_RemoveNan_clicked();

    void on_pushButton_Scale_clicked();

    void on_pushButton_SetColor_clicked();

private:
    Ui::Filters_Others *ui;
    PointCloudT::Ptr CurrentPC;
    QRgb lastColor;
};

#endif // Filters_Others_H

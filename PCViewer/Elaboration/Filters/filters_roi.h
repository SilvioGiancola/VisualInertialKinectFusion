#ifndef FILTERS_ROI_H
#define FILTERS_ROI_H

#include <QWidget>
#include <define.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h> // ROI
#include <pcl/filters/extract_indices.h>

#include <pcl/kdtree/kdtree_flann.h>


namespace Ui {
class Filters_ROI;
}


class Filters_ROI : public QWidget
{
    Q_OBJECT

public:
    explicit Filters_ROI(QWidget *parent = 0);
    ~Filters_ROI();

public slots:
    void setCurrentPC(PointCloudT::Ptr pc) { CurrentPC = pc; updateMinMax(); }


signals:
    void replace(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC, QString ElabName);

private slots:
    void updateMinMax();
    void on_ROI_Start_Elab_clicked();


private:
    Ui::Filters_ROI *ui;
    PointCloudT::Ptr CurrentPC;
};

#endif // FILTERS_ROI_H

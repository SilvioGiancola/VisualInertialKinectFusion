#ifndef FILTERS_VOXEL_GRID_H
#define FILTERS_VOXEL_GRID_H

#include <QWidget>
#include <define.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
namespace Ui {
class Filters_Voxel_Grid;
}


class Filters_Voxel_Grid : public QWidget
{
    Q_OBJECT

public:
    explicit Filters_Voxel_Grid(QWidget *parent = 0);
    ~Filters_Voxel_Grid();

public slots:
    void setCurrentPC(PointCloudT::Ptr pc) { CurrentPC = pc; }

signals:
    void replace(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC, QString ElabName);

private slots:
    void on_VG_Sample_clicked();

private:
    Ui::Filters_Voxel_Grid *ui;
    PointCloudT::Ptr CurrentPC;
};

#endif // FILTERS_VOXEL_GRID_H

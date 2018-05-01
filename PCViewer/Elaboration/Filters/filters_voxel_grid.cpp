#include "filters_voxel_grid.h"
#include "ui_filters_voxel_grid.h"

Filters_Voxel_Grid::Filters_Voxel_Grid(QWidget *parent) :
    QWidget(parent), ui(new Ui::Filters_Voxel_Grid)
{
    ui->setupUi(this);
}

Filters_Voxel_Grid::~Filters_Voxel_Grid()
{
    delete ui;
}



void Filters_Voxel_Grid::on_VG_Sample_clicked()
{
    if (CurrentPC != NULL)
    {
        // Create input and ouptu PointCloudT
        PointCloudT::Ptr input(CurrentPC);
        PointCloudT::Ptr output(new PointCloudT);
        pcl::copyPointCloud(*input, *output);

        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud (input);
        sor.setLeafSize (ui->VG_LeafX->value(), ui->VG_LeafY->value(), ui->VG_LeafZ->value());
        sor.filter (*output);

        // update Model
        emit replace(input, output, QString("Voxel Grid"));

    }
    return;
}

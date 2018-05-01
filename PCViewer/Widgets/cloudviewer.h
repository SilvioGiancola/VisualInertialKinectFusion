#ifndef CLOUDVIEWER_H
#define CLOUDVIEWER_H

#include <QWidget>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>
#include <QVTKWidget2.h>

#include <define.h>

namespace Ui {
class CloudViewer;
}

class CloudViewer : public QWidget
{
    Q_OBJECT

public:
    explicit CloudViewer(QWidget *parent = 0);
    ~CloudViewer();


public slots:
    void ShowGround(bool checked);
    void ShowRefSystem(bool checked);
    void ShowCoordinateSystem(double size, Eigen::Affine3f aff, std::string id);

private:
    Ui::CloudViewer *ui;
    pcl::visualization::PCLVisualizer::Ptr viewer;

    double back_col = 0.5;

    void pointSelected(const pcl::visualization::PointPickingEvent &event, void* );
    void areaSelected(const pcl::visualization::AreaPickingEvent &event, void* );
    void keyBoardPressed(const pcl::visualization::KeyboardEvent &event, void* );


};

#endif // CLOUDVIEWER_H

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <QModelIndex>
#include <QStandardItemModel>





#include <pcl/io/pcd_io.h> // for opening & saving .pcd



//per Kinect
#include <QToolBox>
#include <PCViewer/kinecttab.h>

// Per elab UI
#include <Elaboration/Filters/filters_mls.h>
#include <Elaboration/Filters/filters_outliers.h>
#include <Elaboration/Filters/filters_roi.h>
#include <Elaboration/Filters/filters_voxel_grid.h>
#include <Elaboration/Filters/filters_others.h>
#include <Elaboration/features_normal.h>
#include <Elaboration/features_keypoints.h>
#include <Elaboration/transformation.h>
#include <Elaboration/registration_keypoints.h>
#include <Elaboration/registration_icp.h>
#include <Elaboration/registration_ransac.h>
#include <Elaboration/registration_LUM.h>
#include <Elaboration/elaboration_lf_rec.h>
#include <Elaboration/elaboration_papazov.h>
#include <Elaboration/Calibration/calibration_adafruit.h>


#include <QtGui/QCloseEvent>
#include <PCViewer/modelpointcloudlist.h>

#include <boost/signals2.hpp>
#include <boost/bind.hpp>

#include <QUndoStack>
#include <QUndoView>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();


    // pcl 3D Viewer
    pcl::visualization::PCLVisualizer::Ptr viewer;
   // boost::shared_ptr <pcl::visualization::PCLPlotter> plotter;
    float back_col;

    // tree view
    ModelPointCloudList *PointCloudListModel;

    // Kinect Info list
    QList<KinectTab*> FreenectTab;

    void addQWidgetonDock(QWidget* wid, QString name);

private slots:

    // Visu
    void PCselectionChanged(const QModelIndex &index, const QModelIndex &index2);

    // open close save
    void on_actionLoad_PointCloud_triggered();
    void on_actionSave_PointCloud_triggered();
    void on_actionRemove_PointCloud_triggered();
    void on_actionCopy_PointCloud_triggered();
    void on_actionRemoveAll_triggered();
    void on_actionHideAll_triggered();
    void on_actionShowAll_triggered();
    void on_actionMerge_All_triggered();


    void on_actionShowRefSystem_triggered(bool checked);
    void on_actionShowGround_triggered(bool checked);
    void on_actionShowSelectedPointCloud_triggered();





    // IO UI
    void pointSelected(const pcl::visualization::PointPickingEvent &event, void* );
    void areaSelected(const pcl::visualization::AreaPickingEvent &event, void* );
    void keyBoardPressed(const pcl::visualization::KeyboardEvent &event, void* );







// Kinect
    void on_actionSyncGrab_triggered();
    void on_actionSyncOpen_triggered();
    void on_actionSyncClose_triggered();
 //   bool KinectRegistration(int CurrentIndex, int ReferenceIndex);










    void on_actionSaveAll_triggered();



    // Undo Stack
    void on_spinBox_UndoLimit_editingFinished();
    void on_pushButton_ClearUndoStack_clicked();


    void on_actionTestPCLPlotter_triggered();


    void on_actionExportPoses_triggered();

protected:
    void closeEvent ( QCloseEvent * event );

private:
    Ui::MainWindow *ui;
    QString DefaultDir;

};
#endif // MAINWINDOW_H

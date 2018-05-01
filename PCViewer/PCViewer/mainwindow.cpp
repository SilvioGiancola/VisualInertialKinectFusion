#if QT_VERSION >= 0x050000
#include <QApplication>
#else
#include <QtGui>
#endif
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QColorDialog>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <QFileDialog>
#include <math.h>
#include <QMessageBox>
#include <QStandardItemModel>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/search/kdtree.h>
#include <vector>
#include <pcl/search/search.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/range_image/range_image.h>
#include <boost/thread/thread.hpp>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <QDialogButtonBox>
#include <QInputDialog>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <QVector3D>
#include <QCheckBox>
#include <pcl/visualization/point_picking_event.h>


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


//#include <QtUiTools>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QRadioButton>
#include <Widgets/transformationmatrixspinbox.h>
#include <QMetaObject>
#include <QMetaMethod>
using namespace std;

/*********************
 *  User Interface
 *********************/
// Constructor
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    // Set the user interface from Qt Designer
    ui->setupUi(this);
 //   ui->centralWidget->hide();

   // tabifyDockWidget(ui->dockWidget_PointCloudList, ui->dockWidget_UndoStack);
 //   ui->dockWidget_PointCloudList->raise();




    // Create 3D Viewer
    viewer.reset(new pcl::visualization::PCLVisualizer("Viewer",false));
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    on_actionShowGround_triggered(true);
    on_actionShowRefSystem_triggered(true);
    ui->qvtkWidget->update ();

    viewer->setCameraPosition(-5,1,1, // mi posiziono dietro ad un Kinect
                              0.5,0.5,0.5, // guardo un punto centrale
                              0,0,1);   // orientato con la y verso l'alto
    viewer->setCameraClipDistances(-10,10);
    back_col = 0.5;
    viewer->setBackgroundColor (back_col, back_col, back_col);
    ui->qvtkWidget->update ();




    // point selected (shift + left click) info
    viewer->registerPointPickingCallback(&MainWindow::pointSelected, *this);    // Register the callback from Point Picking event to the function
    viewer->registerAreaPickingCallback(&MainWindow::areaSelected, *this);      // Register the callback from Area Picking event to the function
    viewer->registerKeyboardCallback(&MainWindow::keyBoardPressed, *this);      // Register the callback from Keyborad event to the function
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(),ui->qvtkWidget->GetRenderWindow());
    viewer->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);



    // Create Plotter
    /*   plotter.reset(new pcl::visualization::PCLPlotter("Plotter"));
    ui->Plotter_Widget->SetRenderWindow(plotter->getRenderWindow());
    ui->Plotter_Widget->update ();
    plotter->setViewInteractor(ui->Plotter_Widget->GetInteractor());//ui->Plotter_Widget->GetRenderWindow());
    //plotter->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);

    //  plotter->setupInteractor(ui->Plotter_Widget->GetInteractor(),ui->Plotter_Widget->GetRenderWindow());

*/

    // Create tree view
    PointCloudListModel = new ModelPointCloudList();
    PointCloudListModel->setViewer(viewer);
    ui->CloudListView->setModel(PointCloudListModel);

    connect(PointCloudListModel, SIGNAL(updateViewer()), ui->qvtkWidget, SLOT(update()));
    connect(ui->CloudListView->selectionModel(), SIGNAL(currentChanged(QModelIndex,QModelIndex)), this, SLOT(PCselectionChanged(QModelIndex,QModelIndex)));







    // Set all in tab
    setTabPosition(Qt::RightDockWidgetArea, QTabWidget::East);



    // Filters
    addQWidgetonDock(new Filters_ROI(), QString("Filters/ROI"));
    addQWidgetonDock(new Filters_MLS(), QString("Filters/MLS"));
    addQWidgetonDock(new Filters_Outliers(), QString("Filters/Outliers"));
    addQWidgetonDock(new Filters_Voxel_Grid(), QString("Filters/Voxel Grid"));
    addQWidgetonDock(new Filters_Others(), QString("Filters/Others"));

    addQWidgetonDock(new Transformation(), QString("Transformation"));

    // Features
    addQWidgetonDock(new Features_Normal(), QString("Features/Normal"));
    addQWidgetonDock(new Features_Keypoints(), QString("Features/Keypoints"));



    // REGISTRATION
    Registration_ICP *icp = new Registration_ICP();
    addQWidgetonDock(icp, QString("Registration/ICP"));
    icp->setViewer(viewer);

    Registration_RANSAC *ransac = new Registration_RANSAC();
    addQWidgetonDock(ransac, QString("Registration/RANSAC"));
    ransac->setViewer(viewer);

    addQWidgetonDock(new Registration_LUM(), QString("Registration/LUM"));


    /*   Registration_Keypoints *reg = new Registration_Keypoints();
    addQWidgetonDock(reg, QString("Registration/KeyPoints"));
    reg->setViewer(viewer);*/



    // RECOGNITION
    //   addQWidgetonDock(new Elaboration_LF_Rec(), QString("Recognition/LF_REC"));

    //   Elaboration_Papazov *pap = new Elaboration_Papazov();
    //   addQWidgetonDock(pap, QString("Recognition/Papazov"));
    //  pap->setViewer(viewer);


    // CALIBRATION
    addQWidgetonDock(new Calibration_Adafruit(), QString("Calibration/Adafruit"));






    // Action ToolBar View
    ui->dockWidget_PointCloudList->toggleViewAction()->setText(QString("Hide/Show Point Cloud List"));
    ui->menuWindow->addAction(ui->dockWidget_PointCloudList->toggleViewAction());

    ui->dockWidget_UndoStack->toggleViewAction()->setText(QString("Hide/Show Undo Stack"));
    ui->menuWindow->addAction(ui->dockWidget_UndoStack->toggleViewAction());

    /* ui->dockWidget_Elaboration->toggleViewAction()->setText(QString("Hide/Show Elaboration Widget"));
    ui->menuWindow->addAction(ui->dockWidget_Elaboration->toggleViewAction());*/
    //  ui->dockWidget_Elaboration->hide();

    ui->dockWidget_Kinect->toggleViewAction()->setText(QString("Hide/Show Kinect Tab"));
    ui->menuWindow->addAction(ui->dockWidget_Kinect->toggleViewAction());
    ui->dockWidget_Kinect->hide();


    ui->menuWindow->addSeparator();

    ui->toolBar->toggleViewAction()->setText(QString("Hide/Show Tool Bar"));
    ui->menuWindow->addAction(ui->toolBar->toggleViewAction());





    // Undo Stack Widget
    ui->UndoStackView->setStack(PointCloudListModel->undoStack);

    QAction* undoAction = ui->UndoStackView->stack()->createUndoAction(this, tr("&Undo"));
    undoAction->setShortcut(QKeySequence::Undo);

    QAction* redoAction = ui->UndoStackView->stack()->createRedoAction(this, tr("&Redo"));
    redoAction->setShortcut(QKeySequence::Redo);

    ui->menuEdit->addAction(undoAction);
    ui->menuEdit->addAction(redoAction);







    // Set the default path for my point cloud
    DefaultDir = QString("%1/PointClouds").arg(QDir::homePath());



    // Find the number of kinect connected
    libfreenect2::Freenect2 freenect2;
    int nbKinect = freenect2.enumerateDevices();

    // if kinect is connected
    if (nbKinect > 0)
    {
        // for every Kinect connected
        for (int i = 0; i < nbKinect; i++)
        {
            KinectTab* PageKinect = new KinectTab(freenect2.getDeviceSerialNumber(i));

            ui->KintoolBox->insertItem(i, PageKinect, QString("Kinect %1").arg(i));

            connect(PageKinect, SIGNAL(PCgrabbed(PointCloudT::Ptr)), PointCloudListModel, SLOT(addPointCloud(PointCloudT::Ptr)));
            FreenectTab.push_back(PageKinect);

        }
        ui->KintoolBox->removeItem(ui->KintoolBox->count()-1);
        delete ui->EmptyKinect;
        ui->dockWidget_Kinect->show();
        ui->dockWidget_Kinect->raise();
    }

}



// Destructor
MainWindow::~MainWindow()
{
    for (int i = 0; i <FreenectTab.size(); i++)
        FreenectTab.at(i)->on_pushButton_Close_clicked();

    delete PointCloudListModel;
    delete ui;
}

// Close dependency
void MainWindow::closeEvent ( QCloseEvent * event )
{
    if (QMessageBox::question(this, "Exit", "Are you sure you want to quit?", QMessageBox::No|QMessageBox::Yes) == QMessageBox::Yes)
        event->accept();
    else
        event->ignore();

    return;
}



/*********************
 *  Functions
 *********************/
void MainWindow::addQWidgetonDock(QWidget *wid, QString name)
{
    QDockWidget * DockWidget = new QDockWidget(name.section("/",-1,-1));
    DockWidget->setWidget(wid);
    this->addDockWidget(Qt::RightDockWidgetArea, DockWidget, Qt::Vertical);

    tabifyDockWidget(ui->dockWidget_Kinect, DockWidget);

    if (name.section("/",0,0) == ui->menuFilters->title())
        ui->menuFilters->addAction(DockWidget->toggleViewAction());

    else if (name.section("/",0,0) == ui->menuRegistration->title())
        ui->menuRegistration->addAction(DockWidget->toggleViewAction());

    else
        ui->menuElaborattion->addAction(DockWidget->toggleViewAction());



    const QMetaObject* metaObject = wid->metaObject();


    for(int i = metaObject->methodOffset(); i < metaObject->methodCount(); ++i)
    {
#if QT_VERSION >= 0x050000
        QString name = QString::fromLatin1(metaObject->method(i).name());
#else
        QString name = QString::fromLatin1(metaObject->method(i).signature()).section("(",0,0);
#endif



        if (name == "addPC")
            connect(wid, SIGNAL(addPC(PointCloudT::Ptr)), PointCloudListModel, SLOT(addPointCloud(PointCloudT::Ptr)));

        else if (name == "replace")
            connect(wid, SIGNAL(replace(PointCloudT::Ptr, PointCloudT::Ptr, QString)), PointCloudListModel, SLOT(replacePointCloud(PointCloudT::Ptr, PointCloudT::Ptr, QString)));

        else if (name == "setCurrentPC")
            connect(PointCloudListModel, SIGNAL(setCurrentPC(PointCloudT::Ptr)), wid, SLOT(setCurrentPC(PointCloudT::Ptr)));

        else if (name == "setCurrentPCList")
            connect(PointCloudListModel, SIGNAL(setCurrentPCList(QList<PointCloudT::Ptr>)), wid, SLOT(setCurrentPCList(QList<PointCloudT::Ptr>)));

    }


}

void MainWindow::PCselectionChanged(const QModelIndex &myIndex, const QModelIndex &index2)
{
    if (!myIndex.isValid())
    {
        PointCloudListModel->selectedPC.reset();
        return;
    }

    if (!index2.isValid())
    {

        return;
    }

    QModelIndex realIndex;

    if (myIndex.parent() == PointCloudListModel->invisibleRootItem()->index())
        realIndex = myIndex;

    else if (myIndex.parent().parent() == PointCloudListModel->invisibleRootItem()->index())
        realIndex = myIndex.parent();

    PointCloudListModel->selectedPC = PointCloudListModel->getPC(realIndex.row());

  //  viewer->setPointCloudSelected(1, PointCloudListModel->getPCname(myIndex.row()));
   // viewer->setPointCloudSelected(0, PointCloudListModel->getPCname(index2.row()));

    // unselect all Point Cloud
    for (int i = 0; i < PointCloudListModel->getPCList().size(); i++)
        if (viewer->contains(PointCloudListModel->getPCname(i)))
            viewer->setPointCloudSelected(0, PointCloudListModel->getPCname(i));

    // select 1 Point Cloud
    if (ui->actionShowSelectedPointCloud->isChecked())
        if (viewer->contains(PointCloudListModel->selectedPC->header.frame_id))
            viewer->setPointCloudSelected(1,PointCloudListModel->selectedPC->header.frame_id);


    ui->qvtkWidget->update();

    PointCloudListModel->emitCurrentPC();
    //    emit PointCloudListModel->setCurrentPC(PointCloudListModel->selectedPC);

    return;

}

void MainWindow::on_actionShowGround_triggered(bool checked)
{
    int XMin = 0; int XMax = 5;
    int YMin = -2; int YMax = 2;

    if (checked)
    {
        pcl::ModelCoefficients line_coeff;
        line_coeff.values.resize (6);    // We need 6 values


        line_coeff.values[2] = 0;
        line_coeff.values[5] = 0;

        // Add X Lines
        for (int i = XMin; i <= XMax; i++)
        {
            line_coeff.values[0] = i;
            line_coeff.values[1] = YMin;

            line_coeff.values[3] = 0;
            line_coeff.values[4] = YMax-YMin;
            viewer->addLine (line_coeff, QString("line X%1").arg(i).toStdString());

        }

        // Add Y Lines
        for (int i = YMin; i <= YMax; i++)
        {
            line_coeff.values[0] = XMin;
            line_coeff.values[1] = i;

            line_coeff.values[3] = XMax-XMin;
            line_coeff.values[4] = 0;
            viewer->addLine (line_coeff, QString("line Y%1").arg(i).toStdString());

        }

        viewer->setCameraClipDistances(-10,10);

        ui->statusBar->showMessage(QString("Ground is shown"));
    }
    else
    {
        // Remove X Lines
        for (int i = XMin; i <= XMax; i++)
            viewer->removeShape(QString("line X%1").arg(i).toStdString());

        // Remove Y Lines
        for (int i = YMin; i <= YMax; i++)
            viewer->removeShape(QString("line Y%1").arg(i).toStdString());

        ui->statusBar->showMessage(QString("Ground has been removed"));
    }


    ui->qvtkWidget->update();


}

void MainWindow::on_actionShowRefSystem_triggered(bool checked)
{
    // Show the Main reference System
    if (checked)
        viewer->addCoordinateSystem(1.0, "Main Reference System");

    // Remove the Main reference System
    else
        viewer->removeCoordinateSystem("Main Reference System");


    ui->statusBar->showMessage(QString("Ref system is now: %1").arg(checked));
    ui->qvtkWidget->update();

    return;

}

void MainWindow::on_actionShowSelectedPointCloud_triggered()
{
    // unselect all Point Cloud
    for (int i = 0; i < PointCloudListModel->getPCList().size(); i++)
        if (viewer->contains(PointCloudListModel->getPCname(i)))
            viewer->setPointCloudSelected(0, PointCloudListModel->getPCname(i));

    // select 1 Point Cloud
    if (ui->actionShowSelectedPointCloud->isChecked())
        if (viewer->contains(PointCloudListModel->selectedPC->header.frame_id))
            viewer->setPointCloudSelected(1,PointCloudListModel->selectedPC->header.frame_id);

    ui->qvtkWidget->update();

}


/*********************
 *  I/O Operations
 *********************/

// Load Point Cloud Button
void MainWindow::on_actionLoad_PointCloud_triggered()
{
    QStringList Files = QFileDialog::getOpenFileNames(this, tr("Open file..."),DefaultDir);//, tr("All Files (*.pcd;*.ply)"));

    foreach (QString file, Files)
    {
        PointCloudT::Ptr cloud (new PointCloudT);

        if (file.endsWith(".pcd"))
            pcl::io::loadPCDFile(file.toStdString(), *cloud);

        else if (file.endsWith(".ply"))
            pcl::io::loadPLYFile(file.toStdString(), *cloud);

        else
            return;

        cloud->header.frame_id = file.toStdString();

        ui->UndoStackView->stack()->push(new LoadCommand(cloud, PointCloudListModel));
    }
    ui->statusBar->showMessage(QString("%1 Point Clouds has been opened successfully").arg(Files.size()));

    return;
}

// Remove a point cloud
void MainWindow::on_actionRemove_PointCloud_triggered()
{
    std::string id = PointCloudListModel->selectedPC->header.frame_id;

    ui->UndoStackView->stack()->push(new RemoveCommand(PointCloudListModel->selectedPC, PointCloudListModel));

    ui->statusBar->showMessage(QString("%1 has been removed successfully").arg(id.c_str()));

    ui->qvtkWidget->update();
    return;
}

// Save current selected cloud to file .pcd
void MainWindow::on_actionSave_PointCloud_triggered()
{
    if (PointCloudListModel->saveCurrentPointCloud(ui->actionBinaryMode->isChecked(), ui->actionSaveNormal->isChecked()) == 0)
        ui->statusBar->showMessage(QString("%1 has been saved successfully").arg(PointCloudListModel->selectedPC->header.frame_id.c_str()));

    else
        ui->statusBar->showMessage(QString("ERROR in saving %1 point cloud").arg(PointCloudListModel->selectedPC->header.frame_id.c_str()));

    ui->qvtkWidget->update();
    return;
}

// Copy a point cloud
void MainWindow::on_actionCopy_PointCloud_triggered()
{
    if (PointCloudListModel->selectedPC->size() > 0)
    {

        ui->UndoStackView->stack()->push(new CopyCommand(PointCloudListModel->selectedPC, PointCloudListModel));

        ui->statusBar->showMessage(QString("%1 Has successfully been copied").arg(PointCloudListModel->selectedPC->header.frame_id.c_str()));

        ui->qvtkWidget->update();
    }
    else
    {
        ui->statusBar->showMessage(QString("No point has been found in selected Point Cloud %1").arg(PointCloudListModel->selectedPC->header.frame_id.c_str()));
    }
    ui->qvtkWidget->update();

    return;
}






// Save all point cloud
void MainWindow::on_actionSaveAll_triggered()
{
    PointCloudListModel->saveAllPointClouds(ui->actionBinaryMode->isChecked(), ui->actionSaveNormal->isChecked());

    ui->qvtkWidget->update();
    return;
}

// Remove all point cloud
void MainWindow::on_actionRemoveAll_triggered()
{
    PointCloudListModel->removeAllPointClouds();

    on_actionShowGround_triggered(true);
    on_actionShowRefSystem_triggered(true);

    ui->qvtkWidget->update();
    return;
}

// hide All Point Cloud
void MainWindow::on_actionHideAll_triggered()
{
    PointCloudListModel->hideAllPointClouds();

    on_actionShowGround_triggered(true);
    on_actionShowRefSystem_triggered(true);

    ui->qvtkWidget->update();
    return;
}

// Show All Point Cloud
void MainWindow::on_actionShowAll_triggered()
{
    PointCloudListModel->showAllPointClouds();

    ui->qvtkWidget->update();
    return;
}


void MainWindow::on_actionMerge_All_triggered()
{
    PointCloudListModel->mergeAllPointClouds();

    ui->qvtkWidget->update();
    return;
}



#include <pcl/common/geometry.h>
// Viewer Select a point
void MainWindow::pointSelected(const pcl::visualization::PointPickingEvent& event,void*)
{
    if (ui->actionPointSelection->isChecked())
    {
        int idx = event.getPointIndex ();
        if (idx == -1)
            return;

        pcl::search::KdTree<PointT> search;

        float global_distance = 1000.0f;
        int ind = idx;

        PointT coloredpoint;
        PointT picked_pt;
        event.getPoint (picked_pt.x, picked_pt.y, picked_pt.z);

        for (int i = 0; i < PointCloudListModel->getPCList().size(); i++)
        {
            search.setInputCloud (PointCloudListModel->getPC(i));
            // Return the correct index in the cloud instead of the index on the screen
            std::vector<int> indices (1);
            std::vector<float> distances (1);

            // Because VTK/OpenGL stores data without NaN, we lose the 1-1 correspondence, so we must search for the real point
            search.nearestKSearch (picked_pt, 1, indices, distances);

            if (global_distance > (float)distances[0])
            {
                global_distance = (float)distances[0];
                ind = indices[0];
                coloredpoint= PointCloudListModel->getPC(i)->at(indices[0]);
            }
        }

        QString Message = QString("Selected Point: index=%1; XYZ=(%2;%3;%4); RGB=(%5;%6;%7); curvature=%8")
                .arg(ind)
                .arg(coloredpoint.x).arg(coloredpoint.y).arg(coloredpoint.z)
                .arg((int)coloredpoint.r).arg((int)coloredpoint.g).arg((int)coloredpoint.b)
                .arg(coloredpoint.curvature);

        ui->statusBar->showMessage(Message);
    }
    return;
}

// Viewer Select a point
void MainWindow::areaSelected(const pcl::visualization::AreaPickingEvent& event,void*)
{
    std::vector<int> indices;
    event.getPointsIndices(indices);
    QString Message = QString("You have selected %1 Points").arg(indices.size());//.arg(ind).arg(global_distance);

    ui->statusBar->showMessage(Message);
    // std::cout << indices.size() << std::endl;
    return;
}

// Viewer keyboard event
void MainWindow::keyBoardPressed(const pcl::visualization::KeyboardEvent& event,void*)
{
    if (event.keyDown())
    {
        if (event.getKeySym () == "k" )
        {
            if (back_col < 1.0)
                back_col = back_col + 0.05;

            viewer->setBackgroundColor (back_col, back_col, back_col);
            ui->qvtkWidget->update();
        }
        else if (event.getKeySym () == "l" )
        {
            if (back_col > 0.0)
                back_col = back_col - 0.05;

            viewer->setBackgroundColor (back_col, back_col, back_col);
            ui->qvtkWidget->update();
        }
    }

    return;
}





/* KINECT*/

void MainWindow::on_actionSyncGrab_triggered()
{
    for (int i = 0; i <FreenectTab.size(); i++)
        FreenectTab.at(i)->on_pushButton_Grab_clicked();

    return;
}

void MainWindow::on_actionSyncOpen_triggered()
{
    for (int i = 0; i <FreenectTab.size(); i++)
        FreenectTab.at(i)->on_pushButton_Open_clicked();

    return;
}

void MainWindow::on_actionSyncClose_triggered()
{
    for (int i = 0; i <FreenectTab.size(); i++)
        FreenectTab.at(i)->on_pushButton_Close_clicked();

    return;
}





void MainWindow::on_spinBox_UndoLimit_editingFinished()
{
    if (ui->UndoStackView->stack()->count() > 0)
        if (QMessageBox::question(this, "Undo Stack Limit", "Need to clear the Stack, are you sure?", QMessageBox::No|QMessageBox::Yes) != QMessageBox::Yes)
            return;

    ui->UndoStackView->stack()->clear();
    ui->UndoStackView->stack()->setUndoLimit(ui->spinBox_UndoLimit->value());
    return;
}

void MainWindow::on_pushButton_ClearUndoStack_clicked()
{
    ui->UndoStackView->stack()->clear();
    return;
}





/*
 * MAIN FUNCTION
 */

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setWindowIcon(QIcon(QString(":/Images/Show.png")));

    MainWindow w;
    w.showMaximized();

    return a.exec();
}


void MainWindow::on_actionTestPCLPlotter_triggered()
{
    /*   //defining a plotter
    //  pcl::visualization::PCLPlotter plotter;// = new pcl::visualization::PCLPlotter ();

    //defining the polynomial function, y = x^2. Index of x^2 is 1, rest is 0
    vector<double> func1;
    func1.push_back(0);
    func1.push_back(1);
    func1.push_back(2);
    func1.push_back(3);

    //defining the polynomial function, y = x^2. Index of x^2 is 1, rest is 0
    vector<double> func2;
    func2.push_back(3);
    func2.push_back(2);
    func2.push_back(1);
    func2.push_back(2);


    //adding the polynomial func1 to the plotter with [-10, 10] as the range in X axis and "y = x^2" as title
    plotter->addPlotData (func1, func2,"line", vtkChart::LINE);
    //adding the polynomial func1 to the plotter with [-10, 10] as the range in X axis and "y = x^2" as title
    plotter->addPlotData (func1, func2,"point", vtkChart::POINTS);

    //display the plot, DONE!
    plotter->plot ();
    ui->Plotter_Widget->update();
*/
}

void MainWindow::on_actionExportPoses_triggered()
{
    QString FilePoses = QFileDialog::getSaveFileName(this, tr("Save file..."),DefaultDir);//, tr("All Files (*.pcd;*.ply)"));


        PointCloudListModel->exportPoses(FilePoses);

}

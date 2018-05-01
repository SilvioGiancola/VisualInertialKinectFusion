#include "cloudviewer.h"
#include "ui_cloudviewer.h"

CloudViewer::CloudViewer(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CloudViewer)
{
    ui->setupUi(this);


    // Create 3D Viewer
    viewer.reset(new pcl::visualization::PCLVisualizer("Viewer",false));
    ui->myQVTKWidget->SetRenderWindow(viewer->getRenderWindow());
    ShowGround(true);
    ShowRefSystem(true);
    ui->myQVTKWidget->update ();

    viewer->setCameraPosition(-5,1,1, // mi posiziono dietro ad un Kinect
                              0.5,0.5,0.5, // guardo un punto centrale
                              0,0,1);   // orientato con la y verso l'alto
    viewer->setCameraClipDistances(-10,10);
    viewer->setBackgroundColor (back_col, back_col, back_col);
    ui->myQVTKWidget->update ();




    // point selected (shift + left click) info
    viewer->registerPointPickingCallback(&CloudViewer::pointSelected, *this);    // Register the callback from Point Picking event to the function
    viewer->registerAreaPickingCallback(&CloudViewer::areaSelected, *this);      // Register the callback from Area Picking event to the function
    viewer->registerKeyboardCallback(&CloudViewer::keyBoardPressed, *this);      // Register the callback from Keyborad event to the function
    viewer->setupInteractor(ui->myQVTKWidget->GetInteractor(),ui->myQVTKWidget->GetRenderWindow());
    viewer->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);


}

CloudViewer::~CloudViewer()
{
    delete ui;
}



#include <pcl/common/geometry.h>
// Viewer Select a point
void CloudViewer::pointSelected(const pcl::visualization::PointPickingEvent& event,void*)
{

    int idx = event.getPointIndex ();
    if (idx == -1)
        return;


    PointT picked_pt;
    event.getPoint (picked_pt.x, picked_pt.y, picked_pt.z);


    QString Message = QString("Selected Point: index=%1; XYZ=(%2;%3;%4)")
            .arg(idx).arg(picked_pt.x).arg(picked_pt.y).arg(picked_pt.z);

    std::cout << Message.toStdString() << std::endl;

    return;
}

// Viewer Select a point
void CloudViewer::areaSelected(const pcl::visualization::AreaPickingEvent& event,void*)
{
    std::vector<int> indices;
    event.getPointsIndices(indices);
    QString Message = QString("You have selected %1 Points").arg(indices.size());//.arg(ind).arg(global_distance);

    std::cout << Message.toStdString() << std::endl;
    return;
}

// Viewer keyboard event
void CloudViewer::keyBoardPressed(const pcl::visualization::KeyboardEvent& event,void*)
{
    if (event.keyDown())
    {
        if (event.getKeySym () == "k" )
        {
            if (back_col < 1.0)
                back_col = back_col + 0.05;

            viewer->setBackgroundColor (back_col, back_col, back_col);
            ui->myQVTKWidget->update();
        }
        else if (event.getKeySym () == "l" )
        {
            if (back_col > 0.0)
                back_col = back_col - 0.05;

            viewer->setBackgroundColor (back_col, back_col, back_col);
            ui->myQVTKWidget->update();
        }
    }

    return;
}





void CloudViewer::ShowGround(bool checked)
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

    }
    else
    {
        // Remove X Lines
        for (int i = XMin; i <= XMax; i++)
            viewer->removeShape(QString("line X%1").arg(i).toStdString());

        // Remove Y Lines
        for (int i = YMin; i <= YMax; i++)
            viewer->removeShape(QString("line Y%1").arg(i).toStdString());

    }


    ui->myQVTKWidget->update();


}

void CloudViewer::ShowRefSystem(bool checked)
{
    // Show the Main reference System
    if (checked)
        viewer->addCoordinateSystem(1.0, "Main Reference System");

    // Remove the Main reference System
    else
        viewer->removeCoordinateSystem("Main Reference System");


    ui->myQVTKWidget->update();

    return;

}





void CloudViewer::ShowCoordinateSystem(double size, Eigen::Affine3f aff, std::string id)
{
    if (viewer->contains(id))
        viewer->removeCoordinateSystem(id);
    viewer->addCoordinateSystem(size, aff, id);

    ui->myQVTKWidget->update();
}

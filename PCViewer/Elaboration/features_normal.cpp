#include "features_normal.h"
#include "ui_features_normal.h"

#include <pcl/filters/filter.h>
#include <QElapsedTimer>
Features_Normal::Features_Normal(QWidget *parent) :
    QWidget(parent), ui(new Ui::Features_Normal)
{
    ui->setupUi(this);
}

Features_Normal::~Features_Normal()
{
    delete ui;
}


void Features_Normal::on_pushButton_Compute_clicked()
{
  std::cout << "Computing normals ";

    if (CurrentPC != NULL)
    {

        PointCloudT::Ptr input(CurrentPC);
        PointCloudT::Ptr output(new PointCloudT);
        pcl::copyPointCloud(*input, *output);

        ComputeNormal(input, output);

        emit replace(input, output, QString("Normal"));
    }
    std::cout << "DONE"<<std::endl;


    return;

}



void Features_Normal::on_pushButton_Comput_All_Normals_clicked()
{
    std::cout << "Computing all normals...";

    QElapsedTimer *Timer = new QElapsedTimer();
    Timer->start();

    for(int i=0; i<CurrentPCList.size(); i++)
    {
        PointCloudT::Ptr input(CurrentPCList.at(i));
        PointCloudT::Ptr output(new PointCloudT);
        pcl::copyPointCloud(*input, *output);

        ComputeNormal(input, output);

        emit replace(input, output, QString("Normal"));

        std::cout << "DONE"<<std::endl;
        std::cout << "Compute all normal in: "  << Timer->elapsed() << " milliseconds" << std::endl;
    }


    return;

}



void Features_Normal::ComputeNormal(PointCloudT::Ptr input, PointCloudT::Ptr output)
{
    if (ui->radioButton_ImageIntegral->isChecked())
    {
        std::cout << "with Integral Image Estimation...";
        if (input->isOrganized())
        {
            pcl::IntegralImageNormalEstimation<PointT, PointT> ne;
            ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
            ne.setMaxDepthChangeFactor(ui->doubleSpinBox_MaxDepthChangeFactor->value());
            ne.setNormalSmoothingSize(ui->doubleSpinBox_NormalSmoothingSize->value());

            ne.setInputCloud(input);

            ne.compute(*output);
        }
    }
    else
    {
        std::cout << "with Normal Estimationn...";
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*input, *input, indices);

        pcl::NormalEstimation<PointT, PointT> normal_estimator;

        if (ui->radioButton_K_Search->isChecked())
            normal_estimator.setKSearch(ui->spinBox_K->value());

        else if (ui->radioButton_Radius_Search->isChecked())
            normal_estimator.setRadiusSearch (ui->doubleSpinBox_Radius->value());

        else
            return;

        pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);
        normal_estimator.setSearchMethod (tree);


        normal_estimator.setInputCloud (input);
        normal_estimator.compute (*output);
    }

}

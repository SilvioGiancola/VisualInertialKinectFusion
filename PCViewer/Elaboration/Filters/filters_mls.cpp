#include "filters_mls.h"
#include "ui_filters_mls.h"

Filters_MLS::Filters_MLS(QWidget *parent) :
    QWidget(parent), ui(new Ui::Filters_MLS)
{
    ui->setupUi(this);
}

Filters_MLS::~Filters_MLS()
{
    delete ui;
}


void Filters_MLS::on_MLS_process_clicked()
{
    pcl::ScopeTime t("MLS");

    if (CurrentPC != NULL)
    {
        // Create input and ouptu PointCloudT
        PointCloudT::Ptr input(CurrentPC);
        PointCloudT::Ptr output(new PointCloudT);
        pcl::copyPointCloud(*input, *output);

        MovingLeastSquare(input, output);

        // update Model
        emit replace(input, output, QString("MLS"));

    }
    return;

}

void Filters_MLS::on_MLS_processALL_clicked()
{
    pcl::ScopeTime t("MLS ALL");

    for(int i=0; i<CurrentPCList.size(); i++)
    {
        PointCloudT::Ptr input(CurrentPCList.at(i));
        PointCloudT::Ptr output(new PointCloudT);
        pcl::copyPointCloud(*input, *output);

        MovingLeastSquare(input, output);

        // Update Model
        emit replace(input, output, QString("MLS"));

    }
}


void Filters_MLS::MovingLeastSquare(PointCloudT::Ptr input, PointCloudT::Ptr output)
{


    // Create MLS
    pcl::MovingLeastSquares<pcl::PointXYZRGB, PointT> mls;

    // Set the computation of normals
    mls.setComputeNormals (ui->MLS_ComputeNormals->isChecked());


    // define Tree Search
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    mls.setSearchMethod (tree);

    // set the Point Cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::copyPointCloud(*input, *points);
    mls.setInputCloud (points);
//
    // set search
    mls.setSearchRadius (ui->MLS_SearchRadius->value()); // 0.05

    // Other parameters
    mls.setPolynomialFit (true);


    // Proceed
    mls.process (*output);

}





void Filters_MLS::on_BF_process_clicked()
{
    pcl::ScopeTime t("Bilateral Filter");

    if (CurrentPC != NULL)
    {
        // Create input and ouptu PointCloudT
        PointCloudT::Ptr input(CurrentPC);
        PointCloudT::Ptr output(new PointCloudT);
        pcl::copyPointCloud(*input, *output);

        BilateralFilter(input, output);

        // update Model
        emit replace(input, output, QString("BF"));

    }
    return;

}

void Filters_MLS::on_BF_processALL_clicked()
{

}


void Filters_MLS::BilateralFilter(PointCloudT::Ptr input, PointCloudT::Ptr output)
{


    // Create MLS
    pcl::FastBilateralFilter<pcl::PointXYZRGB> bf;

    // Set Sigma
    bf.setSigmaR(ui->BF_SigmaR->value());
    bf.setSigmaS(ui->BF_SigmaS->value());




  /*/ì  // define Tree Search
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    mls.setSearchMethod (tree);
*/
    // set the Point Cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::copyPointCloud(*input, *points);
    bf.setInputCloud (points);

    // set search
   // mls.setSearchRadius (ui->MLS_SearchRadius->value()); // 0.05

    // Other parameters
   // mls.setPolynomialFit (true);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_out (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Proceed
    bf.applyFilter(*points_out);

    pcl::copyPointCloud(*points_out, *output);


}


#include "filters_outliers.h"
#include "ui_filters_outliers.h"

Filters_Outliers::Filters_Outliers(QWidget *parent) :
    QWidget(parent), ui(new Ui::Filters_Outliers)
{
    ui->setupUi(this);
}

Filters_Outliers::~Filters_Outliers()
{
    delete ui;
}


void Filters_Outliers::on_Remove_Radius_Outliers_clicked()
{
    pcl::ScopeTime t("Remove Radius Outliers");

    if (CurrentPC != NULL)
    {
        // Create input and ouptu PointCloudT
        PointCloudT::Ptr input(CurrentPC);
        PointCloudT::Ptr output(new PointCloudT);
        pcl::copyPointCloud(*input, *output);

        RemoveRadiusOutliers(input, output);

        // update Model
        emit replace(input, output, QString("Rad Outliers"));
    }
    return;

}

void Filters_Outliers::on_Remove_Radius_Outliers_ALL_clicked()
{
    pcl::ScopeTime t("Remove Radius Outliers ALL");

    for(int i=0; i<CurrentPCList.size(); i++)
    {
        PointCloudT::Ptr input(CurrentPCList.at(i));
        PointCloudT::Ptr output(new PointCloudT);
        pcl::copyPointCloud(*input, *output);

        RemoveRadiusOutliers(input, output);

        // Update Model
        emit replace(input, output, QString("Rad Outliers"));

    }
}


void Filters_Outliers::RemoveRadiusOutliers(PointCloudT::Ptr input, PointCloudT::Ptr output)
{

    pcl::RadiusOutlierRemoval<PointT> outrem;
    // build the filter
    outrem.setInputCloud(input);
    outrem.setRadiusSearch(ui->ROR_RadiusSearch->value());
    outrem.setMinNeighborsInRadius (ui->ROR_MinNeighborsInRadius->value());
    outrem.setNegative(ui->ROR_Inverse->isChecked());
    outrem.setKeepOrganized(input->isOrganized());
    // apply filter
    outrem.filter (*output);
}






void Filters_Outliers::on_Remove_Statistical_Outliers_clicked()
{
    pcl::ScopeTime t("Remove Statistical Outliers");

    if (CurrentPC != NULL)
    {
        // Create input and ouptu PointCloudT
        PointCloudT::Ptr input(CurrentPC);
        PointCloudT::Ptr output(new PointCloudT);
        pcl::copyPointCloud(*input, *output);

        RemoveStatisticalOutliers(input, output);

        // update Model
        emit replace(input, output, QString("Stat Outliers"));
    }
    return;

}

void Filters_Outliers::on_Remove_Statistical_Outliers_ALL_clicked()
{
    pcl::ScopeTime t("Remove Statistical Outliers ALL");

    for(int i=0; i<CurrentPCList.size(); i++)
    {
        PointCloudT::Ptr input(CurrentPCList.at(i));
        PointCloudT::Ptr output(new PointCloudT);
        pcl::copyPointCloud(*input, *output);

        RemoveStatisticalOutliers(input, output);

        // Update Model
        emit replace(input, output, QString("Rad Outliers"));

    }
}


void Filters_Outliers::RemoveStatisticalOutliers(PointCloudT::Ptr input, PointCloudT::Ptr output)
{

    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (input);
    sor.setMeanK (ui->SOR_MeanK->value());//    number of neighbors to analyze
    sor.setStddevMulThresh (ui->SOR_StddevMulThresh->value()); //    standard deviation multiplier (1-2 or 3 sigma)
    sor.setNegative(ui->SOR_Inverse->isChecked());
    sor.setKeepOrganized(input->isOrganized());
    // aply filter
    sor.filter (*output);
}

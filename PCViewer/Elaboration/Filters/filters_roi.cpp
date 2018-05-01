#include "filters_roi.h"
#include "ui_filters_roi.h"

Filters_ROI::Filters_ROI(QWidget *parent) :
    QWidget(parent), ui(new Ui::Filters_ROI)
{
    ui->setupUi(this);
}

Filters_ROI::~Filters_ROI()
{
    delete ui;
}



void Filters_ROI::updateMinMax()
{
    if (CurrentPC == NULL)
        return;

    PointCloudT::Ptr cloud (new PointCloudT());

    if (ui->radioButton_Local->isChecked())
        pcl::copyPointCloud(*CurrentPC, *cloud);

    else if (ui->radioButton_Global->isChecked())
        pcl::transformPointCloud(*CurrentPC, *cloud,  CurrentPC->sensor_origin_.head(3), CurrentPC->sensor_orientation_);
    else
        return;


    PointT max;
    max.x = max.y = max.z = -FLT_MAX;
    max.r = max.g = max.b = 0;
    PointT min;
    min.x = min.y = min.z = FLT_MAX;
    min.r = min.g = min.b = 255;

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        // Check if the point is invalid
        if (!pcl_isfinite (cloud->points[i].x) || !pcl_isfinite (cloud->points[i].y) || !pcl_isfinite (cloud->points[i].z))
            continue;

        max.x = fmax(max.x, cloud->points[i].x);
        max.y = fmax(max.y, cloud->points[i].y);
        max.z = fmax(max.z, cloud->points[i].z);
        max.r = fmax(max.r, cloud->points[i].r);
        max.g = fmax(max.g, cloud->points[i].g);
        max.b = fmax(max.b, cloud->points[i].b);

        min.x = fmin(min.x, cloud->points[i].x);
        min.y = fmin(min.y, cloud->points[i].y);
        min.z = fmin(min.z, cloud->points[i].z);
        min.r = fmin(min.r, cloud->points[i].r);
        min.g = fmin(min.g, cloud->points[i].g);
        min.b = fmin(min.b, cloud->points[i].b);

    }


    ui->ROI_XMin->setValue(min.x);
    ui->ROI_XMax->setValue(max.x);
    ui->ROI_YMin->setValue(min.y);
    ui->ROI_YMax->setValue(max.y);
    ui->ROI_ZMin->setValue(min.z);
    ui->ROI_ZMax->setValue(max.z);
    ui->ROI_RMin->setValue(min.r);
    ui->ROI_RMax->setValue(max.r);
    ui->ROI_GMin->setValue(min.g);
    ui->ROI_GMax->setValue(max.g);
    ui->ROI_BMin->setValue(min.b);
    ui->ROI_BMax->setValue(max.b);



    return;
}

void Filters_ROI::on_ROI_Start_Elab_clicked()
{
    if (CurrentPC == NULL)
        return;

    PointCloudT::Ptr input(CurrentPC);
    PointCloudT::Ptr output(new PointCloudT);



    if (ui->radioButton_Local->isChecked())
        pcl::copyPointCloud(*input, *output);

    else if (ui->radioButton_Global->isChecked())
        pcl::transformPointCloud(*input, *output, CurrentPC->sensor_origin_.head(3), CurrentPC->sensor_orientation_);
    else
        return;



    pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GT, ui->ROI_XMin->value())));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT, ui->ROI_XMax->value())));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GT, ui->ROI_YMin->value())));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, ui->ROI_YMax->value())));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, ui->ROI_ZMin->value())));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT, ui->ROI_ZMax->value())));
    range_cond->addComparison (pcl::PackedRGBComparison<PointT>::ConstPtr (new pcl::PackedRGBComparison<PointT> ("r", pcl::ComparisonOps::GE, ui->ROI_RMin->value())));
    range_cond->addComparison (pcl::PackedRGBComparison<PointT>::ConstPtr (new pcl::PackedRGBComparison<PointT> ("r", pcl::ComparisonOps::LE, ui->ROI_RMax->value())));
    range_cond->addComparison (pcl::PackedRGBComparison<PointT>::ConstPtr (new pcl::PackedRGBComparison<PointT> ("g", pcl::ComparisonOps::GE, ui->ROI_GMin->value())));
    range_cond->addComparison (pcl::PackedRGBComparison<PointT>::ConstPtr (new pcl::PackedRGBComparison<PointT> ("g", pcl::ComparisonOps::LE, ui->ROI_GMax->value())));
    range_cond->addComparison (pcl::PackedRGBComparison<PointT>::ConstPtr (new pcl::PackedRGBComparison<PointT> ("b", pcl::ComparisonOps::GE, ui->ROI_BMin->value())));
    range_cond->addComparison (pcl::PackedRGBComparison<PointT>::ConstPtr (new pcl::PackedRGBComparison<PointT> ("b", pcl::ComparisonOps::LE, ui->ROI_BMax->value())));

    // build the filter
    pcl::ConditionalRemoval<PointT> condrem(true);
    condrem.setCondition(range_cond);


    condrem.setInputCloud (output);
    condrem.setKeepOrganized(output->isOrganized());


    PointCloudT::Ptr temp (new PointCloudT);
    condrem.filter (*temp);

    pcl::PointIndices::Ptr ind (new pcl::PointIndices());
    condrem.getRemovedIndices(*ind);
    //condrem.setUserFilterValue();

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (input);
    extract.setIndices (ind);
    extract.setNegative (true);
    extract.setKeepOrganized(input->isOrganized());
    extract.filter (*output);


    emit replace(input, output, QString("ROI"));

    updateMinMax();



    return;

}



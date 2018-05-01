#include "features_keypoints.h"
#include "ui_features_keypoints.h"

#include <pcl/filters/filter.h>
#include <QElapsedTimer>
Features_Keypoints::Features_Keypoints(QWidget *parent) :
    QWidget(parent), ui(new Ui::Features_Keypoints)
{
    ui->setupUi(this);
}

Features_Keypoints::~Features_Keypoints()
{
    delete ui;
}




//*******************//
//***  DETECTORS  ***//
//*******************//
#include <pcl/keypoints/brisk_2d.h>
void Features_Keypoints::DetectBRISK(PointCloudT::Ptr input, PointCloudT::Ptr output, int paramThreshold, int octave)
{
    std::cout << "BRISK Detector...";

    // constructor
    pcl::BriskKeypoint2D<PointT> brisk_keypoint_estimation;

    //output
    pcl::PointCloud<pcl::PointWithScale> brisk_keypoints_2D;

    //parameters
    brisk_keypoint_estimation.setThreshold(paramThreshold);
    brisk_keypoint_estimation.setOctaves(octave);
    brisk_keypoint_estimation.setInputCloud (input);

    //compute
    brisk_keypoint_estimation.compute (brisk_keypoints_2D);

    //convert pointwithscale to 3D
    output->resize(brisk_keypoints_2D.size());

    int k = brisk_keypoints_2D.size();
    for(int i = 0, j = 0; i < k; ++i)
    {
        /// TO DO: improve accuracy
        int u = floor(brisk_keypoints_2D.points[i].x + 0.5);
        int v = floor(brisk_keypoints_2D.points[i].y + 0.5);

        j = u + v * input->width;

        if(std::isnan(input->points[j].x))
        {
            --k;
        }
        else
        {
            output->points[i]=input->points[j];
            /* output->points[i].b=input->points[j].b;
            output->points[i].g=input->points[j].g;
            output->points[i].r=input->points[j].r;
            output->points[i].x=input->points[j].x;
            output->points[i].y=input->points[j].y;
            output->points[i].z=input->points[j].z;*/
        }
    }

    std::vector<PointT,Eigen::aligned_allocator<PointT> >::iterator    keypointIt=output->begin();

    for(size_t i=k; k<brisk_keypoints_2D.size(); ++k)
        output->erase(keypointIt+i);



    pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    // Remove NAN from input cloud
    pcl::removeNaNFromPointCloud(*input, *input,indices->indices);
    pcl::removeNaNNormalsFromPointCloud(*input, *input,indices->indices);

    // Remove NAN from keypoints cloud
    pcl::removeNaNFromPointCloud(*output, *output,indices->indices);
    pcl::removeNaNNormalsFromPointCloud(*output, *output,indices->indices);


    // remove 0 points
    pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, 0.1)));

    // build the filter
    pcl::ConditionalRemoval<PointT> condrem(true);
    condrem.setCondition(range_cond);

    condrem.setInputCloud (output);
    condrem.setKeepOrganized(output->isOrganized());
    condrem.filter (*output);


    std::cout << "DONE " << output->size() << std::endl;
}


#include <pcl/keypoints/brisk_2d.h>
void Features_Keypoints::DetectBRISKQuad(PointCloudT::Ptr input, PointCloudT::Ptr output, int paramThreshold, int octave)
{
    std::cout << "BRISK Detector...";

    // constructor
    pcl::BriskKeypoint2D<PointT> brisk_keypoint_estimation;

    //output
    pcl::PointCloud<pcl::PointWithScale> brisk_keypoints_2D;

    //parameters
    brisk_keypoint_estimation.setThreshold(paramThreshold);
    brisk_keypoint_estimation.setOctaves(octave);
    brisk_keypoint_estimation.setInputCloud (input);

    //compute
    brisk_keypoint_estimation.compute (brisk_keypoints_2D);

    //convert pointwithscale to 3D
    output->resize(brisk_keypoints_2D.size());

    int k = brisk_keypoints_2D.size();
    for(int i = 0, j = 0; i < k; ++i)
    {
        /// TO DO: improve accuracy


        int umin = floor(brisk_keypoints_2D.points[i].x);
        int vmin = floor(brisk_keypoints_2D.points[i].y);
        int umax = ceil(brisk_keypoints_2D.points[i].x);
        int vmax = ceil(brisk_keypoints_2D.points[i].y);
        double ures = brisk_keypoints_2D.points[i].x - floor(brisk_keypoints_2D.points[i].x);
        double vres = brisk_keypoints_2D.points[i].y - floor(brisk_keypoints_2D.points[i].y);

        PointT TL = input->points[umin + vmax * input->width];
        PointT TR = input->points[umax + vmax * input->width];
        PointT BL = input->points[umin + vmin * input->width];
        PointT BR = input->points[umax + vmin * input->width];

        double wTL =    ures +(1-vres);
        double wTR = (1-ures)+(1-vres);
        double wBL =    ures +   vres;
        double wBR = (1-ures)+   vres;


        if(std::isnan(TL.x) || std::isnan(TR.x) || std::isnan(BL.x) || std::isnan(BR.x))
        {
            --k;
        }
        else
        {
            output->points[i].b = (wTL*TL.b + wTR*TR.b + wBL*BL.b + wBR*BR.b) / (wTL+wTR+wBL+wBR);
            output->points[i].g = (wTL*TL.g + wTR*TR.g + wBL*BL.g + wBR*BR.g) / (wTL+wTR+wBL+wBR);
            output->points[i].r = (wTL*TL.r + wTR*TR.r + wBL*BL.r + wBR*BR.r) / (wTL+wTR+wBL+wBR);

            output->points[i].x = (wTL*TL.x + wTR*TR.x + wBL*BL.x + wBR*BR.x) / (wTL+wTR+wBL+wBR);
            output->points[i].y = (wTL*TL.y + wTR*TR.y + wBL*BL.y + wBR*BR.y) / (wTL+wTR+wBL+wBR);
            output->points[i].z = (wTL*TL.z + wTR*TR.z + wBL*BL.z + wBR*BR.z) / (wTL+wTR+wBL+wBR);
        }
    }

    std::vector<PointT,Eigen::aligned_allocator<PointT> >::iterator    keypointIt=output->begin();

    for(size_t i=k; k<brisk_keypoints_2D.size(); ++k)
        output->erase(keypointIt+i);



    //   pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    // Remove NAN from input cloud
    //   pcl::removeNaNFromPointCloud(*input, *input,indices->indices);
    //  pcl::removeNaNNormalsFromPointCloud(*input, *input,indices->indices);

    // Remove NAN from keypoints cloud
    // pcl::removeNaNFromPointCloud(*output, *output,indices->indices);
    //  pcl::removeNaNNormalsFromPointCloud(*output, *output,indices->indices);


    std::cout << "DONE " << output->size() << std::endl;
}


#include <pcl/keypoints/agast_2d.h>
void Features_Keypoints::DetectAGAST(PointCloudT::Ptr input, PointCloudT::Ptr output, int paramThreshold)
{
    std::cout << "AGAST Detector...";

    // constructor
    pcl::AgastKeypoint2D<PointT> agast_keypoint_estimation;

    //output
    pcl::PointCloud<pcl::PointUV> agast_keypoints_2D;

    //parameters
    agast_keypoint_estimation.setThreshold (paramThreshold);
    agast_keypoint_estimation.setInputCloud (input);

    //compute
    agast_keypoint_estimation.compute (agast_keypoints_2D);

    //convert UV to 3D
    output->resize(agast_keypoints_2D.size());
    int k = agast_keypoints_2D.size();
    for(int i = 0, j = 0; i < k; ++i)
    {
        j = agast_keypoints_2D.points[i].u + agast_keypoints_2D.points[i].v * input->width;

        if(std::isnan(input->points[j].x))
        {
            --k;
        }
        else
        {
            output->points[i].b=input->points[j].b;
            output->points[i].g=input->points[j].g;
            output->points[i].r=input->points[j].r;
            output->points[i].x=input->points[j].x;
            output->points[i].y=input->points[j].y;
            output->points[i].z=input->points[j].z;
        }
    }

    std::vector<PointT,Eigen::aligned_allocator<PointT> >::iterator    keypointIt=output->begin();

    for(int i=k; k<agast_keypoints_2D.size(); ++k)
        output->erase(keypointIt+i);


    //    pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    // Remove NAN from input cloud
    //  pcl::removeNaNFromPointCloud(*input, *input,indices->indices);
    //  pcl::removeNaNNormalsFromPointCloud(*input, *input,indices->indices);

    // Remove NAN from keypoints cloud
    //  pcl::removeNaNFromPointCloud(*output, *output,indices->indices);
    //  pcl::removeNaNNormalsFromPointCloud(*output, *output,indices->indices);


    std::cout << "DONE " << output->size() << std::endl;
}

#include <pcl/keypoints/sift_keypoint.h>
void Features_Keypoints::DetectSIFT(PointCloudT::Ptr input, PointCloudT::Ptr output)
{
    std::cout << "SIFT Detector...";

    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI> sift_detect;

    const float min_scale = 0.005f;
    const int nr_octaves = 6;
    const int nr_scales_per_octave = 4;
    const float min_contrast = 0.005f;


    sift_detect.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
    sift_detect.setMinimumContrast (min_contrast);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputXYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*input, *inputXYZRGB);
    sift_detect.setInputCloud (inputXYZRGB);

    pcl::PointCloud<pcl::PointXYZI> keypoints_temp;
    sift_detect.compute (keypoints_temp);

    //OPT1
    // pcl::PointIndices::Ptr ind (new pcl::PointIndices ());
    //OPT2
    // std::vector<int> ind2;
    // ind2 = sift_detect.getKeypointsIndices();
    //
    // pcl::ExtractIndices<PointT> extract;
    //extract.setInputCloud (input);
    // extract.setIndices (ind);
    //  extract.setNegative (false);
    //  extract.filter (*output);


    output->resize(keypoints_temp.size());
    pcl::copyPointCloud (keypoints_temp, *output);




    //  pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    // Remove NAN from input cloud
    //  pcl::removeNaNFromPointCloud(*input, *input,indices->indices);
    //  pcl::removeNaNNormalsFromPointCloud(*input, *input,indices->indices);

    // Remove NAN from keypoints cloud
    //  pcl::removeNaNFromPointCloud(*output, *output,indices->indices);
    //  pcl::removeNaNNormalsFromPointCloud(*output, *output,indices->indices);

    std::cout << "DONE " << output->size() << std::endl;
    return ;
}


void Features_Keypoints::DetectCurvature(PointCloudT::Ptr input, PointCloudT::Ptr output, double curvature)
{
    std::cout << "Curvature Detector...";


    pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("curvature", pcl::ComparisonOps::GT, ui->doubleSpinBox_Curvature->value())));

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




    std::cout << "DONE " << output->size() << std::endl;
    return ;
}




void Features_Keypoints::on_pushButton_Compute_clicked()
{
    std::cout << "Computing keypoints ";

    if (CurrentPC != NULL)
    {

        PointCloudT::Ptr input(CurrentPC);
        PointCloudT::Ptr output(new PointCloudT);
        pcl::copyPointCloud(*input, *output);

        std::cout << "INIT " << input->size() << std::endl;

        if (ui->radioButton_BRISK->isChecked())
        {
            std::cout << "BRISK...";
            DetectBRISK(input, output, ui->spinBox_BRISK->value() );
        }
        else if (ui->radioButton_BRISKQuad->isChecked())
        {
            std::cout << "BRISK Quad...";
            DetectBRISKQuad(input, output, ui->spinBox_BRISK->value() );
        }
        else if (ui->radioButton_AGAST->isChecked())
        {
            std::cout << "AGAST...";
            DetectAGAST(input, output, ui->spinBox_AGAST->value());
        }
        else if (ui->radioButton_SIFT3D->isChecked())
        {
            std::cout << "SIFT...";
            DetectSIFT(input, output);
        }
        else if (ui->radioButton_Curvature->isChecked())
        {
            std::cout << "CURVE...";
            DetectCurvature(input, output, ui->doubleSpinBox_Curvature->value());
        }

        output->sensor_orientation_ = input->sensor_orientation_;

        output->sensor_origin_ = input->sensor_origin_;



        emit replace(input, output, QString("Keypoints"));
    }
    std::cout << "DONE"<<std::endl;


    return;

}



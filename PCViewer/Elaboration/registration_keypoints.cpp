#include "registration_keypoints.h"
#include "ui_registration_keypoints.h"

Registration_Keypoints::Registration_Keypoints(QWidget *parent) :
    QWidget(parent), ui(new Ui::Registration_Keypoints)
{
    ui->setupUi(this);
}

Registration_Keypoints::~Registration_Keypoints()
{
    delete ui;
}

using namespace std;


void Registration_Keypoints::updateList()
{
    QString ModelString = ui->comboBox_Model->currentText();
    QString PCString = ui->comboBox_PC_to_Align->currentText();

    ui->comboBox_Model->clear();
    ui->comboBox_PC_to_Align->clear();

    for (int i = 0; i < CurrentPCList.size(); i++)
    {
        QString PCname = QString::fromStdString(CurrentPCList.at(i)->header.frame_id);
        ui->comboBox_Model->addItem(PCname);
        ui->comboBox_PC_to_Align->addItem(PCname);
    }
    // Qt 4
    for (int i = 0; i < CurrentPCList.size(); i++)
    {
        if (ui->comboBox_Model->itemText(i) == ModelString)
            ui->comboBox_Model->setCurrentIndex(i);
        if (ui->comboBox_PC_to_Align->itemText(i) == PCString)
            ui->comboBox_PC_to_Align->setCurrentIndex(i);
    }
}



void Registration_Keypoints::setViewer(pcl::visualization::PCLVisualizer::Ptr myViewer)
{
    viewer = myViewer;
    return;
}


///  DETECTORS

#include <pcl/keypoints/brisk_2d.h>
void Registration_Keypoints::DetectBRISK(PointCloudT::Ptr input, PointCloudT::Ptr output, int paramThreshold, int octave)
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

    //convert UV to 3D
    output->resize(brisk_keypoints_2D.size());

    int k = brisk_keypoints_2D.size();
    for(int i = 0, j = 0; i < k; ++i)
    {
        /// TO DO: improve accuracy
        int u = floor(brisk_keypoints_2D.points[i].x + 0.5);
        int v = floor(brisk_keypoints_2D.points[i].y + 0.5);

        j = u + v * input->width;

        if(isnan(input->points[j].x))
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

    for(size_t i=k; k<brisk_keypoints_2D.size(); ++k)
        output->erase(keypointIt+i);



    pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    // Remove NAN from input cloud
    pcl::removeNaNFromPointCloud(*input, *input,indices->indices);
    pcl::removeNaNNormalsFromPointCloud(*input, *input,indices->indices);

    // Remove NAN from keypoints cloud
    pcl::removeNaNFromPointCloud(*output, *output,indices->indices);
    pcl::removeNaNNormalsFromPointCloud(*output, *output,indices->indices);


    std::cout << "DONE"<<std::endl;
}


#include <pcl/keypoints/agast_2d.h>
void Registration_Keypoints::DetectAGAST(PointCloudT::Ptr input, PointCloudT::Ptr output, int paramThreshold)
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

        if(isnan(input->points[j].x))
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


    pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    // Remove NAN from input cloud
    pcl::removeNaNFromPointCloud(*input, *input,indices->indices);
    pcl::removeNaNNormalsFromPointCloud(*input, *input,indices->indices);

    // Remove NAN from keypoints cloud
    pcl::removeNaNFromPointCloud(*output, *output,indices->indices);
    pcl::removeNaNNormalsFromPointCloud(*output, *output,indices->indices);


    std::cout << "DONE"<<endl;
}


#include <pcl/keypoints/sift_keypoint.h>
void Registration_Keypoints::DetectSIFT(PointCloudT::Ptr input, PointCloudT::Ptr output)
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




    pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    // Remove NAN from input cloud
    pcl::removeNaNFromPointCloud(*input, *input,indices->indices);
    pcl::removeNaNNormalsFromPointCloud(*input, *input,indices->indices);

    // Remove NAN from keypoints cloud
    pcl::removeNaNFromPointCloud(*output, *output,indices->indices);
    pcl::removeNaNNormalsFromPointCloud(*output, *output,indices->indices);

    std::cout << "DONE"<<endl;
    return ;
}





///  DESCRIPTORS

#include <pcl/features/fpfh.h>
void Registration_Keypoints::DescribeFPFH(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptor)
{
    std::cout << "Description with FPFH..." <<std::endl;

    QElapsedTimer *Timer = new QElapsedTimer();
    Timer->start();


    // Calculate features FPFH
    pcl::FPFHEstimation<PointT, PointT, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud (keypoints);
    fpfh.setSearchSurface(input);
    fpfh.setInputNormals (input);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    fpfh.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (0.05);

    // Compute the features
    fpfh.compute(*descriptor);

    cout << "OK. keypoints described in " << Timer->elapsed() << " milliseconds" << endl;

    /*
    // Check if there are NAN points in input cloud
    for ( size_t i=0; i<input_noNAN->size(); i++ )
    {
        cout <<  "x:   " << input_noNAN->points[i].x << "   y:   " << input_noNAN->points[i].y << "   z:   " << input_noNAN->points[i].z <<"  normal_x:   " << input_noNAN->points[i].normal_x  << "  normal_y:   "<< input_noNAN->points[i].normal_y <<"  normal_z:   "<< input_noNAN->points[i].normal_z <<endl;
    }

    // Visualize histogram
    for ( size_t i=0; i<descriptor->size(); i++ )
        for ( int j=0; j<33; j++ )
            cout << descriptor->points[i].histogram[j] <<endl;
*/

    pcl::PointIndices::Ptr indices_NAN_feature (new pcl::PointIndices);
    bool found_NAN_at_i =false;

    //Take Nan indices of point of features that have an histogram value NAN
    for ( size_t i=0; i<descriptor->size(); i++ )
    {
        found_NAN_at_i =false;
        for ( int j=0; j<33; j++ )
        {
            if( !pcl_isfinite(descriptor->points[i].histogram[j]))
            {
                found_NAN_at_i =true;
            }
        }
        if(found_NAN_at_i)
            indices_NAN_feature->indices.push_back(i);
    }

    // Filter descriptor with the indices founded
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptor_noNAN ( new pcl::PointCloud<pcl::FPFHSignature33>);

    pcl::ExtractIndices<pcl::FPFHSignature33> extract;
    extract.setInputCloud (descriptor);
    extract.setIndices (indices_NAN_feature);
    extract.setNegative (true);
    extract.filter (*descriptor_noNAN);
    *descriptor = *descriptor_noNAN;

    /*
      // Check same length -> obviously not!
      cout << "Try: Is:" << endl;
      cout << descriptor->size() << " = " << keypoints->size() << endl;
     */

    // We have to erase the keypoints that have no more features.
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extractKP;
    extractKP.setInputCloud (keypoints);
    extractKP.setIndices (indices_NAN_feature);
    extractKP.setNegative (true);
    extractKP.filter (*keypoints);


    // Check same length ->obviously yes!
    cout << "Try2: Is:" << endl;
    cout <<  descriptor->size() << " = " << keypoints->size() << endl;


}

void Registration_Keypoints::DescribePFH(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptor)
{

    cout << "Description with PFH..." <<endl;

    QElapsedTimer *Timer = new QElapsedTimer();
    Timer->start();


    // Calculate features PFH
    pcl::PFHEstimation<PointT, PointT, pcl::PFHSignature125> pfh;
    pfh.setInputCloud (keypoints);
    pfh.setSearchSurface(input);
    pfh.setInputNormals (input);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    pfh.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    pfh.setRadiusSearch (0.05);

    pfh.compute(*descriptor);
    // Compute the features

    cout << "OK. keypoints described in " << Timer->elapsed() << " milliseconds" << endl;

    /*
    // Check if there are NAN points in input cloud
    for ( size_t i=0; i<input_noNAN->size(); i++ )
    {
        cout <<  "x:   " << input_noNAN->points[i].x << "   y:   " << input_noNAN->points[i].y << "   z:   " << input_noNAN->points[i].z <<"  normal_x:   " << input_noNAN->points[i].normal_x  << "  normal_y:   "<< input_noNAN->points[i].normal_y <<"  normal_z:   "<< input_noNAN->points[i].normal_z <<endl;
    }

    // Visualize histogram
    for ( size_t i=0; i<descriptor->size(); i++ )
        for ( int j=0; j<33; j++ )
            cout << descriptor->points[i].histogram[j] <<endl;
*/

    pcl::PointIndices::Ptr indices_NAN_feature (new pcl::PointIndices);
    bool found_NAN_at_i =false;

    //Take Nan indices of point of features that have an histogram value NAN
    for ( size_t i=0; i<descriptor->size(); i++ )
    {
        found_NAN_at_i =false;
        for ( int j=0; j<125; j++ )
        {
            if( !pcl_isfinite(descriptor->points[i].histogram[j]))
            {
                found_NAN_at_i =true;
            }
        }
        if(found_NAN_at_i)
            indices_NAN_feature->indices.push_back(i);
    }

    // Filter descriptor with the indices founded
    pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptor_noNAN ( new pcl::PointCloud<pcl::PFHSignature125>);

    pcl::ExtractIndices<pcl::PFHSignature125> extract;
    extract.setInputCloud (descriptor);
    extract.setIndices (indices_NAN_feature);
    extract.setNegative (true);
    extract.filter (*descriptor_noNAN);
    *descriptor = *descriptor_noNAN;

    /*
      // Check same length -> obviously not!
      cout << "Try: Is:" << endl;
      cout << descriptor->size() << " = " << keypoints->size() << endl;
     */

    // We have to erase the keypoints that have no more features.
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extractKP;
    extractKP.setInputCloud (keypoints);
    extractKP.setIndices (indices_NAN_feature);
    extractKP.setNegative (true);
    extractKP.filter (*keypoints);

    // Check same length ->obviously yes!
    cout << "Try2: Is:" << endl;
    cout <<  descriptor->size() << " = " << keypoints->size() << endl;
}

void Registration_Keypoints::DescribePFHRGB(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descriptor)
{
    std::cout << "Description with PFHRGB..." <<endl;

    QElapsedTimer *Timer = new QElapsedTimer();
    Timer->start();


    // Calculate features PFHRGB
    pcl::PFHRGBEstimation<PointT, PointT, pcl::PFHRGBSignature250> pfhrgb;
    pfhrgb.setInputCloud (keypoints);
    pfhrgb.setSearchSurface(input);
    pfhrgb.setInputNormals (input);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    pfhrgb.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    pfhrgb.setRadiusSearch (0.05);

    // Compute the features
    pfhrgb.compute(*descriptor);

    cout << "OK. keypoints described in " << Timer->elapsed() << " milliseconds" << endl;

    /*
    // Check if there are NAN points in input cloud
    for ( size_t i=0; i<input_noNAN->size(); i++ )
    {
        cout <<  "x:   " << input_noNAN->points[i].x << "   y:   " << input_noNAN->points[i].y << "   z:   " << input_noNAN->points[i].z <<"  normal_x:   " << input_noNAN->points[i].normal_x  << "  normal_y:   "<< input_noNAN->points[i].normal_y <<"  normal_z:   "<< input_noNAN->points[i].normal_z <<endl;
    }

    // Visualize histogram
    for ( size_t i=0; i<descriptor->size(); i++ )
        for ( int j=0; j<33; j++ )
            cout << descriptor->points[i].histogram[j] <<endl;
*/

    pcl::PointIndices::Ptr indices_NAN_feature (new pcl::PointIndices);
    bool found_NAN_at_i =false;

    //Take Nan indices of point of features that have an histogram value NAN
    for ( size_t i=0; i<descriptor->size(); i++ )
    {
        found_NAN_at_i =false;
        for ( int j=0; j<250; j++ )
        {
            if( !pcl_isfinite(descriptor->points[i].histogram[j]))
            {
                found_NAN_at_i =true;
            }
        }
        if(found_NAN_at_i)
            indices_NAN_feature->indices.push_back(i);
    }

    // Filter descriptor with the indices founded
    pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descriptor_noNAN ( new pcl::PointCloud<pcl::PFHRGBSignature250>);

    pcl::ExtractIndices<pcl::PFHRGBSignature250> extract;
    extract.setInputCloud (descriptor);
    extract.setIndices (indices_NAN_feature);
    extract.setNegative (true);
    extract.filter (*descriptor_noNAN);
    *descriptor = *descriptor_noNAN;

    /*
      // Check same length -> obviously not!
      cout << "Try: Is:" << endl;
      cout << descriptor->size() << " = " << keypoints->size() << endl;
     */

    // We have to erase the keypoints that have no more features.
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extractKP;
    extractKP.setInputCloud (keypoints);
    extractKP.setIndices (indices_NAN_feature);
    extractKP.setNegative (true);
    extractKP.filter (*keypoints);

    // Check same length ->obviously yes!
    cout << "Try2: Is:" << endl;
    cout <<  descriptor->size() << " = " << keypoints->size() << endl;
}

void Registration_Keypoints::DescribeSHOT(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::SHOT352>::Ptr descriptor)
{
    cout << "Description with SHOT..." <<endl;

    QElapsedTimer *Timer = new QElapsedTimer();
    Timer->start();


    // Calculate features SHOT
    pcl::SHOTEstimation<PointT, PointT, pcl::SHOT352> shot;
    shot.setInputCloud (keypoints);
    shot.setSearchSurface(input);
    shot.setInputNormals (input);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    shot.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    shot.setRadiusSearch (0.05);

    // Compute the features
    shot.compute(*descriptor);

    cout << "OK. keypoints described in " << Timer->elapsed() << " milliseconds" << endl;

    /*
    // Check if there are NAN points in input cloud
    for ( size_t i=0; i<input_noNAN->size(); i++ )
    {
        cout <<  "x:   " << input_noNAN->points[i].x << "   y:   " << input_noNAN->points[i].y << "   z:   " << input_noNAN->points[i].z <<"  normal_x:   " << input_noNAN->points[i].normal_x  << "  normal_y:   "<< input_noNAN->points[i].normal_y <<"  normal_z:   "<< input_noNAN->points[i].normal_z <<endl;
    }

    // Visualize histogram
    for ( size_t i=0; i<descriptor->size(); i++ )
        for ( int j=0; j<33; j++ )
            cout << descriptor->points[i].histogram[j] <<endl;
*/

    pcl::PointIndices::Ptr indices_NAN_feature (new pcl::PointIndices);
    bool found_NAN_at_i =false;

    //Take Nan indices of point of features that have an histogram value NAN
    for ( size_t i=0; i<descriptor->size(); i++ )
    {
        found_NAN_at_i =false;
        for ( int j=0; j<352; j++ )
        {
            if( !pcl_isfinite(descriptor->points[i].descriptor[j]))
            {
                found_NAN_at_i =true;
            }
        }
        if(found_NAN_at_i)
            indices_NAN_feature->indices.push_back(i);
    }

    // Filter descriptor with the indices founded
    pcl::PointCloud<pcl::SHOT352>::Ptr descriptor_noNAN ( new pcl::PointCloud<pcl::SHOT352>);

    pcl::ExtractIndices<pcl::SHOT352> extract;
    extract.setInputCloud (descriptor);
    extract.setIndices (indices_NAN_feature);
    extract.setNegative (true);
    extract.filter (*descriptor_noNAN);
    *descriptor = *descriptor_noNAN;

    /*
      // Check same length -> obviously not!
      cout << "Try: Is:" << endl;
      cout << descriptor->size() << " = " << keypoints->size() << endl;
     */

    // We have to erase the keypoints that have no more features.
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extractKP;
    extractKP.setInputCloud (keypoints);
    extractKP.setIndices (indices_NAN_feature);
    extractKP.setNegative (true);
    extractKP.filter (*keypoints);

    // Check same length ->obviously yes!
    cout << "Try2: Is:" << endl;
    cout <<  descriptor->size() << " = " << keypoints->size() << endl;
}

void Registration_Keypoints::DescribeCSHOT(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::SHOT1344>::Ptr descriptor )
{

    cout << "Description with SHOTRGB..." <<endl;

    QElapsedTimer *Timer = new QElapsedTimer();
    Timer->start();


    // Calculate features FPFH
    pcl::SHOTColorEstimation<PointT, PointT, pcl::SHOT1344> shotrgb;
    shotrgb.setInputCloud (keypoints);
    shotrgb.setSearchSurface(input);
    shotrgb.setInputNormals (input);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    shotrgb.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    shotrgb.setRadiusSearch (0.05);

    // Compute the features
    shotrgb.compute(*descriptor);

    cout << "OK. keypoints described in " << Timer->elapsed() << " milliseconds" << endl;

    /*
    // Check if there are NAN points in input cloud
    for ( size_t i=0; i<input_noNAN->size(); i++ )
    {
        cout <<  "x:   " << input_noNAN->points[i].x << "   y:   " << input_noNAN->points[i].y << "   z:   " << input_noNAN->points[i].z <<"  normal_x:   " << input_noNAN->points[i].normal_x  << "  normal_y:   "<< input_noNAN->points[i].normal_y <<"  normal_z:   "<< input_noNAN->points[i].normal_z <<endl;
    }

    // Visualize histogram
    for ( size_t i=0; i<descriptor->size(); i++ )
        for ( int j=0; j<33; j++ )
            cout << descriptor->points[i].histogram[j] <<endl;
*/

    pcl::PointIndices::Ptr indices_NAN_feature (new pcl::PointIndices);
    bool found_NAN_at_i =false;

    //Take Nan indices of point of features that have an histogram value NAN
    for ( size_t i=0; i<descriptor->size(); i++ )
    {
        found_NAN_at_i =false;
        for ( int j=0; j<1344; j++ )        
            if( !pcl_isfinite(descriptor->points[i].descriptor[j]))            
                found_NAN_at_i =true;


        if(found_NAN_at_i)
            indices_NAN_feature->indices.push_back(i);
    }

    // Filter descriptor with the indices founded
    pcl::PointCloud<pcl::SHOT1344>::Ptr descriptor_noNAN ( new pcl::PointCloud<pcl::SHOT1344>);

    pcl::ExtractIndices<pcl::SHOT1344> extract;
    extract.setInputCloud (descriptor);
    extract.setIndices (indices_NAN_feature);
    extract.setNegative (true);
    extract.filter (*descriptor_noNAN);
    *descriptor = *descriptor_noNAN;

    /*
      // Check same length -> obviously not!
      cout << "Try: Is:" << endl;
      cout << descriptor->size() << " = " << keypoints->size() << endl;
     */

    // We have to erase the keypoints that have no more features.
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extractKP;
    extractKP.setInputCloud (keypoints);
    extractKP.setIndices (indices_NAN_feature);
    extractKP.setNegative (true);
    extractKP.filter (*keypoints);

    // Check same length ->obviously yes!
    cout << "Try2: Is:" << endl;
    cout <<  descriptor->size() << " = " << keypoints->size() << endl;
}




///  CORRESPONDENCES FINDING

void Registration_Keypoints::FindCorrespondencePoints (PointCloudT::Ptr features_1, PointCloudT::Ptr features_2, pcl::CorrespondencesPtr correspondences)
{
    pcl::registration::CorrespondenceEstimation<PointT, PointT> est;
    est.setInputSource (features_2);
    est.setInputTarget (features_1);
    est.determineReciprocalCorrespondences (*correspondences);

}

void Registration_Keypoints::FindCorrespondenceFPFH (pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_1,pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_2, pcl::CorrespondencesPtr correspondences)
{
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
    est.setInputSource (features_2);
    est.setInputTarget (features_1);
    est.determineReciprocalCorrespondences (*correspondences);

}

void Registration_Keypoints::FindCorrespondencePFH (pcl::PointCloud<pcl::PFHSignature125>::Ptr features_1 ,pcl::PointCloud<pcl::PFHSignature125>::Ptr features_2, pcl::CorrespondencesPtr correspondences)
{
    pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> est;
    est.setInputSource (features_2);
    est.setInputTarget (features_1);
    est.determineReciprocalCorrespondences (*correspondences);

}

void Registration_Keypoints::FindCorrespondencePFHRGB (pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr features_1 ,pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr features_2, pcl::CorrespondencesPtr correspondences)
{
    pcl::registration::CorrespondenceEstimation<pcl::PFHRGBSignature250, pcl::PFHRGBSignature250> est;
    est.setInputSource (features_2);
    est.setInputTarget (features_1);

}

void Registration_Keypoints::FindCorrespondenceSHOT (pcl::PointCloud<pcl::SHOT352>::Ptr features_1 ,pcl::PointCloud<pcl::SHOT352>::Ptr features_2, pcl::CorrespondencesPtr correspondences)
{
    pcl::registration::CorrespondenceEstimation<pcl::SHOT352, pcl::SHOT352> est;
    est.setInputSource (features_2);
    est.setInputTarget (features_1);
    est.determineReciprocalCorrespondences (*correspondences);
}

void Registration_Keypoints::FindCorrespondenceCSHOT(pcl::PointCloud<pcl::SHOT1344>::Ptr features_1 ,pcl::PointCloud<pcl::SHOT1344>::Ptr features_2, pcl::CorrespondencesPtr correspondences)
{
    pcl::registration::CorrespondenceEstimation<pcl::SHOT1344, pcl::SHOT1344> est;
    est.setInputSource (features_2);
    est.setInputTarget (features_1);
    est.determineReciprocalCorrespondences (*correspondences);
}



#include <QElapsedTimer>
void Registration_Keypoints::on_pushButton_clicked()
{

    // START TIMER
    QElapsedTimer *Timer = new QElapsedTimer();
    Timer->start();

    // INPUT
  /*    PointCloudT::Ptr input(new PointCloudT);
    input = CurrentPCList.at(ui->comboBox_Model->currentIndex());

    PointCloudT::Ptr input_2(new PointCloudT);
    input_2 = CurrentPCList.at(ui->comboBox_PC_to_Align->currentIndex());
*/
    PointCloudT::Ptr input(new PointCloudT);
    pcl::copyPointCloud(*CurrentPCList.at(ui->comboBox_Model->currentIndex()), *input);
    pcl::transformPointCloud(*input, *input, input->sensor_origin_.head(3), input->sensor_orientation_);
    input->sensor_orientation_ = Eigen::Quaternionf::Identity();
    input->sensor_origin_ = Eigen::Vector4f::Zero();

    PointCloudT::Ptr input_2(new PointCloudT);
    pcl::copyPointCloud(*CurrentPCList.at(ui->comboBox_PC_to_Align->currentIndex()), *input_2);
    pcl::transformPointCloud(*input_2, *input_2, input_2->sensor_origin_.head(3), input_2->sensor_orientation_);
    input_2->sensor_orientation_ = Eigen::Quaternionf::Identity();
    input_2->sensor_origin_ = Eigen::Vector4f::Zero();




    int param_thresholt = ui->spinBox_threshold_Detector->value();






    // DETECTION Keypoint
    PointCloudT::Ptr input_keypoints (new PointCloudT);
    PointCloudT::Ptr input_keypoints_2 (new PointCloudT);
    if ( QString::compare(ui->comboBox_FeaturesType->currentText(), QString("SIFT3D"), Qt::CaseInsensitive) == 0)
    {
        DetectSIFT(input, input_keypoints);
        DetectSIFT(input_2,input_keypoints_2);
    }

    if ( QString::compare(ui->comboBox_FeaturesType->currentText(), QString("AGAST"), Qt::CaseInsensitive) == 0)
    {
        DetectAGAST(input, input_keypoints,param_thresholt);
        DetectAGAST(input_2, input_keypoints_2,param_thresholt);
    }

    if ( QString::compare(ui->comboBox_FeaturesType->currentText(), QString("BRISK"), Qt::CaseInsensitive) == 0)
    {
        DetectBRISK(input, input_keypoints, param_thresholt,4);
        DetectBRISK(input_2, input_keypoints_2, param_thresholt,4);
    }

  /*  input_keypoints->header.frame_id = input->header.frame_id + ui->comboBox_FeaturesType->currentText().toStdString();
    emit addPC(input_keypoints);

    input_keypoints_2->header.frame_id = input_2->header.frame_id + ui->comboBox_FeaturesType->currentText().toStdString();
    emit addPC(input_keypoints_2);
*/

    cout << "OK. keypoints of first PC (dense=" << input_keypoints->is_dense << ") found: " << input_keypoints->points.size() <<  endl;
    cout << "OK. keypoints of second PC (dense=" << input_keypoints_2->is_dense << ") found: " << input_keypoints_2->points.size() << endl;
    cout << "In a total time: " << Timer->elapsed() << " milliseconds" << endl;








    // DESCRIPTION + CORISPONDENCEs
    Timer->restart();
    cout << "Search correspondence..." <<endl;
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);

    if (QString::compare(ui->comboBox_DescriptorType->currentText(), QString("Points"), Qt::CaseInsensitive) == 0)
    {

      //  pcl::PointCloud<pcl::FPFHSignature33>::Ptr input_descriptor ( new pcl::PointCloud<pcl::FPFHSignature33>);
      //  pcl::PointCloud<pcl::FPFHSignature33>::Ptr input_descriptor_2 ( new pcl::PointCloud<pcl::FPFHSignature33>);

      //  DescribeFPFH(input, input_keypoints, input_descriptor);
      //  DescribeFPFH(input_2, input_keypoints_2, input_descriptor_2);

        FindCorrespondencePoints(input, input_2, correspondences);

       // cout << "Try3: Is:" << endl;
      //  cout <<  input_descriptor->size() << " = " << input_keypoints->size() << endl;
    }

    else if (QString::compare(ui->comboBox_DescriptorType->currentText(), QString("FPFH"), Qt::CaseInsensitive) == 0)
    {

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr input_descriptor ( new pcl::PointCloud<pcl::FPFHSignature33>);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr input_descriptor_2 ( new pcl::PointCloud<pcl::FPFHSignature33>);

        DescribeFPFH(input, input_keypoints, input_descriptor);
        DescribeFPFH(input_2, input_keypoints_2, input_descriptor_2);

        FindCorrespondenceFPFH(input_descriptor, input_descriptor_2, correspondences);

        cout << "Try3: Is:" << endl;
       cout <<  input_descriptor->size() << " = " << input_keypoints->size() << endl;
    }

    else if (QString::compare(ui->comboBox_DescriptorType->currentText(), QString("PFH"), Qt::CaseInsensitive) == 0)
    {
        pcl::PointCloud<pcl::PFHSignature125>::Ptr input_descriptor ( new pcl::PointCloud<pcl::PFHSignature125>);
        pcl::PointCloud<pcl::PFHSignature125>::Ptr input_descriptor_2 ( new pcl::PointCloud<pcl::PFHSignature125>);

        DescribePFH(input, input_keypoints, input_descriptor);
        DescribePFH(input_2, input_keypoints_2, input_descriptor_2);

        FindCorrespondencePFH(input_descriptor, input_descriptor_2, correspondences);

        cout << "Try3: Is:" << endl;
        cout <<  input_descriptor->size() << " = " << input_keypoints->size() << endl;
    }


    else if (QString::compare(ui->comboBox_DescriptorType->currentText(), QString("PFHRGB"), Qt::CaseInsensitive) == 0)
    {
        pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr input_descriptor ( new pcl::PointCloud<pcl::PFHRGBSignature250>);
        pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr input_descriptor_2 ( new pcl::PointCloud<pcl::PFHRGBSignature250>);

        DescribePFHRGB(input, input_keypoints, input_descriptor);
        DescribePFHRGB(input_2, input_keypoints_2, input_descriptor_2);

        FindCorrespondencePFHRGB(input_descriptor, input_descriptor_2, correspondences);

        cout << "Try3: Is:" << endl;
        cout <<  input_descriptor->size() << " = " << input_keypoints->size() << endl;

    }

    else if (QString::compare(ui->comboBox_DescriptorType->currentText(), QString("SHOT"), Qt::CaseInsensitive) == 0)
    {
        pcl::PointCloud<pcl::SHOT352>::Ptr input_descriptor ( new pcl::PointCloud<pcl::SHOT352>);
        pcl::PointCloud<pcl::SHOT352>::Ptr input_descriptor_2 ( new pcl::PointCloud<pcl::SHOT352>);

        DescribeSHOT(input, input_keypoints, input_descriptor);
        DescribeSHOT(input_2, input_keypoints_2, input_descriptor_2);

        FindCorrespondenceSHOT(input_descriptor, input_descriptor_2, correspondences);

        cout << "Try3: Is:" << endl;
        cout <<  input_descriptor->size() << " = " << input_keypoints->size() << endl;
    }

    else if (QString::compare(ui->comboBox_DescriptorType->currentText(), QString("SHOTRGB"), Qt::CaseInsensitive) == 0)
    {
        pcl::PointCloud<pcl::SHOT1344>::Ptr input_descriptor ( new pcl::PointCloud<pcl::SHOT1344>);
        pcl::PointCloud<pcl::SHOT1344>::Ptr input_descriptor_2 ( new pcl::PointCloud<pcl::SHOT1344>);

        DescribeCSHOT(input, input_keypoints, input_descriptor);
        DescribeCSHOT(input_2, input_keypoints_2, input_descriptor_2);

        FindCorrespondenceCSHOT(input_descriptor, input_descriptor_2, correspondences);

        cout << "Try3: Is:" << endl;
        cout <<  input_descriptor->size() << " = " << input_keypoints->size() << endl;
    }



    cout << "correspondences, before the CorrespondenceRejectorSampleConsensus, has size= " << correspondences->size() << endl;

    for (int i = 0; i < correspondences->size(); ++i)
    {
        if (i > 500)
            continue;

        PointT scene_point = input_keypoints->at(correspondences->at(i).index_match);
        PointT model_point = input_keypoints_2->at(correspondences->at(i).index_query);

        viewer->addLine<PointT, PointT> (model_point, scene_point, 0,0,0, QString("ALL_LINES_%1").arg(i).toStdString());
    }



 //   pcl::registration::CorrespondenceRejectorSampleConsensus


    // Sample Consensus Rejector in modo da rendere robuste le corrispondenze
    double corr_rejection_Threshold = 0.3;
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGBNormal> rejector;
    rejector.setInputSource (input_keypoints_2);
    rejector.setInputTarget (input_keypoints);
    rejector.setInlierThreshold(corr_rejection_Threshold);
    rejector.setInputCorrespondences(correspondences);
    rejector.getCorrespondences(*correspondences);
    cout << "correspondences, after the CorrespondenceRejectorSampleConsensus, has size= " << correspondences->size() << endl;



    for (int i = 0; i < correspondences->size(); ++i)
    {
        if (i > 500)
            continue;

        PointT scene_point = input_keypoints->at(correspondences->at(i).index_match);
        PointT model_point = input_keypoints_2->at(correspondences->at(i).index_query);

        viewer->addLine<PointT, PointT> (model_point, scene_point, 255,0,0, QString("GOOD_LINES_%1").arg(i).toStdString());
    }



    // INITIAL ALIGN
    Eigen::Matrix4f initial_transformation_matrix_;
    cout << "initial alignment..." << std::endl;


    pcl::registration::TransformationEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr transformation_estimation ;
           // (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>);

    if (ui->Registration_SVD->isChecked())
      transformation_estimation.reset(new pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>);
    if (ui->Registration_TranslationOnly->isChecked())
      transformation_estimation.reset(new pcl::registration::TransformationEstimationTranslation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>);


    transformation_estimation->estimateRigidTransformation (*input_keypoints_2, *input_keypoints, *correspondences, initial_transformation_matrix_);









    PointCloudT::Ptr output_2 (new PointCloudT);
    pcl::copyPointCloud(*CurrentPCList.at(ui->comboBox_PC_to_Align->currentIndex()), *output_2);


    Eigen::Matrix4f currentPose = Eigen::Matrix4f::Identity();
    currentPose.block(0,0,3,3) = CurrentPCList.at(ui->comboBox_PC_to_Align->currentIndex())->sensor_orientation_.matrix();
    currentPose.block(0,3,3,1) = CurrentPCList.at(ui->comboBox_PC_to_Align->currentIndex())->sensor_origin_.head(3);


    initial_transformation_matrix_ = initial_transformation_matrix_ * currentPose;

    output_2->sensor_origin_ = initial_transformation_matrix_.block<4,1>(0,3);
    output_2->sensor_orientation_ = Eigen::Quaternionf(initial_transformation_matrix_.block<3,3>(0,0));

    emit replace(CurrentPCList.at(ui->comboBox_PC_to_Align->currentIndex()), output_2, QString("Register KP"));
}

void Registration_Keypoints::on_pushButton_Elaboration_All_PC_clicked()
{
/*    QList<pcl::PointCloud<pcl::PFHSignature125>::Ptr>  FeaturesPFHList;
    QList<pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr>  FeaturesPFHRGBList;
    QList<pcl::PointCloud<pcl::FPFHSignature33>::Ptr>  FeaturesFPFHList;
    QList<pcl::PointCloud<pcl::SHOT352>::Ptr>  FeaturesSHOTList;
    QList<pcl::PointCloud<pcl::SHOT1344>::Ptr>  FeaturesCSHOTList;

    QList<PointCloudT::Ptr>  KeypointsList;

    int param_thresholt = ui->spinBox_threshold_Detector->value();

    int num_tot= CurrentPCList.size();

    for (int i=0; i< CurrentPCList.size();i++)
    {
        // INPUT
        PointCloudT::Ptr input(new PointCloudT);
        input = CurrentPCList.at(i);

        cout << "START ELABORATION  " << i+1 << "/" << num_tot<< endl;

        // DETECTION Keypoint
        PointCloudT::Ptr input_keypoints (new PointCloudT);

        if ( QString::compare(ui->comboBox_FeaturesType->currentText(), QString("SIFT3D"), Qt::CaseInsensitive) == 0)
        {
            DetectSIFT(input, input_keypoints);
        }

        if ( QString::compare(ui->comboBox_FeaturesType->currentText(), QString("AGAST"), Qt::CaseInsensitive) == 0)
        {
            DetectAGAST(input, input_keypoints,param_thresholt);
        }

        if ( QString::compare(ui->comboBox_FeaturesType->currentText(), QString("BRISK"), Qt::CaseInsensitive) == 0)
        {
            DetectBRISK(input, input_keypoints, param_thresholt,4);
        }

        // FEATURES DESCRIPTION

        PointCloudT::Ptr input_keypoints(new PointCloudT);
        PointCloudT::Ptr input_noNAN(new PointCloudT);

        if (QString::compare(ui->comboBox_DescriptorType->currentText(), QString("FPFH"), Qt::CaseInsensitive) == 0)
        {

            pcl::PointCloud<pcl::FPFHSignature33>::Ptr input_descriptor ( new pcl::PointCloud<pcl::FPFHSignature33>);
            DescribeFPFH(input, input_keypoints, input_descriptor, input_noNAN, input_keypoints_noNAN);
            FeaturesFPFHList.append(input_descriptor);
            KeypointsList.append(input_keypoints_noNAN);
        }

        else if (QString::compare(ui->comboBox_DescriptorType->currentText(), QString("PFH"), Qt::CaseInsensitive) == 0)
        {
            pcl::PointCloud<pcl::PFHSignature125>::Ptr input_descriptor ( new pcl::PointCloud<pcl::PFHSignature125>);
            DescribePFH(input, input_keypoints, input_descriptor, input_noNAN, input_keypoints_noNAN);
            FeaturesPFHList.append(input_descriptor);
            KeypointsList.append(input_keypoints_noNAN);
        }


        else if (QString::compare(ui->comboBox_DescriptorType->currentText(), QString("PFHRGB"), Qt::CaseInsensitive) == 0)
        {
            pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr input_descriptor ( new pcl::PointCloud<pcl::PFHRGBSignature250>);
            DescribePFHRGB(input, input_keypoints, input_descriptor, input_noNAN, input_keypoints_noNAN);
            FeaturesPFHRGBList.append(input_descriptor);
            KeypointsList.append(input_keypoints_noNAN);
        }

        else if (QString::compare(ui->comboBox_DescriptorType->currentText(), QString("SHOT"), Qt::CaseInsensitive) == 0)
        {
            pcl::PointCloud<pcl::SHOT352>::Ptr input_descriptor ( new pcl::PointCloud<pcl::SHOT352>);
            DescribeSHOT(input, input_keypoints, input_descriptor, input_noNAN, input_keypoints_noNAN);
            FeaturesSHOTList.append(input_descriptor);
            KeypointsList.append(input_keypoints_noNAN);
        }


        if (QString::compare(ui->comboBox_DescriptorType->currentText(), QString("SHOTRGB"), Qt::CaseInsensitive) == 0)
        {
            pcl::PointCloud<pcl::SHOT1344>::Ptr input_descriptor ( new pcl::PointCloud<pcl::SHOT1344>);
            DescribeCSHOT(input, input_keypoints, input_descriptor, input_noNAN, input_keypoints_noNAN);
            FeaturesCSHOTList.append(input_descriptor);
            KeypointsList.append(input_keypoints_noNAN);
        }
    }



    // CORRESPONDENCE

    Eigen::Matrix4f abs_transformation_matrix=Eigen::Matrix4f::Identity();

    for (int i=1; i < CurrentPCList.size();i++)
    {
        cout << "START ALIGN " << i << "/" << num_tot-1<< endl;

        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);

        if (QString::compare(ui->comboBox_DescriptorType->currentText(), QString("FPFH"), Qt::CaseInsensitive) == 0)
        {
            FindCorrespondenceFPFH(FeaturesFPFHList.at(i-1), FeaturesFPFHList.at(i), correspondences);
        }

        else if (QString::compare(ui->comboBox_DescriptorType->currentText(), QString("PFH"), Qt::CaseInsensitive) == 0)
        {
            FindCorrespondencePFH(FeaturesPFHList.at(i-1), FeaturesPFHList.at(i), correspondences);
        }


        else if (QString::compare(ui->comboBox_DescriptorType->currentText(), QString("PFHRGB"), Qt::CaseInsensitive) == 0)
        {
            FindCorrespondencePFHRGB(FeaturesPFHRGBList.at(i-1), FeaturesPFHRGBList.at(i), correspondences);
        }

        else if (QString::compare(ui->comboBox_DescriptorType->currentText(), QString("SHOT"), Qt::CaseInsensitive) == 0)
        {
            FindCorrespondenceSHOT(FeaturesSHOTList.at(i-1), FeaturesSHOTList.at(i), correspondences);
        }


        if (QString::compare(ui->comboBox_DescriptorType->currentText(), QString("SHOTRGB"), Qt::CaseInsensitive) == 0)
        {
            FindCorrespondenceCSHOT(FeaturesCSHOTList.at(i-1), FeaturesCSHOTList.at(i), correspondences);
        }


        // Sample Consensus Rejector in modo da rendere robuste le corrispondenze
        double corr_rejection_Threshold = 0.3;
        pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGBNormal> rejector;
        rejector.setInputSource (KeypointsList.at(i));
        rejector.setInputTarget (KeypointsList.at(i-1));
        rejector.setInlierThreshold(corr_rejection_Threshold);
        rejector.setInputCorrespondences(correspondences);
        rejector.getCorrespondences(*correspondences);
        cout << "correspondences, after the CorrespondenceRejectorSampleConsensus, has size= " << correspondences->size() << endl;

        // INITIAL ALIGN
        Eigen::Matrix4f initial_transformation_matrix_;
        cout << "initial alignment..." ;

        pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr transformation_estimation (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>);

        transformation_estimation->estimateRigidTransformation (*KeypointsList.at(i), *KeypointsList.at(i-1), *correspondences, initial_transformation_matrix_);

        abs_transformation_matrix=initial_transformation_matrix_*abs_transformation_matrix;

        cout << "DONE"<< endl;
        // CurrentPCList.at(i)->sensor_origin_ = abs_transformation_matrix.block<4,1>(0,3);
        //  CurrentPCList.at(i)->sensor_orientation_ = Eigen::Quaternionf(abs_transformation_matrix.block<3,3>(0,0));

        PointCloudT::Ptr output_2 (new PointCloudT);
        pcl::copyPointCloud(*CurrentPCList.at(i), *output_2);

        output_2->sensor_origin_ = abs_transformation_matrix.block<4,1>(0,3);
        output_2->sensor_orientation_ = Eigen::Quaternionf(abs_transformation_matrix.block<3,3>(0,0));

        emit replace(CurrentPCList.at(i), output_2, QString("Register"));
    }
*/}


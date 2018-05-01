#include "description_keypoints.h"
#include "ui_description_keypoints.h"

Description_Keypoints::Description_Keypoints(QWidget *parent) :
    QWidget(parent), ui(new Ui::Description_Keypoints)
{
    ui->setupUi(this);
}

Description_Keypoints::~Description_Keypoints()
{
    delete ui;
}


using namespace std;





//********************//
//***  DESCRIPTOR  ***//
//********************//
void Description_Keypoints::DescribeFPFH(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptor)
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

void Description_Keypoints::DescribePFH(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptor)
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

void Description_Keypoints::DescribePFHRGB(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descriptor)
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

void Description_Keypoints::DescribeSHOT(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::SHOT352>::Ptr descriptor)
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

void Description_Keypoints::DescribeCSHOT(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::SHOT1344>::Ptr descriptor )
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


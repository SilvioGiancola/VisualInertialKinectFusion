#include <iostream>

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/brisk_2d.h>
#include <pcl/features/brisk_2d.h>
#include <pcl/registration/correspondence_estimation.h>

typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef pcl::PointWithScale KeyPointT;
typedef pcl::PointCloud<KeyPointT> KeyPointCloudT;

typedef pcl::BRISKSignature512 DescriptorT;
typedef pcl::PointCloud<DescriptorT> DescriptorCloudT;

int main(int argc, char *argv[])
{    

    // Open point clouds
    std::string Source_path, Target_path;
    pcl::console::parse(argc, argv, "-source", Source_path);
    pcl::console::parse(argc, argv, "-target", Target_path);

    PointCloudT::Ptr Source(new PointCloudT);
    PointCloudT::Ptr Target(new PointCloudT);

    pcl::io::loadPCDFile(Source_path, *Source);
    pcl::io::loadPCDFile(Target_path, *Target);

    std::cout << "Target points number : " << Target->size() << std::endl;
    std::cout << "Source points number : " << Source->size() << std::endl;





    // Keypoint Detection
    pcl::BriskKeypoint2D<PointT> brisk_keypoint_estimation;
    brisk_keypoint_estimation.setThreshold (60);
    brisk_keypoint_estimation.setOctaves (4);

    // Source Cloud
    KeyPointCloudT::Ptr Source_keypoints (new KeyPointCloudT);
    brisk_keypoint_estimation.setInputCloud (Source);
    brisk_keypoint_estimation.compute(*Source_keypoints);

    // Target Cloud
    KeyPointCloudT::Ptr Target_keypoints (new KeyPointCloudT);
    brisk_keypoint_estimation.setInputCloud (Target);
    brisk_keypoint_estimation.compute (*Target_keypoints);


    std::cout << "Target keypoints number : " << Target_keypoints->size() << std::endl;
    std::cout << "Source keypoints number : " << Source_keypoints->size() << std::endl;






    // Keypoint Description
    pcl::BRISK2DEstimation<PointT> brisk_descriptor_estimation;

    // Source Cloud
    DescriptorCloudT::Ptr Source_descriptors (new DescriptorCloudT);
    brisk_descriptor_estimation.setInputCloud (Source);
    brisk_descriptor_estimation.setKeypoints (Source_keypoints);
    brisk_descriptor_estimation.compute (*Source_descriptors);

    // Target Cloud
    DescriptorCloudT::Ptr Target_descriptors (new DescriptorCloudT);
    brisk_descriptor_estimation.setInputCloud (Target);
    brisk_descriptor_estimation.setKeypoints (Target_keypoints);
    brisk_descriptor_estimation.compute (*Target_descriptors);


    std::cout << "Target descriptor number : " << Target_descriptors->size() << std::endl;
    std::cout << "Source descriptor number : " << Source_descriptors->size() << std::endl;





    // Correspondences matching
    pcl::registration::CorrespondenceEstimation<DescriptorT, DescriptorT> correspondence_estimation;

    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    correspondence_estimation.setInputSource (Source_descriptors);
    correspondence_estimation.setInputTarget (Target_descriptors);
    correspondence_estimation.determineCorrespondences (*correspondences);


    std::cout << "Correspondences found : " << correspondences->size() << std::endl;





    return 0;
}

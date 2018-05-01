#ifndef Description_Keypoints_H
#define Description_Keypoints_H

#include <QWidget>
#include <define.h>

#include <pcl/visualization/pcl_plotter.h>
//ICP
#include <pcl/correspondence.h>

#include <pcl/registration/icp.h> //RegistrationICP
#include <pcl/registration/icp_nl.h> //RegistrationICP
#include <pcl/registration/gicp.h> //RegistrationICP

#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/sample_consensus/sac_model_registration_translation.h>    // My RANSAC Registration


#include <pcl/keypoints/brisk_2d.h>
#include <pcl/keypoints/agast_2d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/brisk_2d.h>



#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/filter.h>

#include <pcl/features/pfhrgb.h>
#include <pcl/features/shot.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/pfh.h>

#include <QElapsedTimer>
#include <QFile>
#include <QFileDialog>
#include <QTextStream>
#include <QString>
#include <QTime>


#include <iostream>
#include <algorithm>
#include <vector>
#include <iterator>
#include <memory>


#include <pcl/common/transforms.h>  // transform point cloud
#include <pcl/common/io.h> // copy point cloud
#include <pcl/common/time.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>

#include <pcl/PCLPointCloud2.h>


namespace Ui {
class Description_Keypoints;
}


class Description_Keypoints : public QWidget
{
    Q_OBJECT

public:
    explicit Description_Keypoints(QWidget *parent = 0);
    ~Description_Keypoints();



    template <class Descriptor>
    Descriptor describe (PointCloudT::Ptr input, PointCloudT::Ptr keypoints)
    {
        Descriptor result;
        if (std::is_same<Descriptor, pcl::FPFHSignature33>::value)          DescribeFPFH( input, keypoints, result);
        else if (std::is_same<Descriptor, pcl::PFHSignature125>::value)     DescribePFH( input, keypoints, result);
        else if (std::is_same<Descriptor, pcl::PFHRGBSignature250>::value)  DescribePFHRGB( input, keypoints, result);
        else if (std::is_same<Descriptor, pcl::SHOT352>::value)             DescribeSHOT( input, keypoints, result);
        else if (std::is_same<Descriptor, pcl::SHOT1344>::value)            DescribeCSHOT( input, keypoints, result);

        return result;
    }

public slots:

    // DESCRIPTION
    void DescribeFPFH(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptor);
    void DescribePFH(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptor);
    void DescribePFHRGB(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descriptor);
    void DescribeSHOT(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::SHOT352>::Ptr descriptor);
    void DescribeCSHOT(PointCloudT::Ptr input,PointCloudT::Ptr keypoints, pcl::PointCloud<pcl::SHOT1344>::Ptr descriptor);







private:
    Ui::Description_Keypoints *ui;


};
#endif // Description_Keypoints_H

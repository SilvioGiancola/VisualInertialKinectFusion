#ifndef Filters_Downsampling_H
#define Filters_Downsampling_H

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

#include <pcl/filters/conditional_removal.h>



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
class Filters_Downsampling;
}


class Filters_Downsampling : public QWidget
{
    Q_OBJECT

public:
    explicit Filters_Downsampling(QWidget *parent = 0);
    ~Filters_Downsampling();

    PointCloudT::Ptr downsample(PointCloudT::Ptr Input);



private slots:


    // DETECTION
    void DetectAGAST(PointCloudT::Ptr input, PointCloudT::Ptr output, int paramThreshold = 30);
    void DetectBRISK(PointCloudT::Ptr input, PointCloudT::Ptr output, int paramThreshold = 30, int octave = 4);
    void DetectBRISKQuad(PointCloudT::Ptr input, PointCloudT::Ptr output, int paramThreshold = 30, int octave = 4);
    void DetectSIFT(PointCloudT::Ptr input, PointCloudT::Ptr output);


private:
    Ui::Filters_Downsampling *ui;

};
#endif // Filters_Downsampling_H

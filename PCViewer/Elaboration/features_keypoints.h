#ifndef FEATURES_KEYPOINTS_H
#define FEATURES_KEYPOINTS_H

#include <QWidget>
#include <define.h>
#include <pcl/kdtree/kdtree_flann.h>


#include <pcl/filters/conditional_removal.h> // ROI
#include <pcl/filters/extract_indices.h>

namespace Ui {
class Features_Keypoints;
}


class Features_Keypoints : public QWidget
{
    Q_OBJECT

public:
    explicit Features_Keypoints(QWidget *parent = 0);
    ~Features_Keypoints();

    void DetectAGAST(PointCloudT::Ptr input, PointCloudT::Ptr output, int paramThreshold = 30);
    void DetectBRISK(PointCloudT::Ptr input, PointCloudT::Ptr output, int paramThreshold = 30, int octave = 4);
    void DetectBRISKQuad(PointCloudT::Ptr input, PointCloudT::Ptr output, int paramThreshold = 30, int octave = 4);
    void DetectSIFT(PointCloudT::Ptr input, PointCloudT::Ptr output);
    void DetectCurvature(PointCloudT::Ptr input, PointCloudT::Ptr output, double curvature);



public slots:
    void setCurrentPC(PointCloudT::Ptr pc) { CurrentPC = pc; }

signals:
    void replace(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC, QString ElabName);


private slots:
    void on_pushButton_Compute_clicked();

private:
    Ui::Features_Keypoints *ui;
    PointCloudT::Ptr CurrentPC;
};

#endif // FEATURES_KEYPOINTS_H

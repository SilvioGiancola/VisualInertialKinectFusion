#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <QWidget>
#include <define.h>
#include <pcl/common/transforms.h>  // transform point cloud
#include <pcl/common/io.h> // copy point cloud
#include <QUndoStack>


namespace Ui {
class Transformation;
}


class Transformation : public QWidget
{
    Q_OBJECT

public:
    explicit Transformation(QWidget *parent = 0);
    ~Transformation();


public slots:
    void setCurrentPC(PointCloudT::Ptr pc) { CurrentPC = pc; on_Transform_getMat_clicked();}
    void setCurrentPCList(QList<PointCloudT::Ptr> pcList) { CurrentPCList = pcList;}

signals:
    void replace(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC, QString ElabName);


private slots:
    void on_Transform_Rxneg_clicked();
    void on_Transform_Rxpos_clicked();
    void on_Transform_Ryneg_clicked();
    void on_Transform_Rypos_clicked();
    void on_Transform_Rzneg_clicked();
    void on_Transform_Rzpos_clicked();

    void on_Transform_Txneg_clicked();
    void on_Transform_Txpos_clicked();
    void on_Transform_Tyneg_clicked();
    void on_Transform_Typos_clicked();
    void on_Transform_Tzneg_clicked();
    void on_Transform_Tzpos_clicked();


    void on_Transform_getMat_clicked();
    void on_Transform_setMat_clicked();

    void on_Transform_reset_clicked();


    void on_Transform_solidify_clicked();

    void on_Transform_demean_clicked();

private:
    Ui::Transformation *ui;
    PointCloudT::Ptr CurrentPC;
    QList<PointCloudT::Ptr> CurrentPCList;

    void Rotate(PointCloudT::Ptr input, PointCloudT::Ptr output, Eigen::Quaternionf quaternion);
    void Translate(PointCloudT::Ptr input, PointCloudT::Ptr output, Eigen::Vector3f translation);

    void ApplyTransformation(PointCloudT::Ptr input, PointCloudT::Ptr output,  Eigen::Matrix4f transformationMatrix);
};

#endif // TRANSFORM_H

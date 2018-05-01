#include "registration_LUM.h"
#include "ui_registration_LUM.h"

Registration_LUM::Registration_LUM(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Registration_LUM)
{
    ui->setupUi(this);
    ui->myCorrespondenceRejection->on_Registration_Max_Dist_Correspondences_clicked(true);
    ui->myCorrespondenceRejection->on_Registration_OnetoOne_clicked(true);
}

Registration_LUM::~Registration_LUM()
{
    delete ui;
}

using namespace std;

void Registration_LUM::on_pushButton_compute_LUM_clicked()
{
    pcl::ScopeTime t("FULL LUM");

    if (CurrentPCList.size() < 2)
    {
        std::cout  << "Not enough Point Clouds" << std::endl;
        return;
    }


    pcl::registration::LUM<PointT> lum;

    // Fill LUM
    for (int i = 0; i < CurrentPCList.size(); i++)
    {
        PointCloudT::Ptr Model_rot(new PointCloudT);
        // Downsample the point clouds
        Model_rot = ui->myDownsampling->downsample(CurrentPCList.at(i));
        // Pre-align point clouds
        pcl::transformPointCloud(*Model_rot, *Model_rot, CurrentPCList.at(i)->sensor_origin_.head(3), CurrentPCList.at(i)->sensor_orientation_);
        // Add point clouds in LUM
        lum.addPointCloud(Model_rot);
    }



    // SET CORRESPONDNCES
    for (int i = 0; i < CurrentPCList.size(); i++)
    {
        int x = i%4;
        int y = (i - x) / 4;
     /*   qDebug() << "";
        qDebug() << "i="<<x;
        qDebug() << "x="<<x;
        qDebug() << "y="<<y;
        qDebug() << y<<"*4+"<<x << "=" <<i;

*/
        int j_min = i+1;
       // int j_max = min((y+2)*4, CurrentPCList.size());
        int j_max = CurrentPCList.size();
        // align all the column with the next one
        for (int j = j_min; j < j_max; j++ )
        {

            Eigen::Vector3f P1 = lum.getPointCloud(i)->sensor_origin_.head(3);
            Eigen::Vector3f P2 = lum.getPointCloud(j)->sensor_origin_.head(3);
            Eigen::Vector3f diff = P1 - P2;
            if (diff.norm() < ui->doubleSpinBox_maxDist->value())
            {

              //  qDebug() << i<<" with "<< j ;

                pcl::CorrespondencesPtr corr (new pcl::Correspondences);
                // Estimate Correspondences
                corr = ui->myCorrespondenceEstimation->getCorrespondences(lum.getPointCloud(i),lum.getPointCloud(j));
                // Reject Bad Correspondences
                corr = ui->myCorrespondenceRejection->filterCorrespondences(corr, lum.getPointCloud(i),lum.getPointCloud(j));

                //if (nbCorr > 20)
                // Set Correspondences in LUM
                lum.setCorrespondences(i, j, corr);
            }
        }

    }

    // Set LUM Parameters
    lum.setMaxIterations (ui->spinBox_LUM_Internal_Iteration->value());
    lum.setConvergenceThreshold (ui->doubleSpinBox_LUM_convergence_threshold->value());

    // Perform the actual LUM computation
    lum.compute ();


    // Return the single point cloud transformation
    for(int i = 0; i < CurrentPCList.size(); i++)
    {
        Eigen::Matrix4f res = lum.getTransformation(i).matrix();
        //std::cout << res << std::endl << std::endl;

        PointCloudT::Ptr output (new PointCloudT);
        PointCloudT::Ptr input (new PointCloudT);
        pcl::copyPointCloud(*CurrentPCList.at(i), *input);
        pcl::copyPointCloud(*CurrentPCList.at(i), *output);

        output->sensor_origin_ = res.block<4,1>(0,3) + output->sensor_origin_;
        output->sensor_orientation_ = Eigen::Quaternionf(res.block<3,3>(0,0)) * output->sensor_orientation_;

        // update Model
        emit replace( CurrentPCList.at(i), output, QString("LUM"));
    }

}



void Registration_LUM::on_pushButton_compute_ELCH_clicked()
{
    pcl::ScopeTime t("FULL ELCH");

    if (CurrentPCList.size() < 2)
    {
        std::cout  << "Not enough Point Clouds" << std::endl;
        return;
    }


    pcl::registration::ELCH<PointT> elch;

    pcl::IterativeClosestPoint<PointT, PointT>::Ptr icp (new pcl::IterativeClosestPoint<PointT, PointT>);
    icp->setMaximumIterations (10);
    icp->setMaxCorrespondenceDistance (0.2);
    icp->setRANSACOutlierRejectionThreshold (0.1);
    icp->setTransformationEstimation(ui->myTransformationEstimation->myTransformationEstimation);

    elch.setReg(icp);

    // Fill ELCH
    for (int i = 0; i < CurrentPCList.size(); i++)
    {
      /*  PointCloudT::Ptr Model_rot(new PointCloudT);
        // Downsample the point clouds
        Model_rot = ui->myDownsampling->downsample(CurrentPCList.at(i));
        // Pre-align point clouds
        pcl::transformPointCloud(*Model_rot, *Model_rot, CurrentPCList.at(i)->sensor_origin_.head(3), CurrentPCList.at(i)->sensor_orientation_);*/
        // Add point clouds in LUM
        elch.addPointCloud(CurrentPCList.at(i));
    }



    // SET CORRESPONDNCES
    for (int i = 0; i < CurrentPCList.size(); i++)
    {
        int x = i%4;
        int y = (i - x) / 4;
        qDebug() << "";
        qDebug() << "i="<<x;
        qDebug() << "x="<<x;
        qDebug() << "y="<<y;
        qDebug() << y<<"*4+"<<x << "=" <<i;


        int j_min = i+1;
        int j_max = min((y+2)*4, CurrentPCList.size());


        elch.setLoopStart (j_min);
        elch.setLoopEnd (j_max);
        elch.compute ();
       //  Eigen::Matrix4f res = elch.getLoopTransform()
        // align all the column with the next one
        /*  for (int j = j_min; j < j_max; j++ )
        {

            qDebug() << i<<" with "<< j ;

            pcl::CorrespondencesPtr corr (new pcl::Correspondences);
            // Estimate Correspondences
            corr = ui->myCorrespondenceEstimation->getCorrespondences(elch.getPointCloud(i),elch.getgetPointCloud(j));
            // Reject Bad Correspondences
            corr = ui->myCorrespondenceRejection->filterCorrespondences(corr, elch.getPointCloud(i),elch.getPointCloud(j));

            //if (nbCorr > 20)
            // Set Correspondences in LUM
            elch.sesetCorrespondences(i, j, corr);
        }*/

    }

    // Set LUM Parameters
    /*  lum.setMaxIterations (ui->spinBox_LUM_Internal_Iteration->value());
    lum.setConvergenceThreshold (ui->doubleSpinBox_LUM_convergence_threshold->value());

    // Perform the actual LUM computation
    lum.compute ();
*/
    //  elch.compute();

    // Return the single point cloud transformation
   /* for(int i = 0; i < CurrentPCList.size(); i++)
    {
      //  Eigen::Matrix4f res = elch.getLoopTransform().getTransformation(i).matrix();
        //std::cout << res << std::endl << std::endl;

        PointCloudT::Ptr output (new PointCloudT);
        PointCloudT::Ptr input (new PointCloudT);
        pcl::copyPointCloud(*CurrentPCList.at(i), *input);
        pcl::copyPointCloud(*CurrentPCList.at(i), *output);

        output->sensor_origin_ = res.block<4,1>(0,3) + output->sensor_origin_;
        output->sensor_orientation_ = Eigen::Quaternionf(res.block<3,3>(0,0)) * output->sensor_orientation_;

        // update Model
        emit replace( CurrentPCList.at(i), output, QString("LUM"));
    }*/
}

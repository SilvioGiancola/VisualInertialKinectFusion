#include "registration_correspondencerejection.h"
#include "ui_registration_correspondencerejection.h"

Registration_CorrespondenceRejection::Registration_CorrespondenceRejection(QWidget *parent) :
    QWidget(parent), ui(new Ui::Registration_CorrespondenceRejection)
{
    ui->setupUi(this);
   /* on_Registration_Max_Dist_Correspondences_clicked(ui->Registration_Max_Dist_Correspondences->isChecked());
    on_Registration_OnetoOne_clicked(ui->Registration_OnetoOne->isChecked());
    on_Registration_Trimmed_clicked(ui->Registration_Trimmed->isChecked());
    on_Registration_VarTrimmed_clicked(ui->Registration_VarTrimmed->isChecked());
    on_Registration_MedianDistance_clicked(ui->Registration_MedianDistance->isChecked());
    on_Registration_SampleConsensus_clicked(ui->Registration_SampleConsensus->isChecked());*/
}

Registration_CorrespondenceRejection::~Registration_CorrespondenceRejection()
{
    delete ui;
}


using namespace std;




void Registration_CorrespondenceRejection::AddCorrespondenceRejectorDistance(double MaximumDistance)
{
    pcl::registration::CorrespondenceRejectorDistance::Ptr cor_rej_dist (new pcl::registration::CorrespondenceRejectorDistance);
    cor_rej_dist->setMaximumDistance(MaximumDistance);
    correspondence_rejectors_.push_back(cor_rej_dist);
    ui->Registration_Max_Dist_Correspondences->setChecked(true);
}

void Registration_CorrespondenceRejection::AddCorrespondenceRejectorOneToOne()
{
    pcl::registration::CorrespondenceRejectorOneToOne::Ptr cor_rej_o2o (new pcl::registration::CorrespondenceRejectorOneToOne);
    correspondence_rejectors_.push_back(cor_rej_o2o);
    ui->Registration_OnetoOne->setChecked(true);
}

void Registration_CorrespondenceRejection::AddCorrespondenceRejectorTrimmed(double OverlapRatio)
{
    pcl::registration::CorrespondenceRejectorTrimmed::Ptr cor_rej_trimmed (new pcl::registration::CorrespondenceRejectorTrimmed);
    cor_rej_trimmed->setOverlapRatio(OverlapRatio);
    correspondence_rejectors_.push_back(cor_rej_trimmed);
    ui->Registration_Trimmed->setChecked(true);
}

void Registration_CorrespondenceRejection::AddCorrespondenceRejectorVarTrimmed()
{
    pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr cor_rej_trimmed (new pcl::registration::CorrespondenceRejectorVarTrimmed);
    correspondence_rejectors_.push_back(cor_rej_trimmed);
    ui->Registration_VarTrimmed->setChecked(true);
}

void Registration_CorrespondenceRejection::AddCorrespondenceRejectorMedianDistance(double MedianFactor)
{
    pcl::registration::CorrespondenceRejectorMedianDistance::Ptr cor_rej_med (new pcl::registration::CorrespondenceRejectorMedianDistance);
    cor_rej_med->setMedianFactor(MedianFactor);
    correspondence_rejectors_.push_back(cor_rej_med);
    ui->Registration_MedianDistance->setChecked(true);
}

void Registration_CorrespondenceRejection::AddCorrespondenceRejectorSampleConsensus(double InliersThreshold, int Iteration)
{
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::Ptr cor_rej_sac (new pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>);
    cor_rej_sac->setInlierThreshold (InliersThreshold);
    cor_rej_sac->setMaximumIterations (Iteration);
    correspondence_rejectors_.push_back(cor_rej_sac);
    ui->Registration_SampleConsensus->setChecked(true);
}







void Registration_CorrespondenceRejection::on_Registration_Max_Dist_Correspondences_clicked(bool checked)
{
    if (checked)
        AddCorrespondenceRejectorDistance(ui->doubleSpinBox_Max_Dist_Correspondences->value());
    else
        for ( int i = 0; i < correspondence_rejectors_.size(); i++ )
            if (correspondence_rejectors_.at(i)->getClassName() == "CorrespondenceRejectorDistance")
                correspondence_rejectors_.erase(correspondence_rejectors_.begin() + (i--));

}

void Registration_CorrespondenceRejection::on_Registration_OnetoOne_clicked(bool checked)
{
    if (checked)
        AddCorrespondenceRejectorOneToOne();

    else
        for ( int i = 0; i < correspondence_rejectors_.size(); i++ )
            if (correspondence_rejectors_.at(i)->getClassName() == "CorrespondenceRejectorOneToOne")
                correspondence_rejectors_.erase(correspondence_rejectors_.begin() + (i--));

}

void Registration_CorrespondenceRejection::on_Registration_Trimmed_clicked(bool checked)
{
    if (checked)
        AddCorrespondenceRejectorTrimmed(ui->doubleSpinBox_Trimmed->value());

    else
        for ( int i = 0; i < correspondence_rejectors_.size(); i++ )
            if (correspondence_rejectors_.at(i)->getClassName() == "CorrespondenceRejectorTrimmed")
                correspondence_rejectors_.erase(correspondence_rejectors_.begin() + (i--));

}

void Registration_CorrespondenceRejection::on_Registration_VarTrimmed_clicked(bool checked)
{
    if (checked)
        AddCorrespondenceRejectorVarTrimmed();

    else
        for ( int i = 0; i < correspondence_rejectors_.size(); i++ )
            if (correspondence_rejectors_.at(i)->getClassName() == "CorrespondenceRejectorVarTrimmed")
                correspondence_rejectors_.erase(correspondence_rejectors_.begin() + (i--));

}

void Registration_CorrespondenceRejection::on_Registration_MedianDistance_clicked(bool checked)
{
    if (checked)
        AddCorrespondenceRejectorMedianDistance(ui->Registration_MedianFactor->value());

    else
        for ( int i = 0; i < correspondence_rejectors_.size(); i++ )
            if (correspondence_rejectors_.at(i)->getClassName() == "CorrespondenceRejectorMedianDistance")
                correspondence_rejectors_.erase(correspondence_rejectors_.begin() + (i--));

}

void Registration_CorrespondenceRejection::on_Registration_SampleConsensus_clicked(bool checked)
{
    if (checked)
        AddCorrespondenceRejectorSampleConsensus(ui->Registration_SAC_InlierThreshold->value(), ui->Registration_SAC_MaxIteration->value());

    else
        for ( int i = 0; i < correspondence_rejectors_.size(); i++ )
            if (correspondence_rejectors_.at(i)->getClassName() == "CorrespondenceRejectorSampleConsensus")
                correspondence_rejectors_.erase(correspondence_rejectors_.begin() + (i--));

}






void Registration_CorrespondenceRejection::on_doubleSpinBox_Max_Dist_Correspondences_valueChanged(double arg)
{
    ui->Registration_Max_Dist_Correspondences->setChecked(true);
    bool presence = false;
    for ( int i = 0; i < correspondence_rejectors_.size(); i++ )
        if (correspondence_rejectors_.at(i)->getClassName() == "CorrespondenceRejectorDistance")
        {
            boost::static_pointer_cast<pcl::registration::CorrespondenceRejectorDistance>(correspondence_rejectors_.at(i))->setMaximumDistance(arg);
            presence = true;
        }

    if (!presence)
        AddCorrespondenceRejectorDistance(arg);

}

void Registration_CorrespondenceRejection::on_doubleSpinBox_Trimmed_valueChanged(double arg1)
{
    ui->Registration_Trimmed->setChecked(true);
    bool presence = false;
    for ( int i = 0; i < correspondence_rejectors_.size(); i++ )
        if (correspondence_rejectors_.at(i)->getClassName() == "CorrespondenceRejectorTrimmed")
        {
            boost::static_pointer_cast<pcl::registration::CorrespondenceRejectorTrimmed>(correspondence_rejectors_.at(i))->setOverlapRatio(arg1);
            presence = true;
        }
    if (!presence)
        AddCorrespondenceRejectorTrimmed(arg1);
}

void Registration_CorrespondenceRejection::on_Registration_MedianFactor_valueChanged(double arg1)
{
    ui->Registration_MedianDistance->setChecked(true);
    bool presence = false;
    for ( int i = 0; i < correspondence_rejectors_.size(); i++ )
        if (correspondence_rejectors_.at(i)->getClassName() == "CorrespondenceRejectorMedianDistance")
        {
            boost::static_pointer_cast<pcl::registration::CorrespondenceRejectorMedianDistance>(correspondence_rejectors_.at(i))->setMedianFactor(arg1);
            presence = true;
        }
    if (!presence)
        AddCorrespondenceRejectorMedianDistance(arg1);

}

void Registration_CorrespondenceRejection::on_Registration_SAC_MaxIteration_valueChanged(int arg)
{
    /*    for ( int i = 0; i < correspondence_rejectors_.size(); i++ )
        if (correspondence_rejectors_.at(i)->getClassName() == "CorrespondenceRejectorSampleConsensus")
            boost::static_pointer_cast<pcl::registration::CorrespondenceRejectorSampleConsensus>(correspondence_rejectors_.at(i))->setMaximumIterations(arg);
*/
}

void Registration_CorrespondenceRejection::on_Registration_SAC_InlierThreshold_valueChanged(double arg)
{
    /*   for ( int i = 0; i < correspondence_rejectors_.size(); i++ )
        if (correspondence_rejectors_.at(i)->getClassName() == "CorrespondenceRejectorSampleConsensus")
            boost::static_pointer_cast<pcl::registration::CorrespondenceRejectorSampleConsensus>(correspondence_rejectors_.at(i))->setInlierThreshold(arg);
*/
}






pcl::CorrespondencesPtr Registration_CorrespondenceRejection::filterCorrespondences(pcl::CorrespondencesPtr correspondences, pcl::PointCloud<PointT>::Ptr Source, pcl::PointCloud<PointT>::Ptr Target)
{
    pcl::ScopeTime t ("Reject Correspondences");

    pcl::CorrespondencesPtr correspondences_out(new pcl::Correspondences);
    *correspondences_out = *correspondences;

    // Correspondences rejection
    for (int i_rej = 0; i_rej < correspondence_rejectors_.size(); i_rej++)
    {


        if (correspondence_rejectors_[i_rej]->requiresSourcePoints())
        {
            pcl::PCLPointCloud2::Ptr Source2 (new pcl::PCLPointCloud2);
            pcl::toPCLPointCloud2 (*Source, *Source2);
            correspondence_rejectors_[i_rej]->setSourcePoints(Source2);
         //   correspondence_rejectors_[i_rej]->setInputSource<PointT>(Source);
        }

        if (correspondence_rejectors_[i_rej]->requiresTargetPoints())
        {
            pcl::PCLPointCloud2::Ptr Target2 (new pcl::PCLPointCloud2);
            pcl::toPCLPointCloud2 (*Target, *Target2);
            correspondence_rejectors_[i_rej]->setTargetPoints(Target2);
           // correspondence_rejectors_[i_rej]->setInputTarget<PointT>(Target);
        }



        correspondence_rejectors_[i_rej]->setInputCorrespondences (correspondences_out);
        correspondence_rejectors_[i_rej]->getCorrespondences (*correspondences_out);

    }

    return correspondences_out;
}





void Registration_CorrespondenceRejection::on_pushButton_GetList_clicked()
{
    for ( int i = 0; i < correspondence_rejectors_.size(); i++ )
        cout << correspondence_rejectors_.at(i)->getClassName() << endl;
    cout << endl;
}


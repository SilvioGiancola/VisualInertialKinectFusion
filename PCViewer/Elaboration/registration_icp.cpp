#include "registration_icp.h"
#include "ui_registration_icp.h"

Registration_ICP::Registration_ICP(QWidget *parent) :
    QWidget(parent), ui(new Ui::Registration_ICP)
{
    ui->setupUi(this);
    ui->myCorrespondenceEstimation->on_checkBox_Reciprok_clicked(true);
    ui->myCorrespondenceRejection->on_Registration_Max_Dist_Correspondences_clicked(true);
    ui->myCorrespondenceRejection->on_Registration_OnetoOne_clicked(true);
}

Registration_ICP::~Registration_ICP()
{
    delete ui;
}


using namespace std;



void Registration_ICP::on_pushButton_ApplyOldTranslation_clicked()
{
    QTime Total_time;
    Total_time.start();

    try
    {
        // INPUT
        PointCloudT::Ptr Model(new PointCloudT);
        pcl::copyPointCloud( *CurrentPCList.at(ui->comboBox_Model->currentIndex()), *Model);

        PointCloudT::Ptr PC(new PointCloudT);
        pcl::copyPointCloud( *CurrentPCList.at(ui->comboBox_PC->currentIndex()), *PC);


        std::cout << "Model " << Model->size() << std::endl;
        std::cout << "PC    " << PC->size() << std::endl;


        // Se PC non è mai stato rotata, scostera un sacco dal suo modello
        // Peccato Perche sono cosi vicini al inizio!
        // In quel caso origin e orientation sono nulli
        // In questo caso gli diamo quello del modello!
        PointCloudT::Ptr PC_tr(new PointCloudT);
        pcl::copyPointCloud( *PC, *PC_tr);
        PC_tr->sensor_origin_ = Model->sensor_origin_;



        // update Model
        emit replace( PC, PC_tr, QString("Apply Old T"));

        std::cout << "Everything implies : " << Total_time.elapsed() << " ms" << std::endl;

        return;

    }
    catch(pcl::PCLException ex)
    {
        std::cout << ex.detailedMessage() << std::endl;
    }
    catch(std::exception ex)
    {
        std::cout << ex.what() << std::endl;
    }

    return;
}

void Registration_ICP::on_pushButton_ApplyOldOrientation_clicked()
{
    QTime Total_time;
    Total_time.start();

    try
    {
        // INPUT
        PointCloudT::Ptr Model(new PointCloudT);
        pcl::copyPointCloud( *CurrentPCList.at(ui->comboBox_Model->currentIndex()), *Model);

        PointCloudT::Ptr PC(new PointCloudT);
        pcl::copyPointCloud( *CurrentPCList.at(ui->comboBox_PC->currentIndex()), *PC);


        std::cout << "Model " << Model->size() << std::endl;
        std::cout << "PC    " << PC->size() << std::endl;


        // Se PC non è mai stato rotata, scostera un sacco dal suo modello
        // Peccato Perche sono cosi vicini al inizio!
        // In quel caso origin e orientation sono nulli
        // In questo caso gli diamo quello del modello!
        PointCloudT::Ptr PC_tr(new PointCloudT);
        pcl::copyPointCloud( *PC, *PC_tr);
        PC_tr->sensor_orientation_ = Model->sensor_orientation_;



        // update Model
        emit replace( PC, PC_tr, QString("Apply Old T"));

        std::cout << "Everything implies : " << Total_time.elapsed() << " ms" << std::endl;

        return;

    }
    catch(pcl::PCLException ex)
    {
        std::cout << ex.detailedMessage() << std::endl;
    }
    catch(std::exception ex)
    {
        std::cout << ex.what() << std::endl;
    }

    return;
}


void Registration_ICP::on_pushButton_UseNextPair_clicked()
{
    if ( (ui->comboBox_PC->currentIndex() < ui->comboBox_PC->count() - 1) &&
         (ui->comboBox_Model->currentIndex() < ui->comboBox_Model->count() - 1) )
    {
        ui->comboBox_Model->setCurrentIndex(ui->comboBox_Model->currentIndex()+1);
        ui->comboBox_PC->setCurrentIndex(ui->comboBox_PC->currentIndex()+1);
    }
}

void Registration_ICP::updateList()
{
    QString ModelString = ui->comboBox_Model->currentText();
    QString PCString = ui->comboBox_PC->currentText();

    ui->comboBox_Model->clear();
    ui->comboBox_PC->clear();

    for (int i = 0; i < CurrentPCList.size(); i++)
    {
        QString PCname = QString::fromStdString(CurrentPCList.at(i)->header.frame_id);
        ui->comboBox_Model->addItem(PCname);
        ui->comboBox_PC->addItem(PCname);
    }


    // Qt 4
    for (int i = 0; i < CurrentPCList.size(); i++)
    {
        if (ui->comboBox_Model->itemText(i) == ModelString)
            ui->comboBox_Model->setCurrentIndex(i);
        if (ui->comboBox_PC->itemText(i) == PCString)
            ui->comboBox_PC->setCurrentIndex(i);
    }
    // Qt 5
    // ui->comboBox_Model->setCurrentText(ModelString);
    //  ui->comboBox_PC->setCurrentText(PCString);

    std::cout << "List Updated" << CurrentPCList.size() << std::endl;
    return;
}





// Align 2 Choosen Point Clouds
void Registration_ICP::on_Registration_Start_Elab_clicked()
{
    QTime Total_time;
    Total_time.start();

    try
    {
        int nbPerRow = ui->spinBox->value();
        int i = ui->comboBox_PC->currentIndex();
        // INPUT
        PointCloudT::Ptr Model(new PointCloudT);
        pcl::copyPointCloud( *CurrentPCList.at(ui->comboBox_Model->currentIndex()), *Model);

        PointCloudT::Ptr PC_init(new PointCloudT);
        pcl::copyPointCloud( *CurrentPCList.at(i), *PC_init);


        std::cout << "Model " << Model->size() << std::endl;
        std::cout << "PC    " << PC_init->size() << std::endl;


        // Se PC non è mai stato rotata, scostera un sacco dal suo modello
        // Peccato Perche sono cosi vicini al inizio!
        // In quel caso origin e orientation sono nulli
        // In questo caso gli diamo quello del modello!
        PointCloudT::Ptr PC(new PointCloudT);
        pcl::copyPointCloud( *PC_init, *PC);
        /*   if (ui->checkBox_UseOldTranslation->isChecked())
            PC_tr->sensor_origin_ = Model->sensor_origin_;
*/





        PointCloudT::Ptr KP_Model (new PointCloudT);
        PointCloudT::Ptr KP_PC (new PointCloudT);

        KP_PC = ui->myDownsampling->downsample(PC);
        KP_Model = ui->myDownsampling->downsample(Model);

        KP_PC->header.frame_id = PC->header.frame_id;

        std::cout << "Model Downsampling " << KP_Model->size() << std::endl;
        std::cout << "PC    Downsampling " << KP_PC->size() << std::endl;

        std::cout << "Pre-elaboration implies : " << Total_time.elapsed() << " ms" << std::endl;






        // Elaborate


        PointCloudT::Ptr fullPC (new PointCloudT);
        PointCloudT::Ptr fullModel (new PointCloudT);
        pcl::copyPointCloud( *PC, *fullPC);
        pcl::copyPointCloud( *Model, *fullModel);
        Eigen::Matrix4f res = DoICP(KP_PC, KP_Model, fullPC, fullModel);


        std::cout << "ICP implies : " << Total_time.elapsed() << " ms" << std::endl;







        // TO DO : Apply Transformation to all next x4 Point Cloud
        int i_final = i+nbPerRow;
        int passo = nbPerRow;
        if (ui->checkBox_Propagation->isChecked() )
            i_final = CurrentPCList.size();

        if (ui->checkBox_Lateral->isChecked())
            passo = 1;


        for (int i_PC_new = i ; i_PC_new < i_final ; i_PC_new = i_PC_new + passo)
        {
            //  qDebug () << "updatePose " << i_PC_new;
            //  int i_PC_new = i+nbPerRow;
            PointCloudT::Ptr PCnew(new PointCloudT);
            pcl::copyPointCloud( *CurrentPCList.at(i_PC_new), *PCnew);

            // Get results
            PointCloudT::Ptr output (new PointCloudT);
            pcl::copyPointCloud( *PCnew, *output);

            output->sensor_origin_ = res.block<4,1>(0,3) + output->sensor_origin_;
            output->sensor_orientation_ = Eigen::Quaternionf(res.block<3,3>(0,0)) * output->sensor_orientation_;

            //  CurrentPCList.replace(i_PC_new, output);
            // update Model
            emit replace( PCnew, output, QString("ICP"));
        }


        /*

        // Get results
        PointCloudT::Ptr output (new PointCloudT);
        pcl::copyPointCloud( *PC_init, *output);

        output->sensor_origin_ = res.block<4,1>(0,3) + output->sensor_origin_;
        output->sensor_orientation_ = Eigen::Quaternionf(res.block<3,3>(0,0)) * output->sensor_orientation_;

        // update Model
        emit replace( PC_init, output, QString("ICP"));
*/

        std::cout << "Everything implies : " << Total_time.elapsed() << " ms" << std::endl;

        return;

    }
    catch(pcl::PCLException ex)
    {
        std::cout << ex.detailedMessage() << std::endl;
    }
    catch(std::exception ex)
    {
        std::cout << ex.what() << std::endl;
    }

    return;

}

// Pairwise Align all the point cloud
void Registration_ICP::on_Registration_Start_Elab_All_clicked()
{
    try
    {
        for (int i = 1; i < CurrentPCList.size(); i++)
        {


            // INPUT
            PointCloudT::Ptr Model (new PointCloudT);
            pcl::copyPointCloud( *CurrentPCList.at(i-1), *Model);

            PointCloudT::Ptr PC (new PointCloudT);
            pcl::copyPointCloud( *CurrentPCList.at(i), *PC);


            // Se PC non è mai stato rotata, scostera un sacco dal suo modello
            // Peccato Perche sono cosi vicini al inizio!
            // In quel caso origin e orientation sono nulli
            // In questo caso gli diamo quello del modello!
            //Eigen::Vector3f::d
            if (PC->sensor_origin_.head(3).squaredNorm() < 0.0001)
                PC->sensor_origin_ = Model->sensor_origin_;


            std::cout << "Model " << Model->size() << std::endl;
            std::cout << "PC    " << PC->size() << std::endl;


            PointCloudT::Ptr KP_Model (new PointCloudT);
            PointCloudT::Ptr KP_PC (new PointCloudT);

            KP_PC = ui->myDownsampling->downsample(PC);
            KP_Model = ui->myDownsampling->downsample(Model);

            /*  if ( ui->DownSampling_RandomSampling->isChecked() )
            {
                float decimate_percent = ui->DownSampling_Random_Percent->value()/100.0;
                pcl::RandomSample<PointT> random_sampler;

                random_sampler.setInputCloud(Model);
                random_sampler.setSample((int) (decimate_percent*Model->points.size()));
                random_sampler.filter(*KP_Model);

                random_sampler.setInputCloud(PC);
                random_sampler.setSample((int) (decimate_percent*PC->points.size()));
                random_sampler.filter(*KP_PC);
            }

            else if ( ui->DownSampling_VoxelGrid->isChecked() )
            {
                pcl::VoxelGrid<PointT> sor;
                sor.setLeafSize (ui->DownSampling_VoxelGrid_Leaf->value(), ui->DownSampling_VoxelGrid_Leaf->value(), ui->DownSampling_VoxelGrid_Leaf->value());

                sor.setInputCloud (Model);
                sor.filter (*KP_Model);

                sor.setInputCloud (PC);
                sor.filter (*KP_PC);
            }

            else if ( ui->Keypoint_SIFT3D->isChecked() )
            {
                DetectSIFT(Model, KP_Model);
                DetectSIFT(PC,KP_PC);
            }

            else if ( ui->Keypoint_AGAST->isChecked() )
            {
                DetectAGAST(Model, KP_Model, ui->Keypoint_AGAST_Threshold->value());
                DetectAGAST(PC, KP_PC, ui->Keypoint_AGAST_Threshold->value());
            }

            else if ( ui->Keypoint_BRISK->isChecked() )
            {
                DetectBRISK(Model, KP_Model, ui->Keypoint_BRISK_Threshold->value(),4);
                DetectBRISK(PC, KP_PC, ui->Keypoint_BRISK_Threshold->value(),4);
            }
            else
            {
                pcl::copyPointCloud( *Model, *KP_Model );
                pcl::copyPointCloud( *PC, *KP_PC );
            }
*/

            std::cout << "Model Downsampling " << KP_Model->size() << std::endl;
            std::cout << "PC    Downsampling " << KP_PC->size() << std::endl;


            KP_PC->header.frame_id = PC->header.frame_id;




            PointCloudT::Ptr fullPC (new PointCloudT);
            PointCloudT::Ptr fullModel (new PointCloudT);
            pcl::copyPointCloud( *PC, *fullPC);
            pcl::copyPointCloud( *Model, *fullModel);
            // Elaborate
            Eigen::Matrix4f res = DoICP(KP_PC, KP_Model, fullPC, fullModel);





            // Get results
            PointCloudT::Ptr result (new PointCloudT);
            pcl::copyPointCloud(*PC, *result);

            result->sensor_origin_ = res.block<4,1>(0,3) + PC->sensor_origin_;
            result->sensor_orientation_ = Eigen::Quaternionf(res.block<3,3>(0,0)) * PC->sensor_orientation_;


            // update Model
            emit replace(PC, result, QString("ICP"));

        }
    }
    catch(pcl::PCLException ex)
    {
        std::cout << ex.detailedMessage() << std::endl;
    }
    catch(std::exception ex)
    {
        std::cout << ex.what() << std::endl;
    }

    return;
}


Eigen::Matrix4f Registration_ICP::DoICP(PointCloudT::Ptr PC, PointCloudT::Ptr Model, PointCloudT::Ptr FullPC, PointCloudT::Ptr FullModel)
{

    std::cout << PC->header.frame_id << std::endl;
    std::cout << Model->header.frame_id << std::endl;


    if (PC == NULL || Model == NULL)
        return Eigen::Matrix4f::Identity();


    // Alineo un PC rispetto a quello precedente nella lista
    //  PointCloudT::Ptr Model = PCList.at(index.row()-1);
    //  PointCloudT::Ptr _PC = PCList.at(index.row());




    // Trasformo i PC in funzione dei loro valori di origin e di orientation,
    //cosi valuto il loro scostamento uno rispetto all'altro
    // In pratica non muovo nulla !! Prealinea soltanto
    PointCloudT::Ptr Model_rot(new PointCloudT);
    PointCloudT::Ptr PC_rot(new PointCloudT);
    pcl::transformPointCloudWithNormals(*Model, *Model_rot, Model->sensor_origin_.head(3), Model->sensor_orientation_);
    pcl::transformPointCloudWithNormals(*PC, *PC_rot, PC->sensor_origin_.head(3), PC->sensor_orientation_);

    if (ui->checkBox_Metrics->isChecked())
    {
        pcl::transformPointCloudWithNormals(*FullPC, *FullPC, FullPC->sensor_origin_.head(3), FullPC->sensor_orientation_);
        pcl::transformPointCloudWithNormals(*FullModel, *FullModel, FullModel->sensor_origin_.head(3), FullModel->sensor_orientation_);
    }
    //  Model->sensor_origin_ =



    std::vector<int> ind;
    if (!PC->is_dense)
    {
        pcl::removeNaNFromPointCloud(*PC_rot, *PC_rot, ind);
        pcl::removeNaNNormalsFromPointCloud(*PC_rot, *PC_rot, ind);
    }

    if (!Model->is_dense)
    {
        pcl::removeNaNFromPointCloud(*Model_rot, *Model_rot, ind);
        pcl::removeNaNNormalsFromPointCloud(*Model_rot, *Model_rot, ind);
    }







    std::vector<double> Time_abs_List;
    std::vector<double> Fitness_RMS;
    std::vector<double> Fitness_Mean;
    std::vector<double> Fitness_Median;


    if (ui->checkBox_Times->isChecked())
        Time_abs_List.push_back( 0 );


    if (ui->checkBox_Metrics->isChecked())
    {

        std::vector<double> tot_dist = getDistances(FullPC, FullModel, QString("%1_Distances_0.txt").arg(PC_rot->header.frame_id.c_str()) );
        std::nth_element(tot_dist.begin(), tot_dist.begin() + tot_dist.size()/2, tot_dist.end());
        double median = tot_dist[tot_dist.size()/2];
        Fitness_Median.push_back( median );

        double mean = std::accumulate(tot_dist.begin(), tot_dist.end(), 0.0) / tot_dist.size();
        Fitness_Mean.push_back( mean );


        std::transform(tot_dist.begin(), tot_dist.end(), tot_dist.begin(), std::bind1st(std::multiplies<double>(),1000));
        double RMS = std::sqrt(  std::inner_product( tot_dist.begin(), tot_dist.end(), tot_dist.begin(), 0 ) / (static_cast<double>( tot_dist.size() ) * 1000000) );
        Fitness_RMS.push_back( RMS );

    }





    Eigen::Matrix4f final_transformation_ = Eigen::Matrix4f::Identity();


    int max_iteration = ui->Registration_NbIteration->value();
    int it = 0;

    while (it < max_iteration)
    {
        pcl::ScopeTime all_icp(" -> Single iteration");


        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);

        // Find Correspondences
        correspondences = ui->myCorrespondenceEstimation->getCorrespondences(PC_rot, Model_rot);

        // Filter Correspondences
        correspondences = ui->myCorrespondenceRejection->filterCorrespondences(correspondences, PC_rot, Model_rot);

        // Find Transformation
        Eigen::Matrix4f transformation_ = ui->myTransformationEstimation->getTransformation(PC_rot, Model_rot, correspondences);




        // Tranform the data
        pcl::transformPointCloudWithNormals (*PC_rot, *PC_rot, transformation_);


        // Obtain the final transformation
        final_transformation_ = transformation_ * final_transformation_;


        if (ui->checkBox_Times->isChecked())
            Time_abs_List.push_back(Time_abs_List.back() + all_icp.getTime());



        // Errors metrics
        if (ui->checkBox_Metrics->isChecked())
        {

            pcl::ScopeTime t ("metrics");

            pcl::transformPointCloudWithNormals (*FullPC, *FullPC, transformation_);


            std::vector<double> tot_dist_i = getDistances(FullPC, FullModel, QString("%1_Distances_%2.txt").arg(PC_rot->header.frame_id.c_str()).arg(it+1) );


            std::nth_element(tot_dist_i.begin(), tot_dist_i.begin() + tot_dist_i.size()/2, tot_dist_i.end());
            double median = tot_dist_i[tot_dist_i.size()/2];
            Fitness_Median.push_back(median);

            double mean = std::accumulate(tot_dist_i.begin(), tot_dist_i.end(), 0.0) / tot_dist_i.size();
            Fitness_Mean.push_back(mean);


            std::transform(tot_dist_i.begin(), tot_dist_i.end(), tot_dist_i.begin(), std::bind1st(std::multiplies<double>(),1000));
            double in = std::inner_product( tot_dist_i.begin(), tot_dist_i.end(), tot_dist_i.begin(), 0 );
            double inmean = in / (static_cast<double>( tot_dist_i.size() ) * 1000000);
            double RMS =  std::sqrt(  inmean );
            Fitness_RMS.push_back(RMS);

        }


        it++;

    }


    if (ui->checkBox_Times->isChecked())
    {
        QString fileName_result =  QString("%1_ResultICP.txt").arg(PC_rot->header.frame_id.c_str()) ;
        QFile fileError(fileName_result );
        fileError.open( QIODevice::WriteOnly );

        if (fileError.isOpen())
        {
            QTextStream streamError( &fileError );
            streamError.setRealNumberPrecision(20);
            for(int i = 0; i < Time_abs_List.size(); i++)
                if (ui->checkBox_Metrics->isChecked())
                    streamError << Time_abs_List.at(i) << "; " << Fitness_RMS.at(i) << "; " << Fitness_Mean.at(i) << "; " << Fitness_Median.at(i) << "; " << '\n';
                else
                    streamError << Time_abs_List.at(i) << '\n';

        }
        else
            std::cout << "Error: file is not opened" <<std::endl;

        fileError.close();
    }


    std::cout << final_transformation_ << std::endl;

    return final_transformation_;
}




void Registration_ICP::on_pushButton_Calc_Dist_clicked()
{
    try
    {

        PointCloudT::Ptr Model(CurrentPCList.at(ui->comboBox_Model->currentIndex()));
        PointCloudT::Ptr PC( CurrentPCList.at(ui->comboBox_PC->currentIndex()));

        PointCloudT::Ptr Model_rot(new PointCloudT);
        PointCloudT::Ptr PC_rot(new PointCloudT);
        pcl::transformPointCloudWithNormals(*Model, *Model_rot, Model->sensor_origin_.head(3), Model->sensor_orientation_);
        pcl::transformPointCloudWithNormals(*PC, *PC_rot, PC->sensor_origin_.head(3), PC->sensor_orientation_);

        std::vector<double> tot_dist = getDistances(PC_rot, Model_rot, QString::fromStdString("/home/silvio/distances.txt"));


        //  std::cout << "tot_dist is" << tot_dist.size() << std::endl;
        // vector<double>::iterator it = remove_if(tot_dist.begin(), tot_dist.end(), std::bind2nd(std::greater<double>(), ui->doubleSpinBox_DistThresh->value()));
        // tot_dist.erase (it, tot_dist.end());  // This is your required vector if you wish to use vector



        double sum = std::accumulate(tot_dist.begin(), tot_dist.end(), 0.0);
        double mean = sum / tot_dist.size();

        double sq_sum = std::inner_product(tot_dist.begin(), tot_dist.end(), tot_dist.begin(), 0.0);
        double stdev =  std::sqrt(sq_sum / tot_dist.size() - mean * mean);

        std::cout << "The mean is " << mean << '\n';

        std::nth_element(tot_dist.begin(), tot_dist.begin() + tot_dist.size()/2, tot_dist.end());
        double median = tot_dist[tot_dist.size()/2];
        std::cout << "The median is " << median << '\n';

        QString res = QString("median is : %1").arg(median);



        /*
        tot_dist.erase(std::remove_if(
                           tot_dist.begin(), tot_dist.end(), boost::bind(greater<int>(), _1, mean +3*stdev)),
                       tot_dist.end());
*/

        std::cout << "tot_dist is" << tot_dist.size() << std::endl;
        //defining a plotter
        pcl::visualization::PCLPlotter plotter;// = new pcl::visualization::PCLPlotter ();
        plotter.addHistogramData(tot_dist,tot_dist.size()/10);
        plotter.setTitle(res.toStdString().c_str());
        plotter.setXRange(0, ui->doubleSpinBox_DistThresh->value());
        plotter.setYRange(0, tot_dist.size()/2);
        plotter.setXTitle("distances");
        plotter.setYTitle("occurrences");
        plotter.plot ();


    }
    catch(pcl::PCLException ex)
    {
        std::cout << ex.detailedMessage() << std::endl;
    }
    catch(std::exception ex)
    {
        std::cout << ex.what() << std::endl;
    }

    return;
}


std::vector<double> Registration_ICP::getDistances(PointCloudT::Ptr PC, PointCloudT::Ptr Model, QString path)
{

    //  PointCloudT::Ptr Model_rot(new PointCloudT);
    //   PointCloudT::Ptr PC_rot(new PointCloudT);
    //   pcl::Model->sensor_origin_Model->sensor_origin_Model->sensor_origin_(*Model, *Model_rot, Model->sensor_origin_.head(3), Model->sensor_orientation_);
    //   pcl::transformPointCloudWithNormals(*PC, *PC_rot, PC->sensor_origin_.head(3), PC->sensor_orientation_);


    std::vector<int> indexes;
    pcl::removeNaNFromPointCloud(*Model, *Model, indexes);
    pcl::removeNaNFromPointCloud(*PC, *PC, indexes);

    Registration_CorrespondenceEstimation myCorr;
    pcl::CorrespondencesPtr corresp = myCorr.getCorrespondences(PC,  Model);


    // Registration_CorrespondenceRejection myCorrRej;
    //  myCorrRej.AddCorrespondenceRejectorOneToOne();
    //  corresp = myCorrRej.filterCorrespondences(corresp, PC, Model);

    std::vector<double> tot_dist;
    for (int i = 0; i < corresp->size(); i++)
        tot_dist.push_back(std::sqrt(corresp->at(i).distance));


    /*
    pcl::search::KdTree<PointT> search;

    search.setInputCloud(Model);

    for (int i = 0; i < PC->size(); i++)
    {
        // Return the correct index in the cloud instead of the index on the screen
        std::vector<int> indices (1);
        std::vector<float> distances (1);

        // Because VTK/OpenGL stores data without NaN, we lose the 1-1 correspondence, so we must search for the real point
        search.nearestKSearch(PC->points[i], 1, indices, distances);


        tot_dist.push_back( std::sqrt(distances[0]));
    }
*/

    // save in file
    if (!path.isEmpty())
    {
        QFile file( path );
        file.open( QIODevice::WriteOnly );

        if (file.isOpen())
        {
            QTextStream streamError( &file );
            streamError.setRealNumberPrecision(20);
            for(int i=0; i<tot_dist.size(); i++)
                streamError << tot_dist.at(i) << '\n';
        }
        else
            std::cout << "Error: file is not opened" <<std::endl;
        file.close();
    }


    return tot_dist;

}




void Registration_ICP::on_pushButton_KinFu_clicked()
{
    QTime Total_time;
    Total_time.start();

    try
    {
        // INPUT
        PointCloudT::Ptr Model(new PointCloudT);
        pcl::copyPointCloud( *CurrentPCList.at(ui->comboBox_Model->currentIndex()), *Model);

        PointCloudT::Ptr PC(new PointCloudT);
        pcl::copyPointCloud( *CurrentPCList.at(ui->comboBox_PC->currentIndex()), *PC);


        std::cout << "Model " << Model->size() << std::endl;
        std::cout << "PC    " << PC->size() << std::endl;


        // Se PC non è mai stato rotata, scostera un sacco dal suo modello
        // Peccato Perche sono cosi vicini al inizio!
        // In quel caso origin e orientation sono nulli
        // In questo caso gli diamo quello del modello!
        PointCloudT::Ptr PC_tr(new PointCloudT);
        pcl::copyPointCloud( *PC, *PC_tr);
        /*   if (ui->checkBox_UseOldTranslation->isChecked())
            PC_tr->sensor_origin_ = Model->sensor_origin_;
*/




        PointCloudT::Ptr KP_Model (new PointCloudT);
        PointCloudT::Ptr KP_PC (new PointCloudT);

        //  KP_Model = ui->myDownsampling->downsample(Model);
        //  KP_PC = ui->myDownsampling->downsample(PC_tr);

        pcl::copyPointCloud( *PC_tr, *KP_PC);
        pcl::copyPointCloud( *Model, *KP_Model);

        //  Downsample(Model, KP_Model);
        //  Downsample(PC_tr, KP_PC);
        KP_PC->header.frame_id = PC_tr->header.frame_id;

        std::cout << "Model Downsampling " << KP_Model->size() << std::endl;
        std::cout << "PC    Downsampling " << KP_PC->size() << std::endl;

        std::cout << "Pre-elaboration implies: " << Total_time.elapsed() << " ms" << std::endl;




        // Get results
        PointCloudT::Ptr fullPC (new PointCloudT);
        PointCloudT::Ptr fullModel (new PointCloudT);
        pcl::copyPointCloud( *PC, *fullPC);
        pcl::copyPointCloud( *Model, *fullModel);

        // Elaborate
        Eigen::Matrix4f res = DoICP(KP_PC, KP_Model, fullPC, fullModel);

        std::cout << "ICP implies : " << Total_time.elapsed() << " ms" << std::endl;







        // Get results
        PointCloudT::Ptr output (new PointCloudT);
        pcl::copyPointCloud( *PC, *output);

        output->sensor_origin_ = res.block<4,1>(0,3) + output->sensor_origin_;
        output->sensor_orientation_ = Eigen::Quaternionf(res.block<3,3>(0,0)) * output->sensor_orientation_;






        // update Model
        emit replace( PC, output, QString("ICP"));


        std::cout << "Everything implies : " << Total_time.elapsed() << " ms" << std::endl;

        return;

    }
    catch(pcl::PCLException ex)
    {
        std::cout << ex.detailedMessage() << std::endl;
    }
    catch(std::exception ex)
    {
        std::cout << ex.what() << std::endl;
    }

    return;
}

void Registration_ICP::on_Registration_Start_Elab_Ponte_clicked()
{


    QTime Total_time;
    Total_time.start();

    try
    {
        int nbPerRow = ui->spinBox->value();

        ui->checkBox_Propagation->setChecked(true);
        ui->spinBox->setValue(1);




        for (int i = 0; i < CurrentPCList.size() - nbPerRow; i++)
        {
            ui->comboBox_Model->setCurrentIndex(i);
            ui->comboBox_PC->setCurrentIndex(i+nbPerRow);
            for (int j = 0; j < 10; j++)
                on_Registration_Start_Elab_clicked();

            /*    qDebug() << "";
            qDebug() << i << " with "<< i+nbPerRow;
            // INPUT
            PointCloudT::Ptr Model(new PointCloudT);
            pcl::copyPointCloud( *CurrentPCList.at(i), *Model);

            PointCloudT::Ptr PC_init(new PointCloudT);
            pcl::copyPointCloud( *CurrentPCList.at(i+nbPerRow), *PC_init);


            qDebug() << "Model " << Model->size() ;
            qDebug() << "PC    " << PC_init->size() ;


            // Se PC non è mai stato rotata, scostera un sacco dal suo modello
            // Peccato Perche sono cosi vicini al inizio!
            // In quel caso origin e orientation sono nulli
            // In questo caso gli diamo quello del modello!
            PointCloudT::Ptr PC(new PointCloudT);
            pcl::copyPointCloud( *PC_init, *PC);






            PointCloudT::Ptr KP_Model (new PointCloudT);
            PointCloudT::Ptr KP_PC (new PointCloudT);

            KP_PC = ui->myDownsampling->downsample(PC);
            KP_Model = ui->myDownsampling->downsample(Model);

            KP_PC->header.frame_id = PC->header.frame_id;

            std::cout << "Model Downsampling " << KP_Model->size() << std::endl;
            std::cout << "PC    Downsampling " << KP_PC->size() << std::endl;

            std::cout << "Pre-elaboration implies : " << Total_time.elapsed() << " ms" << std::endl;






            // Elaborate


            PointCloudT::Ptr fullPC (new PointCloudT);
            PointCloudT::Ptr fullModel (new PointCloudT);
            pcl::copyPointCloud( *PC, *fullPC);
            pcl::copyPointCloud( *Model, *fullModel);
            Eigen::Matrix4f res = DoICP(KP_PC, KP_Model, fullPC, fullModel);


            std::cout << "ICP implies : " << Total_time.elapsed() << " ms" << std::endl;







            // TO DO : Apply Transformation to all next x4 Point Cloud
            int i_final = i+nbPerRow+1;
            if (ui->checkBox_Propagation->isChecked() )
                i_final = CurrentPCList.size();

            for (int i_PC_new = i+nbPerRow ; i_PC_new < i_final ; i_PC_new = i_PC_new + nbPerRow)
            {
                qDebug () << "updatePose " << i_PC_new;
                //  int i_PC_new = i+nbPerRow;
                PointCloudT::Ptr PCnew(new PointCloudT);
                pcl::copyPointCloud( *CurrentPCList.at(i_PC_new), *PCnew);

                // Get results
                PointCloudT::Ptr output (new PointCloudT);
                pcl::copyPointCloud( *PCnew, *output);

                output->sensor_origin_ = res.block<4,1>(0,3) + output->sensor_origin_;
                output->sensor_orientation_ = Eigen::Quaternionf(res.block<3,3>(0,0)) * output->sensor_orientation_;

                CurrentPCList.replace(i_PC_new, output);
                // update Model
                emit replace( PCnew, output, QString("ICP"));
            }

*/
        }
        std::cout << "Everything implies : " << Total_time.elapsed() << " ms" << std::endl;

        return;

    }
    catch(pcl::PCLException ex)
    {
        std::cout << ex.detailedMessage() << std::endl;
    }
    catch(std::exception ex)
    {
        std::cout << ex.what() << std::endl;
    }

    return;
}

void Registration_ICP::on_Registration_Start_Elab_PonteBis_clicked()
{

    QTime Total_time;
    Total_time.start();

    try
    {
        int nbPerRow = ui->spinBox->value();

        ui->checkBox_Propagation->setChecked(true);
        ui->checkBox_Lateral->setChecked(true);
        ui->spinBox->setValue(1);



        for (int i = 0; i < CurrentPCList.size() - nbPerRow; i=i+nbPerRow)
        {
            ui->comboBox_Model->setCurrentIndex(i);
            ui->comboBox_PC->setCurrentIndex(i+nbPerRow);
            for (int j = 0; j < 8; j++)
                on_Registration_Start_Elab_clicked();



        }
        std::cout << "Everything implies : " << Total_time.elapsed() << " ms" << std::endl;

        return;

    }
    catch(pcl::PCLException ex)
    {
        std::cout << ex.detailedMessage() << std::endl;
    }
    catch(std::exception ex)
    {
        std::cout << ex.what() << std::endl;
    }

    return;
}

#include "registration_ransac.h"
#include "ui_registration_ransac.h"

Registration_RANSAC::Registration_RANSAC(QWidget *parent) :
    QWidget(parent), ui(new Ui::Registration_RANSAC)
{
    ui->setupUi(this);
  //  ui->myCorrespondenceEstimation->on_checkBox_Reciprok_clicked(true);
  // ui->myCorrespondenceRejection->on_Registration_Max_Dist_Correspondences_clicked(true);
  //  ui->myCorrespondenceRejection->on_Registration_OnetoOne_clicked(true);
}

Registration_RANSAC::~Registration_RANSAC()
{
    delete ui;
}


using namespace std;



void Registration_RANSAC::on_pushButton_ApplyOldTranslation_clicked()
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

void Registration_RANSAC::on_pushButton_ApplyOldOrientation_clicked()
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


void Registration_RANSAC::on_pushButton_UseNextPair_clicked()
{
    if ( (ui->comboBox_PC->currentIndex() < ui->comboBox_PC->count() - 1) &&
         (ui->comboBox_Model->currentIndex() < ui->comboBox_Model->count() - 1) )
    {
        ui->comboBox_Model->setCurrentIndex(ui->comboBox_Model->currentIndex()+1);
        ui->comboBox_PC->setCurrentIndex(ui->comboBox_PC->currentIndex()+1);
    }
}

void Registration_RANSAC::updateList()
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




void Registration_RANSAC::on_pushButton_Calc_Dist_clicked()
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

        /*
        std::cout << "tot_dist is" << tot_dist.size() << std::endl;
        vector<double>::iterator it = remove_if(tot_dist.begin(), tot_dist.end(), std::bind2nd(std::greater<double>(), ui->doubleSpinBox_DistThresh->value()));
        tot_dist.erase (it, tot_dist.end());  // This is your required vector if you wish to use vector



        double sum = std::accumulate(tot_dist.begin(), tot_dist.end(), 0.0);
        double mean = sum / tot_dist.size();

        double sq_sum = std::inner_product(tot_dist.begin(), tot_dist.end(), tot_dist.begin(), 0.0);
        double stdev =  std::sqrt(sq_sum / tot_dist.size() - mean * mean);

        QString res = QString("mean distance is: %1  with std : %2").arg(mean).arg(stdev);

*/
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
        plotter.setYRange(0, tot_dist.size()/8);
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


std::vector<double> Registration_RANSAC::getDistances(PointCloudT::Ptr PC, PointCloudT::Ptr Model, QString path)
{

    //  PointCloudT::Ptr Model_rot(new PointCloudT);
    //   PointCloudT::Ptr PC_rot(new PointCloudT);
    //   pcl::Model->sensor_origin_Model->sensor_origin_Model->sensor_origin_(*Model, *Model_rot, Model->sensor_origin_.head(3), Model->sensor_orientation_);
    //   pcl::transformPointCloudWithNormals(*PC, *PC_rot, PC->sensor_origin_.head(3), PC->sensor_orientation_);


    std::vector<int> indexes;
    pcl::removeNaNFromPointCloud(*Model, *Model, indexes);
    pcl::removeNaNFromPointCloud(*PC, *PC, indexes);

    pcl::search::KdTree<PointT> search;

    search.setInputCloud(Model);

    std::vector<double> tot_dist;
    for (int i = 0; i < PC->size(); i++)
    {
        // Return the correct index in the cloud instead of the index on the screen
        std::vector<int> indices (1);
        std::vector<float> distances (1);

        // Because VTK/OpenGL stores data without NaN, we lose the 1-1 correspondence, so we must search for the real point
        search.nearestKSearch(PC->points[i], 1, indices, distances);


        tot_dist.push_back( std::sqrt(distances[0]));
    }


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







// Align with ransac
void Registration_RANSAC::on_pushButton_Registration_RANSAC_clicked()
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
        /*    if (ui->checkBox_UseOldTranslation->isChecked())
            PC_tr->sensor_origin_ = Model->sensor_origin_;
*/



        PointCloudT::Ptr KP_Model (new PointCloudT);
        PointCloudT::Ptr KP_PC (new PointCloudT);

        KP_Model = ui->myDownsampling->downsample(Model);
        KP_PC = ui->myDownsampling->downsample(PC);

        KP_PC->header.frame_id = PC_tr->header.frame_id;

        std::cout << "Model Downsampling " << KP_Model->size() << std::endl;
        std::cout << "PC    Downsampling " << KP_PC->size() << std::endl;

        std::cout << "Pre-elaboration implies: " << Total_time.elapsed() << " ms" << std::endl;




        // remove nan points
        std::vector<int> ind;
        if (!PC->is_dense)
        {
            pcl::removeNaNFromPointCloud(*KP_PC, *KP_PC, ind);
            pcl::removeNaNNormalsFromPointCloud(*KP_PC, *KP_PC, ind);
        }

        if (!Model->is_dense)
        {
            pcl::removeNaNFromPointCloud(*KP_Model, *KP_Model, ind);
            pcl::removeNaNNormalsFromPointCloud(*KP_Model, *KP_Model, ind);
        }




        // remove 0 point
        pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());
        range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, 0.1)));

        // build the filter
        pcl::ConditionalRemoval<PointT> condrem(true);
        condrem.setCondition(range_cond);

        condrem.setInputCloud (KP_PC);
        condrem.setKeepOrganized(KP_PC->isOrganized());
        condrem.filter (*KP_PC);

        condrem.setInputCloud (KP_Model);
        condrem.setKeepOrganized(KP_Model->isOrganized());
        condrem.filter (*KP_Model);






        // Trasformo i PC in funzione dei loro valori di origin e di orientation,
        //cosi valuto il loro scostamento uno rispetto all'altro
        // In pratica non muovo nulla !! Prealinea soltanto
        PointCloudT::Ptr KP_Model_rot(new PointCloudT);
        PointCloudT::Ptr KP_PC_rot(new PointCloudT);
        pcl::transformPointCloudWithNormals(*KP_Model, *KP_Model_rot, Model->sensor_origin_.head(3), Model->sensor_orientation_);
        pcl::transformPointCloudWithNormals(*KP_PC, *KP_PC_rot, PC->sensor_origin_.head(3), PC->sensor_orientation_);



        PointCloudT::Ptr Model_nonan (new PointCloudT);
        PointCloudT::Ptr PC_nonan (new PointCloudT);
        pcl::removeNaNFromPointCloud(*Model, *Model_nonan, ind);
        pcl::removeNaNNormalsFromPointCloud(*Model_nonan, *Model_nonan, ind);
        pcl::removeNaNFromPointCloud(*PC, *PC_nonan, ind);
        pcl::removeNaNNormalsFromPointCloud(*PC_nonan, *PC_nonan, ind);




        pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr Descr_Model (new pcl::PointCloud<pcl::PFHRGBSignature250>);
        pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr Descr_PC (new pcl::PointCloud<pcl::PFHRGBSignature250>);
        ui->myDescriptor->DescribePFHRGB(Model_nonan,KP_Model,Descr_Model);
        ui->myDescriptor->DescribePFHRGB(PC_nonan,KP_PC,Descr_PC);

      //  Descr_Model = ui->myDescriptor->describe<pcl::PFHRGBSignature250>(Model_nonan,KP_Model);
     //   Descr_PC = ui->myDescriptor->describe<pcl::PFHRGBSignature250>(PC_nonan,KP_PC);


        pcl::registration::CorrespondenceEstimationBase<pcl::PFHRGBSignature250, pcl::PFHRGBSignature250>::Ptr myCorrespondenceEstimation;
        myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimation<pcl::PFHRGBSignature250, pcl::PFHRGBSignature250>);
        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
        myCorrespondenceEstimation->setInputSource (Descr_PC);
        myCorrespondenceEstimation->setInputTarget (Descr_Model);
       if (ui->myCorrespondenceEstimation->_reciprok)
            myCorrespondenceEstimation->determineReciprocalCorrespondences (*correspondences);
        else
            myCorrespondenceEstimation->determineCorrespondences (*correspondences);



   /*     // CORRESPONDENCE ESTIMATION
        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
        correspondences = ui->myCorrespondenceEstimation->getCorrespondences(KP_PC_rot, KP_Model_rot);

        */
        std::cout << "correspondence size: " << correspondences->size()  << std::endl;


        // CORRESPONDENCE REJECTION
        correspondences = ui->myCorrespondenceRejection->filterCorrespondences(correspondences, KP_PC_rot, KP_Model_rot);
        std::cout << "correspondence size: " << correspondences->size()  << std::endl;






        //  pcl::registration::CorrespondenceRejectorOneToOne::Ptr cor_rej_o2o (new pcl::registration::CorrespondenceRejectorOneToOne);

        /*
        if (viewer->contains("corres"))
            viewer->removeCorrespondences("corres");
        viewer->addCorrespondences<PointT>(KP_PC, KP_Model, *correspondences, "corres");
*/

        std::vector<double> distances;
        for (int i = 0; i < correspondences->size(); ++i)
        {
            PointT& P2 = KP_Model->at (correspondences->at(i).index_match);
            PointT& P1 = KP_PC->at (correspondences->at(i).index_query);
            double distance = std::sqrt((P1.x-P2.x) * (P1.x-P2.x) + (P1.y-P2.y) *(P1.y-P2.y) + (P1.z-P2.z)*(P1.z-P2.z));
            distances.push_back( distance );
            std::cout << distance << std::endl;
        }

        std::sort(distances.begin(), distances.end());

        double median = distances[distances.size()/2];

        int g = 0, r = 0;

                pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences);
        for (int i = 0; i < correspondences->size(); ++i)
        {
            // QString nome= QString("line_%1").arg(i);
            std::stringstream ss_line;
            ss_line << "correspondence_line" << i;

            PointT& scene_point = KP_Model->at (correspondences->at(i).index_match);
            PointT& model_point = KP_PC->at (correspondences->at(i).index_query);

            double distance = std::sqrt((model_point.x-scene_point.x) * (model_point.x-scene_point.x) +
                                        (model_point.y-scene_point.y) * (model_point.y-scene_point.y) +
                                        (model_point.z-scene_point.z) * (model_point.z-scene_point.z));



            //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
            if (distance < median + 0.2 && distance > median - 0.2)
            {
                viewer->addLine<PointT, PointT> (model_point, scene_point, 0, 255, 0, ss_line.str ());
                viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, ss_line.str ());
                g++;
                correspondences_filtered->push_back(correspondences->at(i));
            }
            else
            {
                viewer->addLine<PointT, PointT> (model_point, scene_point, 255, 0, 0, ss_line.str ());
                viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, ss_line.str ());
                r++;
            }
        }
        std::cout << "good: " << g << std::endl;
        std::cout << "bad:  " << r << std::endl;
        /* for (int i = 0; i < correspondences->size(); ++i)
        {
            // QString nome= QString("line_%1").arg(i);
            std::stringstream ss_line;
            ss_line << "correspondence_line" << i;

            PointT& scene_point = KP_PC->at (correspondences->at(i).index_match);
            PointT& model_point = KP_Model->at (correspondences->at(i).index_query);

            //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
            if (correspondences->at(i).distance < median + 0.2 && correspondences->at(i).distance > median - 0.2)
                viewer->addLine<PointT, PointT> (model_point, scene_point, 0, 255, 0, ss_line.str ());
            else
                viewer->addLine<PointT, PointT> (model_point, scene_point, 255, 0, 0, ss_line.str ());

        }*/



        QTime t;
        t.start();

        std::vector<int> source_indices(correspondences_filtered->size());
        std::vector<int> target_indices(correspondences_filtered->size());
        for (int i = 0; i < (int)correspondences_filtered->size(); ++i)
        {
            source_indices[i] = correspondences_filtered->at(i).index_query;
            target_indices[i] = correspondences_filtered->at(i).index_match;
        }





        double inlierThreshold = ui->Registration_SAC_InlierThreshold->value();

        pcl::SampleConsensusModelRegistration<PointT>::Ptr model;
        std::vector<int> inliers;
        Eigen::VectorXf model_coefficients;
        Eigen::Matrix4f Transformation;

        {
            pcl::ScopeTime t("ransac-main");

            if (ui->Registration_TranslationOnly->isChecked())
            {
                std::cout << "using TransOnly" << std::endl;
                model.reset(new pcl::SampleConsensusModelRegistrationTranslation<PointT>(KP_PC_rot, source_indices ));
            }
            else
            {
                std::cout << "using SVD" << std::endl;
                model.reset(new pcl::SampleConsensusModelRegistration<PointT>(KP_PC_rot, source_indices ));
            }
            model->setInputTarget(KP_Model_rot, target_indices);


            pcl::RandomSampleConsensus<PointT> sac(model, inlierThreshold);


            sac.setMaxIterations(ui->Registration_SAC_MaxIteration->value());
            sac.computeModel();


            sac.getInliers(inliers);
            sac.getModelCoefficients (model_coefficients);
        }


        if (ui->checkBox_Refine->isChecked())
        {
            pcl::SampleConsensusModelRegistration<PointT>::Ptr newmodel = boost::static_pointer_cast<pcl::SampleConsensusModelRegistration<PointT> >(model);
            pcl::ScopeTime t("ransac-refine");
            // Refine Model
            double refineModelSigma = 3.0;
            double refineModelIterations = 5;

            double inlier_distance_threshold_sqr = inlierThreshold * inlierThreshold;
            double error_threshold = inlierThreshold;
            double sigma_sqr = refineModelSigma * refineModelSigma;
            int refine_iterations = 0;
            bool inlier_changed = false, oscillating = false;
            std::vector<int> new_inliers, prev_inliers = inliers;
            std::vector<size_t> inliers_sizes;
            do
            {
                // Optimize the model coefficients
                //  model.reset(new pcl::SampleConsensusModelRegistration<PointT>(KP_PC_rot, source_indices ));

                newmodel->optimizeModelCoefficients (prev_inliers, model_coefficients, model_coefficients);
                inliers_sizes.push_back (prev_inliers.size ());

                // Select the new inliers based on the optimized coefficients and new threshold
                newmodel->selectWithinDistance (model_coefficients, error_threshold, new_inliers);

                std::cout << QString("RANSAC refineModel: Number of inliers found (before/after): %1/%2, with an error threshold of %3.")
                             .arg((int)prev_inliers.size ()).arg((int)new_inliers.size ()).arg(error_threshold).toStdString() << std::endl;

                if (new_inliers.empty ())
                {
                    ++refine_iterations;
                    if (refine_iterations >= refineModelIterations)
                    {
                        break;
                    }
                    continue;
                }

                // Estimate the variance and the new threshold
                double variance = newmodel->computeVariance ();
                error_threshold =  std::sqrt (std::min (inlier_distance_threshold_sqr, sigma_sqr * variance));

                std::cout << QString("RANSAC refineModel: New estimated error threshold: %1 (variance=%2) on iteration %3 out of %4.")
                             .arg(error_threshold).arg(variance).arg(refine_iterations).arg(refineModelIterations).toStdString() << std::endl;

                //   UDEBUG ("RANSAC refineModel: New estimated error threshold: %f (variance=%f) on iteration %d out of %d.",
                //        error_threshold, variance, refine_iterations, refineModelIterations);
                inlier_changed = false;
                std::swap (prev_inliers, new_inliers);

                // If the number of inliers changed, then we are still optimizing
                if (new_inliers.size () != prev_inliers.size ())
                {
                    // Check if the number of inliers is oscillating in between two values
                    if (inliers_sizes.size () >= 4)
                    {
                        if (inliers_sizes[inliers_sizes.size () - 1] == inliers_sizes[inliers_sizes.size () - 3] &&
                                inliers_sizes[inliers_sizes.size () - 2] == inliers_sizes[inliers_sizes.size () - 4])
                        {
                            oscillating = true;
                            break;
                        }
                    }
                    inlier_changed = true;
                    continue;
                }

                // Check the values of the inlier set
                for (size_t i = 0; i < prev_inliers.size (); ++i)
                {
                    // If the value of the inliers changed, then we are still optimizing
                    if (prev_inliers[i] != new_inliers[i])
                    {
                        inlier_changed = true;
                        break;
                    }
                }
            }
            while (inlier_changed && ++refine_iterations < refineModelIterations);

            // If the new set of inliers is empty, we didn't do a good job refining
            if (new_inliers.empty ())
                std::cout << "RANSAC refineModel: Refinement failed: got an empty set of inliers!" << std::endl;


            if (oscillating)
                std::cout << "RANSAC refineModel: Detected oscillations in the model refinement." << std::endl;


            std::swap (inliers, new_inliers);
        }



        Transformation.row (0) = model_coefficients.segment<4>(0);
        Transformation.row (1) = model_coefficients.segment<4>(4);
        Transformation.row (2) = model_coefficients.segment<4>(8);
        Transformation.row (3) = model_coefficients.segment<4>(12);



        std::cout << Transformation << std::endl << "  -> in " << t.elapsed() << " ms" << std::endl;

        //Eigen::Matrix4f res = Eigen::Matrix4f::Identity();




        std::cout << "Everything implies : " << Total_time.elapsed() << " ms" << std::endl;

        // Get results
        PointCloudT::Ptr output (new PointCloudT);
        pcl::copyPointCloud( *PC, *output);


        output->sensor_origin_ = Transformation.block<4,1>(0,3) + output->sensor_origin_;
        output->sensor_orientation_ = Eigen::Quaternionf(Transformation.block<3,3>(0,0)) * output->sensor_orientation_;


        // update Model
        emit replace( PC, output, QString("ICP"));


        std::cout << "With Graphics : " << Total_time.elapsed() << " ms" << std::endl;


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



// Align with ransac
void Registration_RANSAC::on_pushButton_Registration_RANSAC3_clicked()
{
    QTime Total_time;
    Total_time.start();

    try
    {
        // INPUT
        PointCloudT::Ptr Target(new PointCloudT);
        pcl::copyPointCloud( *CurrentPCList.at(ui->comboBox_Model->currentIndex()), *Target);

        PointCloudT::Ptr Input(new PointCloudT);
        pcl::copyPointCloud( *CurrentPCList.at(ui->comboBox_PC->currentIndex()), *Input);


        std::cout << "Target" << Target->size() << std::endl;
        std::cout << "Input " << Input->size() << std::endl;


        // Se PC non è mai stato rotata, scostera un sacco dal suo modello
        // Peccato Perche sono cosi vicini al inizio!
        // In quel caso origin e orientation sono nulli
        // In questo caso gli diamo quello del modello!
        //  PointCloudT::Ptr PC_tr(new PointCloudT);
        //   pcl::copyPointCloud( *Input, *PC_tr);
        /*    if (ui->checkBox_UseOldTranslation->isChecked())
            PC_tr->sensor_origin_ = Target->sensor_origin_;
*/







        // Detection
        pcl::BriskKeypoint2D<PointT> brisk_keypoint_estimation;
        brisk_keypoint_estimation.setThreshold (60);
        brisk_keypoint_estimation.setOctaves (4);

        // Input Cloud
        pcl::PointCloud<pcl::PointWithScale>::Ptr Input_keypoints (new pcl::PointCloud<pcl::PointWithScale>);
        brisk_keypoint_estimation.setInputCloud (Input);
        brisk_keypoint_estimation.compute(*Input_keypoints);

        // Target Cloud
        pcl::PointCloud<pcl::PointWithScale>::Ptr Target_keypoints (new pcl::PointCloud<pcl::PointWithScale>);
        brisk_keypoint_estimation.setInputCloud (Target);
        brisk_keypoint_estimation.compute (*Target_keypoints);




        // Description
        pcl::BRISK2DEstimation<PointT> brisk_descriptor_estimation;

        // Input Cloud
        pcl::PointCloud<pcl::BRISKSignature512>::Ptr Input_descriptors (new pcl::PointCloud<pcl::BRISKSignature512>);
        brisk_descriptor_estimation.setInputCloud (Input);
        brisk_descriptor_estimation.setKeypoints (Input_keypoints);
        brisk_descriptor_estimation.compute (*Input_descriptors);

        // Target Cloud
        pcl::PointCloud<pcl::BRISKSignature512>::Ptr Target_descriptors (new pcl::PointCloud<pcl::BRISKSignature512>);
        brisk_descriptor_estimation.setInputCloud (Target);
        brisk_descriptor_estimation.setKeypoints (Target_keypoints);
        brisk_descriptor_estimation.compute (*Target_descriptors);




        // Correspondences
        pcl::registration::CorrespondenceEstimation<pcl::BRISKSignature512, pcl::BRISKSignature512> myCorrespondenceEstimation;

        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
        myCorrespondenceEstimation.setInputSource (Input_descriptors);
        myCorrespondenceEstimation.setInputTarget (Target_descriptors);
        myCorrespondenceEstimation.determineCorrespondences (*correspondences);


        std::cout << correspondences->size() << std::endl;







        std::cout << "Everything implies : " << Total_time.elapsed() << " ms" << std::endl;

        // Get results
        //  PointCloudT::Ptr output (new PointCloudT);
        //  pcl::copyPointCloud( *Input, *output);


        //   output->sensor_origin_ = Transformation.block<4,1>(0,3) + output->sensor_origin_;
        //   output->sensor_orientation_ = Eigen::Quaternionf(Transformation.block<3,3>(0,0)) * output->sensor_orientation_;


        // update Model
        //   emit replace( Input, output, QString("ICP"));


        //  std::cout << "With Graphics : " << Total_time.elapsed() << " ms" << std::endl;


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





#include "elaboration_lf_rec.h"
#include "ui_elaboration_lf_rec.h"

Elaboration_LF_Rec::Elaboration_LF_Rec(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Elaboration_LF_Rec)
{
    ui->setupUi(this);
}

Elaboration_LF_Rec::~Elaboration_LF_Rec()
{
    delete ui;
}

void Elaboration_LF_Rec::setViewer(pcl::visualization::PCLVisualizer::Ptr myViewer)
{
    viewer = myViewer;
    return;
}

void Elaboration_LF_Rec::updateList()
{
  //  SetMaxRes();
    SetMinDim();
 /*   QString ModelString = ui->comboBox_Model->currentText();
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
*/
    return;
}
void Elaboration_LF_Rec::SetMaxRes()
{
    double currentres = 0.0;
    int n_points = 0;
    int nres;
    maxres = 0.0;

    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<PointT> tree;

    for (int k = 0; k < PCList.size(); k++ )
    {
         tree.setInputCloud (PCList.at(k));
         for (size_t i = 0; i < PCList.at(k)->size (); ++i)
         {
           //Considering the second neighbor since the first is the point itself.
           nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
           if (nres == 2)
           {
             currentres += sqrt (sqr_distances[1]);
             ++n_points;
           }
         }
         if (n_points != 0)
         {
           currentres /= n_points;
         }
         if (currentres > maxres)
         {
             maxres = currentres;
         }

    }

    QString LabelMaxresText= QString("Max resolution: %1").arg(maxres);
    ui->label_maxres->setText(LabelMaxresText);

    return;
}

void Elaboration_LF_Rec::SetMinDim()
{
        PointT minPt, maxPt;
        double currentdim = 0.0;
        mindim = 0.0;
        for (int i = 0; i < PCList.size(); i++ )
        {
            pcl::getMinMax3D (*PCList.at(i), minPt, maxPt);
            currentdim = sqrt((maxPt.x-minPt.x)*(maxPt.x-minPt.x)+(maxPt.y-minPt.y)*(maxPt.y-minPt.y)+(maxPt.z-minPt.z)*(maxPt.z-minPt.z));
            if (currentdim < mindim || mindim == 0)
            {
                mindim=currentdim;
            }
        }

        QString LabelMindimText= QString("Min dimension: %1").arg(mindim);
        ui->label_mindim->setText(LabelMindimText);

        return;
}


void Elaboration_LF_Rec::on_Start_LF_Rec_clicked()
{
    std::cout << std::endl << "========================== Start Recognition =========================="  << std::endl;
    viewer -> removeAllPointClouds();
    viewer -> removeAllShapes();

    pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr model (new pcl::PointCloud<PointT>);
    scene_keysOUT.reset (new PointCloudT);
    model_keysOUT.reset (new PointCloudT);
    model_scene_corrs_OUT.reset(new pcl::Correspondences);
    Timer = new QElapsedTimer;

    scene = PCList.at(0);
    model = PCList.at(1);

    //
    //  Compute Normals
    //

    Timer->start();

    pcl::PointCloud<pcl::Normal>::Ptr model_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr scene_normals (new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimation<PointT, pcl::Normal> norm_est;
    norm_est.setKSearch (ui->set_k_search->value());
    norm_est.setInputCloud (PCList.at(0));
    norm_est.compute (*scene_normals);
    norm_est.setInputCloud (PCList.at(1));
    norm_est.compute (*model_normals);

    std::cout << "Normals computing time: " << Timer->elapsed() << " ms" << endl;

    //
    // Compute keypoints
    //

    Timer->restart();

    on_ComputeKeypoints_clicked();

    std::cout << "Keypoints computing time: " << Timer->elapsed() << " ms" << endl;

//   emit PCupdated();

    Timer->restart();

    if (ui->Descr_PFH->isChecked())
    {
        std::cout << "PFH descriptors.." <<std::endl;
        pcl::PFHEstimation<PointT, PointT, pcl::PFHSignature125> PFH_est;
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
        PFH_est.setSearchMethod (tree);
        PFH_est.setRadiusSearch (ui->Descr_radius->value());

        // scene descriptors

        pcl::PointCloud<pcl::PFHSignature125>::Ptr scene_descriptor (new  pcl::PointCloud<pcl::PFHSignature125>);
        PFH_est.setInputCloud(scene_keysOUT);
        PFH_est.setInputNormals(scene);
        PFH_est.setSearchSurface(scene);
        PFH_est.compute (*scene_descriptor);

        // model descriptors

        pcl::PointCloud<pcl::PFHSignature125>::Ptr model_descriptor (new  pcl::PointCloud<pcl::PFHSignature125>);
        PFH_est.setInputCloud(model_keysOUT);
        PFH_est.setInputNormals(model);
        PFH_est.setSearchSurface(model);
        PFH_est.compute (*model_descriptor);

        // Use all neighbors in a sphere of chosen radius
        // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!

        // visualization

        std::cout << "scene keys number: " << scene_keysOUT->size() << std::endl;
        std::cout << "scene descriptor number: " << scene_descriptor->size() << std::endl;
        viewer -> removePointCloud("scene_keysOUT");
        pcl::visualization::PointCloudColorHandlerCustom<PointT> scene_keysOUT_color_handler (scene_keysOUT, 0, 255, 0);
        viewer->addPointCloud (scene_keysOUT, scene_keysOUT_color_handler, "scene_keysOUT");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keysOUT");

        std::cout << "model keys number: " << model_keysOUT->size() << std::endl;
        std::cout << "model descriptor number: " << model_descriptor->size() << std::endl;
        viewer -> removePointCloud("model_keysOUT");
        pcl::visualization::PointCloudColorHandlerCustom<PointT> model_keysOUT_color_handler (model_keysOUT, 0, 255, 0);
        viewer->addPointCloud (model_keysOUT, model_keysOUT_color_handler, "model_keysOUT");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_keysOUT");

        // Elimino i NAN

        pcl::PointIndices::Ptr indices_NAN_scene (new pcl::PointIndices);
        pcl::PointIndices::Ptr indices_NAN_model (new pcl::PointIndices);
        *indices_NAN_scene = FindNANindx(scene_descriptor);
        *indices_NAN_model = FindNANindx(model_descriptor);
        *scene_descriptor = cleanNAN (scene_descriptor, indices_NAN_scene);
        *model_descriptor = cleanNAN (model_descriptor, indices_NAN_model);
        *scene_keysOUT = cleanNAN (scene_keysOUT,indices_NAN_scene);
        *model_keysOUT = cleanNAN (model_keysOUT,indices_NAN_model);

        std::cout << "Scene descriptors number after filtering: " << scene_descriptor->size() << std::endl;
        std::cout << "Model descriptors number after filtering: " << model_descriptor->size() << std::endl;
        std::cout << "Scene keypoints number after filtering: " << scene_keysOUT->size() << std::endl;
        std::cout << "Model keypoints number after filtering: " << model_keysOUT->size() << std::endl;

        // Trovo le corrispondenze

        model_scene_corrs_OUT.reset(new pcl::Correspondences);
        *model_scene_corrs_OUT = findCorrs (scene_descriptor, model_descriptor);
    }

    else if (ui->Descr_FPFH->isChecked())
    {
        std::cout << "FPFH..." << std::endl;

        pcl::FPFHEstimation<PointT, PointT, pcl::FPFHSignature33> FPFH_est;
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
        FPFH_est.setSearchMethod (tree);
        FPFH_est.setRadiusSearch (ui->Descr_radius->value());
        //FPFH_est.setKSearch(40);

        // Scene descriptors

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_descriptor (new  pcl::PointCloud<pcl::FPFHSignature33>);
        FPFH_est.setInputCloud(scene_keysOUT);
        FPFH_est.setInputNormals(scene);
        FPFH_est.setSearchSurface(scene);
        FPFH_est.compute (*scene_descriptor);

        // Model descriptors

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_descriptor (new  pcl::PointCloud<pcl::FPFHSignature33>);
        FPFH_est.setInputCloud(model_keysOUT);
        FPFH_est.setInputNormals(model);
        FPFH_est.setSearchSurface(model);
        FPFH_est.compute (*model_descriptor);

        // Use all neighbors in a sphere of radius 5cm
        // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!

        // visualization

        std::cout << "scene descriptor number: " << scene_descriptor->size() << std::endl;
        viewer -> removePointCloud("scene_keysOUT");
        pcl::visualization::PointCloudColorHandlerCustom<PointT> scene_keysOUT_color_handler (scene_keysOUT, 0, 255, 0);
        viewer->addPointCloud (scene_keysOUT, scene_keysOUT_color_handler, "scene_keysOUT");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keysOUT");

        std::cout << "model descriptor number: " << model_descriptor->size() << std::endl;
        viewer -> removePointCloud("model_keysOUT");
        pcl::visualization::PointCloudColorHandlerCustom<PointT> model_keysOUT_color_handler (model_keysOUT, 0, 255, 0);
        viewer->addPointCloud (model_keysOUT, model_keysOUT_color_handler, "model_keysOUT");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_keysOUT");

        // Elimino i NAN

        pcl::PointIndices::Ptr indices_NAN_scene (new pcl::PointIndices);
        pcl::PointIndices::Ptr indices_NAN_model (new pcl::PointIndices);
        *indices_NAN_scene = FindNANindx(scene_descriptor);
        *indices_NAN_model = FindNANindx(model_descriptor);
        *scene_descriptor = cleanNAN (scene_descriptor, indices_NAN_scene);
        *model_descriptor = cleanNAN (model_descriptor, indices_NAN_model);
        *scene_keysOUT = cleanNAN (scene_keysOUT,indices_NAN_scene);
        *model_keysOUT = cleanNAN (model_keysOUT,indices_NAN_model);

        std::cout << "Scene descriptors number after filtering: " << scene_descriptor->size() << std::endl;
        std::cout << "Model descriptors number after filtering: " << model_descriptor->size() << std::endl;
        std::cout << "Scene keypoints number after filtering: " << scene_keysOUT->size() << std::endl;
        std::cout << "Model keypoints number after filtering: " << model_keysOUT->size() << std::endl;

        // Trovo le corrispondenze

        model_scene_corrs_OUT.reset(new pcl::Correspondences);
        *model_scene_corrs_OUT = findCorrs (scene_descriptor, model_descriptor);
    }

    else if (ui->Descr_SHOT->isChecked())
    {        
        std::cout << "SHOT descriptors.." <<std::endl;
        pcl::SHOTEstimationOMP<PointT, pcl::Normal, pcl::SHOT352 > descr_est;
        descr_est.setRadiusSearch (ui->Descr_radius->value());

        pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptor (new  pcl::PointCloud<pcl::SHOT352 >);
        descr_est.setInputCloud (scene_keysOUT);
        descr_est.setInputNormals (scene_normals);
        descr_est.setSearchSurface (scene);
        descr_est.compute (*scene_descriptor);

        pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptor (new  pcl::PointCloud<pcl::SHOT352 >);
        descr_est.setInputCloud (model_keysOUT);
        descr_est.setInputNormals (model_normals);
        descr_est.setSearchSurface (model);
        descr_est.compute (*model_descriptor);

        // visualization

        std::cout << "scene descriptor number: " << scene_descriptor->size() << std::endl;
        viewer -> removePointCloud("scene_keysOUT");
        pcl::visualization::PointCloudColorHandlerCustom<PointT> scene_keysOUT_color_handler (scene_keysOUT, 0, 255, 0);
        viewer->addPointCloud (scene_keysOUT, scene_keysOUT_color_handler, "scene_keysOUT");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keysOUT");

        std::cout << "model descriptor number: " << model_descriptor->size() << std::endl;
        viewer -> removePointCloud("model_keysOUT");
        pcl::visualization::PointCloudColorHandlerCustom<PointT> model_keysOUT_color_handler (model_keysOUT, 0, 255, 0);
        viewer->addPointCloud (model_keysOUT, model_keysOUT_color_handler, "model_keysOUT");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_keysOUT");

        // Elimino i NAN

        pcl::PointIndices::Ptr indices_NAN_scene (new pcl::PointIndices);
        pcl::PointIndices::Ptr indices_NAN_model (new pcl::PointIndices);
        *indices_NAN_scene = FindNANindx(scene_descriptor);
        *indices_NAN_model = FindNANindx(model_descriptor);
        *scene_descriptor = cleanNAN (scene_descriptor, indices_NAN_scene);
        *model_descriptor = cleanNAN (model_descriptor, indices_NAN_model);
        *scene_keysOUT = cleanNAN (scene_keysOUT,indices_NAN_scene);
        *model_keysOUT = cleanNAN (model_keysOUT,indices_NAN_model);

        std::cout << "Scene descriptors number after filtering: " << scene_descriptor->size() << std::endl;
        std::cout << "Model descriptors number after filtering: " << model_descriptor->size() << std::endl;
        std::cout << "Scene keypoints number after filtering: " << scene_keysOUT->size() << std::endl;
        std::cout << "Model keypoints number after filtering: " << model_keysOUT->size() << std::endl;

        // Trovo le corrispondenze

        model_scene_corrs_OUT.reset(new pcl::Correspondences);
        *model_scene_corrs_OUT = findCorrs (scene_descriptor, model_descriptor);
    }

    else if (ui->Descr_SpinImage->isChecked())
    {

        // Method Setup

        std::cout << "Spin Images descriptors.." <<std::endl;
        pcl::SpinImageEstimation<PointT, PointT, pcl::Histogram<153> > SpinIm_est (8, 0.2, 0);
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> );
        SpinIm_est.setSearchMethod(tree);
        SpinIm_est.setRadiusSearch (ui->Descr_radius->value());

        // scene descriptors

        pcl::PointCloud<pcl::Histogram<153> >::Ptr scene_descriptor (new pcl::PointCloud<pcl::Histogram<153> >);
        SpinIm_est.setInputCloud(scene_keysOUT);
        SpinIm_est.setInputNormals(scene_keysOUT);
        SpinIm_est.setSearchSurface(scene);
        SpinIm_est.compute (*scene_descriptor);

        // model descriptors

        pcl::PointCloud<pcl::Histogram<153> >::Ptr model_descriptor (new pcl::PointCloud<pcl::Histogram<153> >);
        SpinIm_est.setInputCloud(model_keysOUT);
        SpinIm_est.setInputNormals(model_keysOUT);
        SpinIm_est.setSearchSurface(model);
        SpinIm_est.compute (*model_descriptor);

        // Display and retrieve the spin image descriptor vector for the first point.
        pcl::Histogram<153> first_descriptor = scene_descriptor->points[0];
        std::cout << first_descriptor << std::endl;

        // Use all neighbors in a sphere of chosen radius
        // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!

        // visualization

        std::cout << "scene keys number: " << scene_keysOUT->size() << std::endl;
        std::cout << "scene descriptor number: " << scene_descriptor->size() << std::endl;
        viewer -> removePointCloud("scene_keysOUT");
        pcl::visualization::PointCloudColorHandlerCustom<PointT> scene_keysOUT_color_handler (scene_keysOUT, 0, 255, 0);
        viewer->addPointCloud (scene_keysOUT, scene_keysOUT_color_handler, "scene_keysOUT");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keysOUT");

        std::cout << "model keys number: " << model_keysOUT->size() << std::endl;
        std::cout << "model descriptor number: " << model_descriptor->size() << std::endl;
        viewer -> removePointCloud("model_keysOUT");
        pcl::visualization::PointCloudColorHandlerCustom<PointT> model_keysOUT_color_handler (model_keysOUT, 0, 255, 0);
        viewer->addPointCloud (model_keysOUT, model_keysOUT_color_handler, "model_keysOUT");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_keysOUT");


        // Elimino i NAN

        pcl::PointIndices::Ptr indices_NAN_scene (new pcl::PointIndices);
        pcl::PointIndices::Ptr indices_NAN_model (new pcl::PointIndices);
        *indices_NAN_scene = FindNANindx(scene_descriptor);
        *indices_NAN_model = FindNANindx(model_descriptor);
        std::cout << "NAN indexes vector size: " << indices_NAN_scene->indices.size() << ", "<< indices_NAN_model->indices.size() << std::endl;
        *scene_descriptor = cleanNAN (scene_descriptor, indices_NAN_scene);
        *model_descriptor = cleanNAN (model_descriptor, indices_NAN_model);
        *scene_keysOUT = cleanNAN (scene_keysOUT,indices_NAN_scene);
        *model_keysOUT = cleanNAN (model_keysOUT,indices_NAN_model);

        std::cout << "Scene descriptors number after filtering: " << scene_descriptor->size() << std::endl;
        std::cout << "Model descriptors number after filtering: " << model_descriptor->size() << std::endl;
        std::cout << "Scene keypoints number after filtering: " << scene_keysOUT->size() << std::endl;
        std::cout << "Model keypoints number after filtering: " << model_keysOUT->size() << std::endl;

        // Trovo le corrispondenze

        model_scene_corrs_OUT.reset(new pcl::Correspondences);
        *model_scene_corrs_OUT = findCorrs (scene_descriptor, model_descriptor);
        std::cout << "Correspondences number: " << model_scene_corrs_OUT->size() << std::endl;
    }

    std::cout << "Descriptors computing time: " << Timer->elapsed() << " ms" <<std::endl;

   // visualize correspondences

    for (int i = 0; i < model_scene_corrs_OUT->size(); ++i)
    {

        if (i > 500) continue;

        // QString nome= QString("line_%1").arg(i);
        std::stringstream ss_line;
        ss_line << "correspondence_line" << i;

        PointT& scene_point = scene_keysOUT->at (model_scene_corrs_OUT->at(i).index_match);
        PointT& model_point = model_keysOUT->at (model_scene_corrs_OUT->at(i).index_query);

        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer->addLine<PointT, PointT> (model_point, scene_point, 0, 255, 0, ss_line.str ());
    }



    //
    //  Actual Clustering
    //
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    if (ui->Corr_hough->isChecked())
    {
        //  Compute (Keypoints) Reference Frames
        Timer->restart();

        pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());
        pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());

        pcl::BOARDLocalReferenceFrameEstimation<PointT, pcl::Normal, pcl::ReferenceFrame> rf_est;
        rf_est.setFindHoles (true);
        rf_est.setRadiusSearch (ui->Corr_Hough_radius->value());

        rf_est.setInputCloud (model_keysOUT);
        rf_est.setInputNormals (model_normals);
        rf_est.setSearchSurface (model);
        rf_est.compute (*model_rf);

        rf_est.setInputCloud (scene_keysOUT);
        rf_est.setInputNormals (scene_normals);
        rf_est.setSearchSurface (scene);
        rf_est.compute (*scene_rf);

        std::cout << "Reference frame comuputing time for Hough3DGrouping: " << Timer->elapsed() << " ms" << std::endl;

        //  Clustering

        Timer->restart();

        pcl::Hough3DGrouping<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
        clusterer.setHoughBinSize (0.01);
        clusterer.setHoughThreshold (5.0);
        clusterer.setUseInterpolation (true);
        clusterer.setUseDistanceWeight (false);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_keysOUT2 (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_keysOUT2 (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud (*model_keysOUT, *model_keysOUT2);
        pcl::copyPointCloud (*scene_keysOUT, *scene_keysOUT2);
        clusterer.setInputCloud (model_keysOUT2);
        clusterer.setInputRf (model_rf);
        clusterer.setSceneCloud (scene_keysOUT2);
        clusterer.setSceneRf (scene_rf);
        clusterer.setModelSceneCorrespondences (model_scene_corrs_OUT);

  //      clusterer.cluster (clustered_corrs);
        clusterer.recognize (rototranslations, clustered_corrs);

        std::cout << "Net correspondence grouping time: " << Timer->elapsed() << " ms" << std::endl;
    }

    if (ui->Corr_gc->isChecked())
    {
        Timer->restart();

        pcl::GeometricConsistencyGrouping<PointT, PointT> gc_clusterer;
        gc_clusterer.setGCSize (0.01f);
        gc_clusterer.setGCThreshold (5.0f);

        gc_clusterer.setSceneCloud (scene_keysOUT);
        gc_clusterer.setInputCloud (model_keysOUT);
        gc_clusterer.setModelSceneCorrespondences (model_scene_corrs_OUT);

        //gc_clusterer.cluster (clustered_corrs);
        gc_clusterer.recognize (rototranslations, clustered_corrs);

        std::cout << "Correspondence grouping time: " << Timer->elapsed() << " ms" << std::endl;
    }

    //
    //  Output results
    //
    std::cout << "Model instances found: " << rototranslations.size () << std::endl;
    for (size_t i = 0; i < rototranslations.size (); ++i)
    {
      std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
      std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;

      // visualize correspondences

       for (int k = 0; k < clustered_corrs[i].size(); ++k)
       {
           if (k > 400) continue;

           // QString nome= QString("line_%1").arg(i);
           std::stringstream ss_line;
           ss_line << "correspondence_clustered_line" << k;

           PointT& scene_point = scene_keysOUT->at (clustered_corrs[i].at(k).index_match);
           PointT& model_point = model_keysOUT->at (clustered_corrs[i].at(k).index_query);

           //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
           viewer->addLine<PointT, PointT> (model_point, scene_point, 0, 0, 255, ss_line.str ());
       }

      // Print the rotation matrix and translation vector
      Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
      Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);
      pcl::PointCloud<PointT>::Ptr current_disp_hyp (new pcl::PointCloud<PointT>);

      printf ("\n");
      printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
      printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
      printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
      printf ("\n");
      printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
      pcl::transformPointCloud (*PCList.at(1), *current_disp_hyp, rototranslations[i]);

      pcl::visualization::PointCloudColorHandlerCustom<PointT> key_color_handler (current_disp_hyp, 200, 0, 200);
      QString nome= QString("disp_Hyp_%1").arg(i);
      viewer->addPointCloud (current_disp_hyp, key_color_handler, nome.toStdString());
    }

    std::cout << "End of recognition and view updating" <<std::endl;
    return;
}

void Elaboration_LF_Rec::on_ComputeKeypoints_clicked()
{
    viewer -> removePointCloud("scene_keysOUT");
    viewer -> removePointCloud("model_keysOUT");

    pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr model (new pcl::PointCloud<PointT>);
//    PointCloudT::Ptr scene_keysOUT (new PointCloudT);
//    PointCloudT::Ptr model_keysOUT (new PointCloudT);
    scene_keysOUT.reset (new PointCloudT);
    model_keysOUT.reset (new PointCloudT);

    scene = PCList.at(0);
    model = PCList.at(1);

    pcl::PointIndicesConstPtr ind;

    //
    //  Downsample Clouds to Extract keypoints
    //
    if (ui->key_uniform->isChecked())
    {
     /*   pcl::UniformSampling<PointT> uniform_sampling;

        // Scene

        pcl::PointCloud<int> indv;

        uniform_sampling.setInputCloud (scene);
        uniform_sampling.setRadiusSearch (ui->key_uniform_radius->value());
        uniform_sampling.compute (indv);
        pcl::copyPointCloud (*scene, indv.points, *scene_keysOUT);


        // Model

        uniform_sampling.setInputCloud (model);
        uniform_sampling.setRadiusSearch (ui->key_uniform_radius->value());
        uniform_sampling.compute (indv);
        pcl::copyPointCloud (*model, indv.points, *model_keysOUT);

        pcl::visualization::PointCloudColorHandlerCustom<PointT> scene_key_color_handler (scene_keysOUT, 0, 0, 255);
        viewer->addPointCloud (scene_keysOUT, scene_key_color_handler, "scene_keysOUT");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keysOUT");
        pcl::visualization::PointCloudColorHandlerCustom<PointT> model_key_color_handler (model_keysOUT, 0, 0, 255);
        viewer->addPointCloud (model_keysOUT, model_key_color_handler, "model_keysOUT");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_keysOUT");*/
    }
    else if (ui->key_ISS3D->isChecked())
    {
        pcl::ISSKeypoint3D<PointT, PointT> iss_detector;

        // Parameters value

        double iss_salient_radius_;
        double iss_non_max_radius_;
        double iss_normal_radius_;
        double iss_border_radius_;
        double iss_gamma_21_ (0.975);
        double iss_gamma_32_ (0.975);
        double iss_min_neighbors_ (5);
        int iss_threads_ (8);

        iss_salient_radius_ = 6 * maxres * ui->key_ISS3D_factor->value();
        iss_non_max_radius_ = 4 * maxres * ui->key_ISS3D_factor->value();
        iss_normal_radius_ = 4 * maxres * ui->key_ISS3D_factor->value();
        iss_border_radius_ = 1 * maxres * ui->key_ISS3D_factor->value();

        // Set parameters

        //    iss_detector.setSearchMethod (tree);
        iss_detector.setSalientRadius (iss_salient_radius_);
        iss_detector.setNonMaxRadius (iss_non_max_radius_);

        iss_detector.setNormalRadius (iss_normal_radius_);
        iss_detector.setBorderRadius (iss_border_radius_);

        iss_detector.setThreshold21 (iss_gamma_21_);
        iss_detector.setThreshold32 (iss_gamma_32_);
        iss_detector.setMinNeighbors (iss_min_neighbors_);
        iss_detector.setNumberOfThreads (iss_threads_);

        // Scene keypoints

        PointCloudT::Ptr scene_keys (new PointCloudT);
        iss_detector.setInputCloud (scene);
        iss_detector.compute (*scene_keys);
        scene_keysOUT = scene_keys;

        // model keypoints

        PointCloudT::Ptr model_keys (new PointCloudT);
        iss_detector.setInputCloud (model);
        iss_detector.compute (*model_keys);
        model_keysOUT = model_keys;


        pcl::visualization::PointCloudColorHandlerCustom<PointT> scene_key_color_handler (scene_keys, 0, 0, 255);
        viewer->addPointCloud (scene_keysOUT, scene_key_color_handler, "scene_keysOUT");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keysOUT");
        pcl::visualization::PointCloudColorHandlerCustom<PointT> model_key_color_handler (model_keys, 0, 0, 255);
        viewer->addPointCloud (model_keysOUT, model_key_color_handler, "model_keysOUT");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_keysOUT");

    }
    else if (ui->key_Harris->isChecked())
    {
        pcl::HarrisKeypoint3D<PointT, pcl::PointXYZI, PointT> detector;

        // scene

       //PointCloudT::Ptr scene_keysOUT (new PointCloudT);
        pcl::PointCloud<pcl::PointXYZI>::Ptr scene_keys (new pcl::PointCloud<pcl::PointXYZI>);
        //PointCloudT::Ptr keys (new PointCloudT ());

        //detector.setNormals(PC);
        //detector.setNonMaxSupression (true);
        //detector.setRadiusSearch (100);
        detector.setRadius(ui->key_Harris_radius->value());
        detector.setInputCloud(scene);
        detector.compute(*scene_keys);

        ind = detector.getKeypointsIndices();
        pcl::copyPointCloud(*scene, ind->indices, *scene_keysOUT);

        // Model

        pcl::PointCloud<pcl::PointXYZI>::Ptr model_keys (new pcl::PointCloud<pcl::PointXYZI>);
        //PointCloudT::Ptr keys (new PointCloudT ());

        //detector.setNormals(PC);
        //detector.setNonMaxSupression (true);
        //detector.setRadiusSearch (100);
        detector.setRadius(ui->key_Harris_radius->value());
        detector.setInputCloud(model);
        detector.compute(*model_keys);

        ind = detector.getKeypointsIndices();
        pcl::copyPointCloud(*model, ind->indices, *model_keysOUT);

        pcl::visualization::PointCloudColorHandlerCustom<PointT> scene_key_color_handler (scene_keysOUT, 0, 0, 255);
        viewer->addPointCloud (scene_keysOUT, scene_key_color_handler, "scene_keysOUT");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keysOUT");
        pcl::visualization::PointCloudColorHandlerCustom<PointT> model_key_color_handler (model_keysOUT, 0, 0, 255);
        viewer->addPointCloud (model_keysOUT, model_key_color_handler, "model_keysOUT");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_keysOUT");
    }

    std::cout << "Keypoints found on scene: "  << scene_keysOUT->size() << std::endl;
    std::cout << "Keypoints found on model: "  << model_keysOUT->size() << std::endl;

    return;
}

pcl::PointIndices  Elaboration_LF_Rec::FindNANindx(pcl::PointCloud<pcl::PFHSignature125>::Ptr PC)
{
    pcl::PointIndices::Ptr indices_NAN (new pcl::PointIndices);
    bool found_NAN_at_i =false;

    for ( size_t i=0; i<PC->size(); i++ )
    {
        found_NAN_at_i =false;
        for ( int j=0; j<125; j++ )
            if( !pcl_isfinite(PC->points[i].histogram[j]))
                found_NAN_at_i =true;

        if(found_NAN_at_i)
            indices_NAN->indices.push_back(i);
    }
    return *indices_NAN ;
}

pcl::PointIndices  Elaboration_LF_Rec::FindNANindx(pcl::PointCloud<pcl::FPFHSignature33>::Ptr PC)
{
    pcl::PointIndices::Ptr indices_NAN (new pcl::PointIndices);
    bool found_NAN_at_i =false;

    for ( size_t i=0; i<PC->size(); i++ )
    {
        found_NAN_at_i =false;
        for ( int j=0; j<33; j++ )
            if( !pcl_isfinite(PC->points[i].histogram[j]))
                found_NAN_at_i =true;

        if(found_NAN_at_i)
            indices_NAN->indices.push_back(i);
    }
    return *indices_NAN ;
}

pcl::PointIndices  Elaboration_LF_Rec::FindNANindx(pcl::PointCloud<pcl::SHOT352>::Ptr PC)
{
    pcl::PointIndices::Ptr indices_NAN (new pcl::PointIndices);
    bool found_NAN_at_i =false;

    for ( size_t i=0; i<PC->size(); i++ )
    {
        found_NAN_at_i =false;
        for ( int j=0; j<352; j++ )
            if( !pcl_isfinite(PC->at(i).descriptor[j]))
                found_NAN_at_i =true;

        if(found_NAN_at_i)
            indices_NAN->indices.push_back(i);
    }
    return *indices_NAN ;
}

pcl::PointIndices  Elaboration_LF_Rec::FindNANindx(pcl::PointCloud<pcl::Histogram<153> >::Ptr PC)
{
   pcl::PointIndices::Ptr indices_NAN (new pcl::PointIndices);
    bool found_NAN_at_i =false;

    for ( size_t i=0; i<PC->size(); i++ )
    {
        found_NAN_at_i =false;
        for ( int j=0; j<153; j++ )
            if( !pcl_isfinite(PC->points[i].histogram[j]))
                found_NAN_at_i =true;

        if(found_NAN_at_i)
            indices_NAN->indices.push_back(i);
    }
    return *indices_NAN ;
}

pcl::PointCloud<pcl::PFHSignature125> Elaboration_LF_Rec::cleanNAN (pcl::PointCloud<pcl::PFHSignature125>::Ptr PC, pcl::PointIndices::Ptr indices_NAN)
{
    pcl::ExtractIndices<pcl::PFHSignature125> extract;
    extract.setInputCloud (PC);
    extract.setIndices (indices_NAN);
    extract.setNegative (true);
    extract.filter (*PC);
    return *PC;
}

pcl::PointCloud<pcl::FPFHSignature33> Elaboration_LF_Rec::cleanNAN (pcl::PointCloud<pcl::FPFHSignature33>::Ptr PC, pcl::PointIndices::Ptr indices_NAN)
{
    pcl::ExtractIndices<pcl::FPFHSignature33> extract;
    extract.setInputCloud (PC);
    extract.setIndices (indices_NAN);
    extract.setNegative (true);
    extract.filter (*PC);
    return *PC;
}

pcl::PointCloud<pcl::SHOT352> Elaboration_LF_Rec::cleanNAN (pcl::PointCloud<pcl::SHOT352>::Ptr PC, pcl::PointIndices::Ptr indices_NAN)
{
    pcl::ExtractIndices<pcl::SHOT352> extract;
    extract.setInputCloud (PC);
    extract.setIndices (indices_NAN);
    extract.setNegative (true);
    extract.filter (*PC);
    return *PC;
}

pcl::PointCloud<myhist> Elaboration_LF_Rec::cleanNAN (pcl::PointCloud<myhist>::Ptr PC, pcl::PointIndices::Ptr indices_NAN)
{
    int mark = 0;
    pcl::PointCloud<myhist> PCtemp;
    if (indices_NAN->indices.size() > 0)
    {
        for (int i = 0; i < PC->size(); ++i)
        {
            if (i == indices_NAN->indices.at(mark))
                mark = mark + 1;
            else
                PCtemp.push_back(PC->at(i));
        }
    }
    else PCtemp = *PC;

    return PCtemp;

}

PointCloudT Elaboration_LF_Rec::cleanNAN (PointCloudT::Ptr PC, pcl::PointIndices::Ptr indices_NAN)
{
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (PC);
    extract.setIndices (indices_NAN);
    extract.setNegative (true);
    extract.filter (*PC);
    return *PC;
}

pcl::Correspondences Elaboration_LF_Rec::findCorrs (pcl::PointCloud<pcl::PFHSignature125>::Ptr scene_descr, pcl::PointCloud<pcl::PFHSignature125>::Ptr model_descr)
{
    pcl::KdTreeFLANN<pcl::PFHSignature125> match_search;
    match_search.setInputCloud (model_descr);
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences);
    for (size_t i = 0; i < scene_descr->size (); ++i)
    {
      std::vector<int> neigh_indices (1);
      std::vector<float> neigh_sqr_dists (1);
      int found_neighs = match_search.nearestKSearch (scene_descr->at (i), 1, neigh_indices, neigh_sqr_dists);
      if(found_neighs == 1 && (!(ui->Descr_DistThresh_check->isChecked()) || neigh_sqr_dists[0] < ui->Descr_DistThresh->value()))
      {
        pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        model_scene_corrs->push_back (corr);
      }
    }
    return *model_scene_corrs;
}

pcl::Correspondences Elaboration_LF_Rec::findCorrs (pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_descr, pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_descr)
{
    pcl::KdTreeFLANN<pcl::FPFHSignature33> match_search;
    match_search.setInputCloud (model_descr);
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences);
    for (size_t i = 0; i < scene_descr->size (); ++i)
    {
      std::vector<int> neigh_indices (1);
      std::vector<float> neigh_sqr_dists (1);
      int found_neighs = match_search.nearestKSearch (scene_descr->at (i), 1, neigh_indices, neigh_sqr_dists);
      if(found_neighs == 1 && (!(ui->Descr_DistThresh_check->isChecked()) || neigh_sqr_dists[0] < ui->Descr_DistThresh->value()))
      {
        pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        model_scene_corrs->push_back (corr);
      }
    }
    return *model_scene_corrs;
}

pcl::Correspondences Elaboration_LF_Rec::findCorrs (pcl::PointCloud<pcl::SHOT352>::Ptr scene_descr, pcl::PointCloud<pcl::SHOT352>::Ptr model_descr)
{
    pcl::KdTreeFLANN<pcl::SHOT352> match_search;
    match_search.setInputCloud (model_descr);
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences);
    for (size_t i = 0; i < scene_descr->size (); ++i)
    {
      std::vector<int> neigh_indices (1);
      std::vector<float> neigh_sqr_dists (1);
      int found_neighs = match_search.nearestKSearch (scene_descr->at (i), 1, neigh_indices, neigh_sqr_dists);
      if(found_neighs == 1 && (!(ui->Descr_DistThresh_check->isChecked()) || neigh_sqr_dists[0] < ui->Descr_DistThresh->value()))
      {
        pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        model_scene_corrs->push_back (corr);
      }
    }
    return *model_scene_corrs;
}

pcl::Correspondences Elaboration_LF_Rec::findCorrs (pcl::PointCloud<pcl::Histogram<153> >::Ptr scene_descr, pcl::PointCloud<pcl::Histogram<153> >::Ptr model_descr)
{
    pcl::KdTreeFLANN<pcl::Histogram<153> > match_search;
    match_search.setInputCloud (model_descr);
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences);
    for (size_t i = 0; i < scene_descr->size (); ++i)
    {
      std::vector<int> neigh_indices (1);
      std::vector<float> neigh_sqr_dists (1);
      int found_neighs = match_search.nearestKSearch (scene_descr->at (i), 1, neigh_indices, neigh_sqr_dists);
      if(found_neighs == 1 && (!(ui->Descr_DistThresh_check->isChecked()) || neigh_sqr_dists[0] < ui->Descr_DistThresh->value()))
      {
        pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        model_scene_corrs->push_back (corr);
      }
    }
    return *model_scene_corrs;
}

//void Elaboration_LF_Rec::visualize_keys (PointCloudT keypoint)
//{

//}




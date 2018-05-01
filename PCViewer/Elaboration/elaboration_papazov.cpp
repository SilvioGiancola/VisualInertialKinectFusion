#include "elaboration_papazov.h"
#include "ui_elaboration_papazov.h"

Elaboration_Papazov::Elaboration_Papazov(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Elaboration_Papazov)
{
    ui->setupUi(this);
    StartClickCount = 0;
}

Elaboration_Papazov::~Elaboration_Papazov()
{
    delete ui;
}



void Elaboration_Papazov::setViewer(pcl::visualization::PCLVisualizer::Ptr myViewer)
{
    viewer = myViewer;
    return;
}

void Elaboration_Papazov::updateList()
{
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
void Elaboration_Papazov::setMinDim()
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

        StartClickCount = 0;
        return;
}


void Elaboration_Papazov::on_StartRecognition_clicked()
{

    std::cout <<"======================================== " << std::endl;
    std::cout << "Action Start n° " << StartClickCount <<std::endl;


    // instantiate normal estimator
    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
    pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setKSearch (20); //model


    // instantiate RANSAC recognition method class
    pcl::recognition::ObjRecRANSAC myPapazov(ui->PairWidth->value(), ui->VoxelSize->value());
    // current model
    pcl::PointCloud<PointT>::Ptr currentmodeltemp (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentmodel (new pcl::PointCloud<pcl::PointXYZ>);
    // normals
    pcl::PointCloud<pcl::Normal>::Ptr currentmodel_normals (new pcl::PointCloud<pcl::Normal>);

    pcl::PointCloud<pcl::Normal>::Ptr scene_normals (new pcl::PointCloud<pcl::Normal>);
    // declare addModel function output
    bool res = false;
    std::cout << "numero elementi PCList: " << PCList.size() << std::endl;

    for (int i =1; i<=(PCList.size()-1); i++)
    {

         currentmodeltemp = PCList.at(i);
         pcl::copyPointCloud(*currentmodeltemp, *currentmodel);

         normal_estimator.setInputCloud(currentmodeltemp);
         normal_estimator.compute(*currentmodel_normals);
         viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(currentmodel, currentmodel_normals, PCList.at(i)->size()/100, mindim/20.0, PCList.at(i)->header.frame_id + "_Normals");

         std::cout << "... " << std::endl;
         res = myPapazov.addModel (*currentmodel, *currentmodel_normals, PCList.at(i)->header.frame_id);
         std::cout << "risultato: " << res << std::endl;
     }


     StartClickCount = StartClickCount +1;

     pcl::PointCloud<PointT>::Ptr scenetemp (new pcl::PointCloud<PointT>);
     scenetemp = PCList.at(0);
     normal_estimator.setInputCloud(scenetemp);
     normal_estimator.setKSearch (25); //scene
     normal_estimator.compute(*scene_normals);
     pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::copyPointCloud(*scenetemp, *scene);

     if (ui->ICP_check->isChecked())
         myPapazov.icpHypothesesRefinementOn();
     else
         myPapazov.icpHypothesesRefinementOff();

     if (ui->IgnoreCoplanarPair_check->isChecked())
     {
         myPapazov.ignoreCoplanarPointPairsOn();
         myPapazov.setMaxCoplanarityAngleDegrees(ui->MaxCoplanarityAngle->value());
     }
     else
         myPapazov.ignoreCoplanarPointPairsOff();

     int ExtIt = 0;
     double maxMatchConf = 0;
     int DetShapesCount = 0;
     int ObjRecCallCount = 0;

     do{
         std::cout << std::endl << "Chiamata ObjRecRANSAC n°" << ExtIt << std::endl;
         ObjRecCallCount = ObjRecCallCount + 1;
         std::list<pcl::recognition::ObjRecRANSAC::Output> detectedShapes;
         myPapazov.recognize(*scene,*scene_normals,detectedShapes ,ui->SuccessProbability->value());

         // Ciclo le detectedShapes (le ipotesi)
         for ( std::list<pcl::recognition::ObjRecRANSAC::Output>::iterator it = detectedShapes.begin() ; it != detectedShapes.end() ; ++it )
             {
                  DetShapesCount = DetShapesCount + 1;
                  pcl::recognition::ObjRecRANSAC::Output shape = *it;

                  // Se trovo una detectedShape con match confidence maggiore della precedente aggiorno il maxMatchConf
                  if (shape.match_confidence_ > maxMatchConf)
                  {
                     maxMatchConf = shape.match_confidence_;
                  }
                  // se il match confidence è maggiore della soglia (solo quando ho selezionato l'opziode della soglia)
                  if (!(ui->MatchConfThresh_check->isChecked()) || shape.match_confidence_ >= ui->MatchConfThresh_value->value())
                  {

                      // Determino la matrice di rototraslazione

                      pcl::PointCloud<PointT>::Ptr current_disp_hyp (new pcl::PointCloud<PointT>);
                      Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

                      for (int i = 0; i < 3 ; i++)
                      {
                          for (int j = 0; j < 3 ; j++)
                          {
                              transform (i,j) = shape.rigid_transform_[i*3+j];

                          }
                      }
                      transform (0,3) = shape.rigid_transform_[9];
                      transform (1,3) = shape.rigid_transform_[10];
                      transform (2,3) = shape.rigid_transform_[11];
                      transform (3,3) = 1.0;

                      // Visualizzo l'ipotesi accettata e la aggiungo all'elenco delle PC

                      std::cout << std::endl << "SHAPE ACCEPTED: Match confidence:" << shape.match_confidence_ << std::endl;
                      pcl::transformPointCloud (*PCList.at(1), *current_disp_hyp, transform );
                      QString DispHypName= QString("disp_Hyp_%1_%2_%3").arg(StartClickCount).arg(ObjRecCallCount).arg(DetShapesCount);
                      current_disp_hyp->header.frame_id = DispHypName.toStdString();
                      emit addPC(current_disp_hyp);

                      // eseguo l'allineamento finale con ICP

                      pcl::IterativeClosestPoint<PointT, PointT> icp;
                      pcl::registration::CorrespondenceEstimationBase<PointT, PointT>::Ptr cens;
                      cens.reset(new pcl::registration::CorrespondenceEstimation<PointT, PointT>);

                      cens->setInputSource (current_disp_hyp);
                      cens->setInputTarget (PCList.at(0));
                      icp.setCorrespondenceEstimation (cens);
                      pcl::registration::TransformationEstimation<PointT, PointT>::Ptr te;
                      te.reset(new pcl::registration::TransformationEstimationSVD<PointT, PointT>);
                      icp.setTransformationEstimation (te);
                      icp.setInputSource(current_disp_hyp);
                      icp.setInputTarget(PCList.at(0));

                      icp.setMaximumIterations(100);

                      PointCloudT::Ptr current_disp_hyp_ICP (new PointCloudT);
                      icp.align(*current_disp_hyp_ICP);
                      std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
                      Eigen::Matrix4f ICPtransformation = icp.getFinalTransformation();

                      Eigen::Matrix4f transformTOT = Eigen::Matrix4f::Identity();
                      Eigen::Matrix4f INVtransformTOT = Eigen::Matrix4f::Identity();

                      transformTOT = ICPtransformation * transform;
                      INVtransformTOT = transformTOT.inverse();

                      std::cout << std::endl << "Recognition transform: " << std::endl << transform  << std::endl;
                      std::cout << std::endl << "ICP transform: " << std::endl << ICPtransformation  << std::endl;
                      std::cout << std::endl << "Total transform: " << std::endl << transformTOT << std::endl;
                      std::cout << std::endl << "Total inverse transform: " << std::endl << INVtransformTOT << std::endl;

                      QString DispHypICPName= QString("disp_Hyp_%1_%2_%3_ICP").arg(StartClickCount).arg(ObjRecCallCount).arg(DetShapesCount);
                      current_disp_hyp_ICP->header.frame_id =  DispHypICPName.toStdString();
                      emit addPC(current_disp_hyp_ICP);

//                      // Prova matrice inversa totale:
//                      PointCloudT::Ptr trasformazione_inversa (new PointCloudT);
//                      pcl::transformPointCloud (*current_disp_hyp_ICP, *trasformazione_inversa, INVtransformTOT);
//                      pcl::visualization::PointCloudColorHandlerCustom<PointT> trasformazione_inversa_color_handler (trasformazione_inversa, 0, 255, 0);
//                      viewer->addPointCloud (trasformazione_inversa, trasformazione_inversa_color_handler, "trasformazione_inversa");
//                      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "trasformazione_inversa");

                  }
                  else std::cout << "Shape rejected: match confidence is: " << shape.match_confidence_ << std::endl;

              }
         ExtIt=ExtIt+1; 
     }while(ui->MatchConfThresh_check->isChecked() && maxMatchConf <= ui->MatchConfThresh_value->value() && ExtIt < ui->MaxIteration->value());


     emit PCupdated();
}

void Elaboration_Papazov::on_AutoSet_clicked()
{
    // compute maximum resolution among point clouds
      double currentres = 0.0;
      double maxres = 0.0;
      int n_points = 0;
      int nres;


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

      std::cout << "maximum chatacteristic dimention among point clouds: " << maxres << std::endl;
    // Set values
    double chosen_PairWidth = mindim/4.0;
    double chosen_VoxelSize = maxres*2.0;

    std::cout << "chosen pair width: " << mindim/5 << std::endl;
    std::cout << "chosen voxel size: " << maxres*2 << std::endl;

    ui->PairWidth->setValue(chosen_PairWidth);
    ui->VoxelSize->setValue(chosen_VoxelSize);
}



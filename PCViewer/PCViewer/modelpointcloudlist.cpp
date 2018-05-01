#include "modelpointcloudlist.h"


std::string SuffixCoordinateSystem = "_CoordinateSystem";
std::string SuffixNormal = "_Normal";
std::string SuffixKeyPoints = "_KeyPoints";

double lengthNormal = 0.05;
double levelNormal = 1; //20


/* QStandardItemModel - Constructor
 * This is the construtor where the only variable selectedPC is initialized */
ModelPointCloudList::ModelPointCloudList(QObject *parent) : QStandardItemModel(parent)
{
    selectedPC.reset(new PointCloudT());

    undoStack = new QUndoStack(this);
}


/* QStandardItemModel - Count Rows
 * This return the number of opened point clouds, in order to define the number of rows */
int ModelPointCloudList::rowCount(const QModelIndex &parent) const
{
    if(parent == invisibleRootItem()->index())
        return _PCList.size();
    else
        return PROP_NBPROP;
}


/* QStandardItemModel - Get Data
 *
 * return the data to show in the tree
 *
 */
QVariant ModelPointCloudList::data(const QModelIndex &index, int role) const
{

    // in case of display and edition of the data:
    if (role == Qt::DisplayRole || role == Qt::EditRole)
    {

        // First Level: it correspond to the name of the point cloud
        if (index.parent() == invisibleRootItem()->index())
        {
            if (index.column() == 0 )        return QString("Frame ID");
            else if (index.column() == 1 && role == Qt::DisplayRole )   return QString::fromStdString(_PCList.at(index.row())->header.frame_id).section("/", -1,-1);
            else if (index.column() == 1 && role == Qt::EditRole )   return QString::fromStdString(_PCList.at(index.row())->header.frame_id);
        }

        // Second Level: it correspond to the parameters of the point cloud
        else if (index.parent().parent() == invisibleRootItem()->index())
        {

            QModelIndex PCindex = index.parent();

            // Intrinsic parameters
            if (index.row() == PROP_NUMBER && index.column() == 0)    return QString("Number");
            if (index.row() == PROP_NUMBER && index.column() == 1)    return QLocale(QLocale::German).toString((int)_PCList.at(PCindex.row())->size());

            if (index.row() == PROP_ORGANIZED && index.column() == 0)   return QString("Organized");
            if (index.row() == PROP_ORGANIZED && index.column() == 1)
            {
                if ( _PCList.at(PCindex.row())->isOrganized() ) return QString("w:%1 x h:%2").arg(_PCList.at(PCindex.row())->width).arg(_PCList.at(PCindex.row())->height);
                else return false;
            }

            if (index.row() == PROP_DENSE && index.column() == 0)       return QString("Dense");
            if (index.row() == PROP_DENSE && index.column() == 1)       return (bool) ( _PCList.at(PCindex.row())->is_dense);


            // Visualization parameters
            double point_size = 1, opacity = 1;
            if (_viewer->contains(_PCList.at(PCindex.row())->header.frame_id))
            {
                _viewer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, _PCList.at(PCindex.row())->header.frame_id);
                _viewer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, _PCList.at(PCindex.row())->header.frame_id);
            }

            if (index.row() == PROP_SIZE && index.column() == 0)        return QString("Size");
            if (index.row() == PROP_SIZE && index.column() == 1)        return point_size;

            if (index.row() == PROP_OPACITY && index.column() == 0)     return QString("Opacity");
            if (index.row() == PROP_OPACITY && index.column() == 1)     return opacity*10;

            if (index.row() == PROP_NORMAL && index.column() == 0)      return QString("Normals");
            if (index.row() == PROP_NORMAL && index.column() == 1)      return QVariant();
        }

    }


    // This define the font for every index displayed
    else if (role == Qt::FontRole){}


    // This define the alignement for the index displayed
    else if (role ==  Qt::TextAlignmentRole) {}


    // This define if the index can be and is checked
    else if (role ==  Qt::CheckStateRole)
    {
        // This correspond to the first level
        if ( index.parent() == invisibleRootItem()->index() )
        {
            // on which I only care about the first column
            if ( index.column() == 0 ) // If PointCloudItem
            {
                if (_viewer->contains(_PCList.at(index.row())->header.frame_id))        return Qt::Checked;
                else if (!_viewer->contains(_PCList.at(index.row())->header.frame_id))  return Qt::Unchecked;
            }
        }

        // Second Level: it correspond to the parameters of the point cloud
        else if (index.parent().parent() == invisibleRootItem()->index())
        {
            if (index.row() == PROP_NORMAL && index.column() == 0)
            {
                if (_viewer->contains(_PCList.at(index.parent().row())->header.frame_id + SuffixNormal))        return Qt::Checked;
                else if (!_viewer->contains(_PCList.at(index.parent().row())->header.frame_id + SuffixNormal))  return Qt::Unchecked;
            }
        }
    }

    return QVariant();
}


/* QStandardItemModel - Set Data
 *
 * set the data shown in the tree when occur an edition
 *
 */
bool ModelPointCloudList::setData(const QModelIndex & index, const QVariant & value, int role)
{
    // in case of edition of my datas
    if (role == Qt::EditRole)
    {
        // Only the First column can be edited
        if ( index.column() == 1 )
        {
            // First Level : Corespond to the point cloud ID
            if ( index.parent() == invisibleRootItem()->index() )
            {
                // I remove the old point cloud
                removePointCloudFromViewer(_PCList.at(index.row()));

                // I change its name
                _PCList.at(index.row())->header.frame_id = value.toString().toStdString();

                // I add the point cloud with its new name
                addPointCloudToViewer(_PCList.at(index.row()), false, true);
            }

            // Second level: correspond to the properties
            else if ( index.parent().parent() == invisibleRootItem()->index() ) // Second Level
            {
                QModelIndex PCindex = index.parent();
                if (index.row() == PROP_DENSE)
                    _PCList.at(PCindex.row())->is_dense = value.toBool();

                else if (index.row() == PROP_SIZE && _viewer->contains(_PCList.at(PCindex.row())->header.frame_id))
                    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, (value.toInt()>0)?value.toInt():1, _PCList.at(PCindex.row())->header.frame_id);

                else if (index.row() == PROP_OPACITY && _viewer->contains(_PCList.at(PCindex.row())->header.frame_id))
                    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, value.toDouble()/10, _PCList.at(PCindex.row())->header.frame_id);


            }
            emit updateViewer();
        }
    }


    // Handle the checkboxes
    else if (role == Qt::CheckStateRole)
    {
        // Check the first column only
        if ( index.column() == 0 ) // If PointCloudItem
        {
            // First Level: correspond to the Point Clouds
            if ( index.parent() == invisibleRootItem()->index() )
            {
                // If the checkbox is checked, I add the point cloud
                if (value == Qt::Checked)
                    addPointCloudToViewer(_PCList.at(index.row()),false);

                // If the checkbox is unchecked, I remove the point cloud
                else if  (value == Qt::Unchecked)
                    removePointCloudFromViewer(_PCList.at(index.row()));

                emit dataChanged(index.child(0,0),index.child(PROP_NBPROP,1));
            }


            // Second level: correspond to the properties
            else if ( index.parent().parent() == invisibleRootItem()->index() ) // Second Level
            {
                if ( index.row() == PROP_NORMAL )
                {
                    // If the checkbox is checked, I add the Normal to the existing PC
                    if (value == Qt::Checked)
                        addPointCloudToViewer(_PCList.at(index.parent().row()));

                    // If the checkbox is unchecked, I remove only the Normal PC
                    else if  (value == Qt::Unchecked)
                        if (_viewer->contains(_PCList.at(index.parent().row())->header.frame_id + SuffixNormal))
                        {
                            _viewer->removePointCloud(_PCList.at(index.parent().row())->header.frame_id + SuffixNormal);
                            emit updateViewer();
                        }
                }
            }
        }
    }

    return true;
}


/* QStandardItemModel - Set Flags
 *
 * The Flag handles the index properties (Enable, Selectable, Checkable, Editable)
 *
 */
Qt::ItemFlags ModelPointCloudList::flags(const QModelIndex & index) const
{
    // First level: the point cloud
    if ( index.parent() == invisibleRootItem()->index())
    {
        if ( index.column() == 0 )    return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsUserCheckable;
        if ( index.column() == 1 )    return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsEditable;
    }

    // Second Level the Properties
    else if ( index.parent().parent() == invisibleRootItem()->index())
    {
        // First column
        if (index.column() == 0 )
        {
            if (index.row() == PROP_NORMAL )    return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsUserCheckable;
            else                                return Qt::ItemIsSelectable | Qt::ItemIsEnabled;
        }

        // Second Column
        if (index.column() == 1 )
        {
            if (index.row() == PROP_NORMAL)     return Qt::ItemIsSelectable | Qt::ItemIsEnabled;
            if (index.row() == PROP_NUMBER)     return Qt::ItemIsSelectable | Qt::ItemIsEnabled;
            if (index.row() == PROP_ORGANIZED)  return Qt::ItemIsSelectable | Qt::ItemIsEnabled;
            if (index.row() == PROP_DENSE)      return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsEditable;
            if (index.row() == PROP_SIZE)       return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsEditable;
            if (index.row() == PROP_OPACITY)    return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsEditable;
        }

    }

    return Qt::ItemFlags();
}


/* QStandardItemModel - Set Headers
 *
 * The Headers are the titles given to the column and rows
 *
 */
QVariant ModelPointCloudList::headerData(int section, Qt::Orientation orientation, int role) const
{
    // Only the display role is handles
    if (role == Qt::DisplayRole)
    {
        if (orientation == Qt::Horizontal)
        {
            if (section == 0)   return QString("Name");
            if (section == 1)   return QString("Value");
        }

        if (orientation == Qt::Vertical)
            return section;
    }
    return QVariant();
}




/******************
 * Point Cloud Handling
 **************/

#include <pcl/filters/voxel_grid.h>
// add a point cloud to both list and viewer
void ModelPointCloudList::addPointCloud(PointCloudT::Ptr PC)
{
    // Check doublons
    for (int i = 0; i < invisibleRootItem()->rowCount(); i++)
    {
        if (QString::compare(QString::fromStdString(PC->header.frame_id), QString::fromStdString(_PCList.at(i)->header.frame_id)) == 0)
        {
            std::cout << "Point Cloud already in List" << std::endl;
            return;
        }
    }


    // Add PC to Model
    addPointCloudToModel(PC);


    // Add PC to Viewer
    addPointCloudToViewer(PC,false);


    return;
}

void ModelPointCloudList::addPointCloudToModel(PointCloudT::Ptr PC)
{
    QStandardItem* myCloudItem = new QStandardItem();
    myCloudItem->setCheckable(true);

    for(int i = 0; i < PROP_NBPROP; i++)
    {
        QList<QStandardItem*> myCloudProp;
        myCloudProp.append(new QStandardItem("Name"));
        myCloudProp.append(new QStandardItem("Value"));
        myCloudItem->appendRow(myCloudProp);
    }

    QList<QStandardItem*> myCloudItem2;
    myCloudItem2.append(myCloudItem);
    myCloudItem2.append(new QStandardItem());

    invisibleRootItem()->appendRow(myCloudItem2);



    // update PCList
    _PCList.append(PC);
    selectedPC = _PCList.back();
    emit setCurrentPC(selectedPC);
    emit setCurrentPCList(_PCList);

    return;
}

void ModelPointCloudList::addPointCloudToViewer(PointCloudT::Ptr PC, bool showNormal, bool showCoordinateSystem)
{

    PointCloudT::Ptr PCview(new PointCloudT());

    if (PC->size()>10000000)
    {
        PointT min;
        PointT max;
        pcl::getMinMax3D(*PC,min,max);

        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud (PC);
        sor.setLeafSize ((max.x - min.x)/500.0, (max.y - min.y)/500.0, (max.z - min.z)/500.0);
        sor.filter (*PCview);

        std::cout << "reduced point cloud ( " << PC->size() << " -> " << PCview->size() << " )"<< std::endl;
    }
    else
    {
        PCview = PC;
        // pcl::copyPointCloud(*PC, *PCView);
    }

    if ( _viewer->contains(PCview->header.frame_id) )
        _viewer->removePointCloud(PCview->header.frame_id);

    pcl::visualization::PointCloudColorHandlerRGBField<PointT> single_color(PCview);
    _viewer->addPointCloud<PointT>(PCview, single_color, PCview->header.frame_id);
    //  _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, true, PCview->header.frame_id);


    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans.block(0,0,3,3) = PCview->sensor_orientation_.matrix();
    trans.block(0,3,4,1) = PCview->sensor_origin_;

    // Coordinate System
    double dimCoordinateSystem = 0.3; //m
    if (showCoordinateSystem)
        _viewer->addCoordinateSystem(dimCoordinateSystem, Eigen::Affine3f(trans),PCview->header.frame_id + SuffixCoordinateSystem);


    // Normals
    if (showNormal)
    {
        PointCloudT::Ptr PCnormal (new PointCloudT);
        pcl::transformPointCloud (*PCview, *PCnormal, trans);

        if ( _viewer->contains(PCnormal->header.frame_id + SuffixNormal) )
            _viewer->removePointCloud(PCnormal->header.frame_id + SuffixNormal);

        _viewer->addPointCloudNormals<PointT>(PCnormal, levelNormal, lengthNormal, PCnormal->header.frame_id + SuffixNormal);
    }

    emit updateViewer();
    return;
}





bool ModelPointCloudList::removePointCloud(PointCloudT::Ptr PC)
{
    // Remove PC to Viewer
    removePointCloudFromViewer(PC);

    // Remove PC to Model
    removePointCloudFromModel(PC);

    return false;
}

bool ModelPointCloudList::removePointCloudFromModel(PointCloudT::Ptr PC)
{
    bool ret = true;
    QString str1 = QString::fromStdString(PC->header.frame_id).section("/", -1,-1);
    //Update Model
    for (int i = 0; i < invisibleRootItem()->rowCount(); i++)
    {
        QString str2 = this->invisibleRootItem()->child(i,1)->index().data().toString();

        if (QString::compare(str1,str2) == 0)
        {
            ret = false;
            removeRow(i);
            _PCList.removeAt(i);

            if ( _PCList.size() > 0 )
                PC = _PCList.at((i<_PCList.size())?i:_PCList.size()-1);
            else
                PC.reset(new PointCloudT);

            i--;
        }
    }
    emit setCurrentPCList(_PCList);
    emit setCurrentPC(PC);

    return ret;
}

bool ModelPointCloudList::removePointCloudFromViewer(PointCloudT::Ptr PC)
{
    if (_viewer->contains(PC->header.frame_id))
        _viewer->removePointCloud(PC->header.frame_id);

    if (_viewer->contains(PC->header.frame_id + SuffixCoordinateSystem))
        _viewer->removeCoordinateSystem(PC->header.frame_id + SuffixCoordinateSystem);

    if (_viewer->contains(PC->header.frame_id + SuffixNormal))
        _viewer->removePointCloud(PC->header.frame_id + SuffixNormal);

    emit updateViewer();
    return false;
}





void ModelPointCloudList::replacePointCloud(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC, QString ElabName)
{
    undoStack->push(new Elaboration_Command(this, oldPC, newPC, ElabName));
    return;
}

void ModelPointCloudList::replacePointCloudToModel(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC)
{
    for (int i = 0; i < _PCList.size(); i++)
    {
        QString str1 = this->invisibleRootItem()->child(i,1)->index().data().toString();
        QString str2 = QString::fromStdString(oldPC->header.frame_id).section("/",-1,-1);
        if (QString::compare(str1,str2) == 0)
        {
            _PCList.replace(i, newPC);
        }
    }
    // pcl::copyPointCloud(*newPC, *selectedPC);
    selectedPC = newPC;
    emit setCurrentPC(selectedPC);
    emit setCurrentPCList(_PCList);
    emit dataChanged(invisibleRootItem()->index(), invisibleRootItem()->index());
    return;
}

void ModelPointCloudList::replacePointCloudToViewer(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC)
{
    // check rendering properties
    double opacity = 1, point_size =1;
    _viewer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, oldPC->header.frame_id);
    _viewer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, oldPC->header.frame_id);

    // check if normals are also plotted
    int Norm = _viewer->contains(oldPC->header.frame_id + SuffixNormal);

    // Remove PC to Viewer
    removePointCloudFromViewer(oldPC);

    // Add PC to Viewer
    addPointCloudToViewer(newPC,Norm);

    // reset rendering properties
    _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, oldPC->header.frame_id);
    _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, oldPC->header.frame_id);

    emit updateViewer();


}


bool ModelPointCloudList::movePointCloud(PointCloudT::Ptr PC, Eigen::Matrix4f Mat)
{
    PointCloudT::Ptr newPC (new PointCloudT);
    for (int i = 0; i < _PCList.size(); i++)
    {
        QString str1 = this->invisibleRootItem()->child(i,1)->index().data().toString();
        QString str2 = QString::fromStdString(PC->header.frame_id).section("/",-1,-1);
        if (QString::compare(str1,str2) == 0)
        {
            newPC = _PCList.at(i);
        }
    }
    Eigen::Matrix4f currentPose = Eigen::Matrix4f::Identity();
    currentPose.block(0,0,3,3) = newPC->sensor_orientation_.matrix();
    currentPose.block(0,3,3,1) = newPC->sensor_origin_.head(3);

    currentPose = currentPose * Mat;



    newPC->sensor_origin_ = currentPose.block<4,1>(0,3);
    newPC->sensor_orientation_ = Eigen::Quaternionf(currentPose.block<3,3>(0,0));


   // newPC->sensor_orientation_ =  newPC->sensor_orientation_  * Mat
   /* // pcl::copyPointCloud(*newPC, *selectedPC);
    selectedPC = newPC;
    emit setCurrentPC(selectedPC);
    emit setCurrentPCList(_PCList);
    emit dataChanged(invisibleRootItem()->index(), invisibleRootItem()->index());*/

    // Remove PC to Viewer
    removePointCloudFromViewer(newPC);

    // Add PC to Viewer
    addPointCloudToViewer(newPC);

    return false;

}




//Selected PC
bool ModelPointCloudList::saveCurrentPointCloud(bool binary, bool normal)
{
    QString filename = QString::fromStdString(selectedPC->header.frame_id);


    if (selectedPC->size() > 0)
    {
        QDir dir(filename.section("/",0,-2));
        if (!dir.exists()) {
            dir.mkpath(".");
        }

        if (!normal)
        {
            pcl::PointCloud<pcl::PointXYZRGB> PC_nonormal;
            pcl::copyPointCloud(*selectedPC, PC_nonormal);

            if (filename.endsWith(".pcd"))
                return pcl::io::savePCDFile(selectedPC->header.frame_id, PC_nonormal, binary);
            if (filename.endsWith(".ply"))
                return pcl::io::savePLYFile(selectedPC->header.frame_id, PC_nonormal, binary);
        }
        else
        {
            if (filename.endsWith(".pcd"))
                return pcl::io::savePCDFile(selectedPC->header.frame_id, *selectedPC, binary);
            if (filename.endsWith(".ply"))
                return pcl::io::savePLYFile(selectedPC->header.frame_id, *selectedPC, binary);
        }

    }



    return true;
}

bool ModelPointCloudList::removeCurrentPointCloud()
{
    // Remove PC to Viewer
    removePointCloudFromViewer(selectedPC);

    // Remove PC to Model
    removePointCloudFromModel(selectedPC);


    return false;
}

bool ModelPointCloudList::updateCurrentPointCloudToViewer()
{
    int Norm = _viewer->contains(selectedPC->header.frame_id + SuffixNormal);

    // Remove PC to Viewer
    removePointCloudFromViewer(selectedPC);

    // Add PC to Viewer
    addPointCloudToViewer(selectedPC,Norm);

    return false;
}
// ALL
void ModelPointCloudList::saveAllPointClouds(bool binary, bool normal)
{
    for (int i = 0; i < _PCList.size(); i ++)
    {
        QString filename = QString::fromStdString(_PCList.at(i)->header.frame_id);

        QDir dir(filename.section("/",0,-2));
        if (!dir.exists()) {
            dir.mkpath(".");
        }
        if (!normal)
        {
            pcl::PointCloud<pcl::PointXYZRGB> PC_nonormal;
            pcl::copyPointCloud(*_PCList.at(i), PC_nonormal);

            if (filename.endsWith(".pcd"))
                pcl::io::savePCDFile(_PCList.at(i)->header.frame_id, PC_nonormal, binary);
            if (filename.endsWith(".ply"))
                pcl::io::savePLYFile(_PCList.at(i)->header.frame_id, PC_nonormal, binary);
        }
        else
        {
            if (filename.endsWith(".pcd"))
                pcl::io::savePCDFile(_PCList.at(i)->header.frame_id, *_PCList.at(i), binary);
            if (filename.endsWith(".ply"))
                pcl::io::savePLYFile(_PCList.at(i)->header.frame_id, *_PCList.at(i), binary);
        }
    }

    return;
}

void ModelPointCloudList::removeAllPointClouds()
{
    // remove All PCList
    _viewer->removeAllPointClouds();
    _viewer->removeAllCoordinateSystems();
    _viewer->removeAllShapes();
  //  _viewer->addCoordinateSystem(1.0);
    removeRows(0,rowCount());
    _PCList.clear();

    // Emit a signal in order to update the viewer
    emit updateViewer();

    PointCloudT::Ptr pc(new PointCloudT());
    emit setCurrentPC(pc);
    emit setCurrentPCList(_PCList);
    // Emit a signal in order to update the Model
    emit dataChanged(invisibleRootItem()->index(), invisibleRootItem()->index());

}


void ModelPointCloudList::hideAllPointClouds()
{
   // for (int i = 0; i < _PCList.size(); i++)
       // removePointCloudFromViewer(_PCList.at(i));

    _viewer->removeAllPointClouds();
    _viewer->removeAllCoordinateSystems();
    _viewer->removeAllShapes();

    // Emit a signal in order to update the viewer
    emit updateViewer();
    // Emit a signal in order to update the Model
    emit dataChanged(invisibleRootItem()->index(), invisibleRootItem()->index());


    return;
}

void ModelPointCloudList::showAllPointClouds()
{
    for (int i = 0; i < _PCList.size(); i++)
        addPointCloudToViewer(_PCList.at(i),false, true);

    // Emit a signal in order to update the viewer
    emit updateViewer();
    // Emit a signal in order to update the Model
    emit dataChanged(invisibleRootItem()->index(), invisibleRootItem()->index());

    return;
}

void ModelPointCloudList::mergeAllPointClouds()
{
    PointCloudT::Ptr ConcatenatedPC (new PointCloudT());

    ConcatenatedPC->header.frame_id = QString("%1/PointClouds/merged.pcd").arg(QDir::homePath()).toStdString();


    for (int i = 0; i < _PCList.size() ; i++)
    {
        PointCloudT::Ptr temp(new PointCloudT());

        Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
        trans.block(0,0,3,3) = _PCList.at(i)->sensor_orientation_.matrix();
        trans.block(0,3,4,1) = _PCList.at(i)->sensor_origin_;

        pcl::transformPointCloud(*_PCList.at(i), *temp, trans);

        *ConcatenatedPC += *temp;

    }
    addPointCloud(ConcatenatedPC);


    // Emit a signal in order to update the viewer
    emit updateViewer();
    // Emit a signal in order to update the Model
    emit dataChanged(invisibleRootItem()->index(), invisibleRootItem()->index());
}



void ModelPointCloudList::exportPoses(QString filename)
{

    QFile file(filename);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        /*  QMessageBox msgBox;
        msgBox.setText("File opened");
        msgBox.exec();
        */

        for (int i = 0; i < _PCList.size(); i++)
        {
            Eigen::Quaternionf orientation = _PCList.at(i)->sensor_orientation_;
            Eigen::Vector4f origin = _PCList.at(i)->sensor_origin_;
            QString Pose = QString("%1 %2 %3 %4 %5 %6 %7 \n")
                    .arg(origin[0]).arg(origin[1]).arg(origin[2])
                    .arg(orientation.w()).arg(orientation.x()).arg(orientation.y()).arg(orientation.z());
            file.write(Pose.toStdString().c_str());



        }
        file.close();
    }
}


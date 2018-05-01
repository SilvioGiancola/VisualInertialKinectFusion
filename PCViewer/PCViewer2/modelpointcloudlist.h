#ifndef MODELPOINTCLOUDLIST_H
#define MODELPOINTCLOUDLIST_H

#include <QStandardItemModel>
#include <QDoubleSpinBox>
#include <QStyledItemDelegate>
#include <QUndoStack>
#include <QDir>

#include <define.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>



#define PROP_NUMBER 0
#define PROP_ORGANIZED 1
#define PROP_DENSE 2
#define PROP_OPACITY 3
#define PROP_SIZE 4
#define PROP_NORMAL 5
#define PROP_NBPROP 6

#define PROP_KEYPOINTS 19
#define PROP_TIME_STAMP 20
#define PROP_SEQ 21
#define PROP_WIDTH 22
#define PROP_HEIGHT 23


class ModelPointCloudList : public QStandardItemModel
{
    Q_OBJECT
public:
    ModelPointCloudList(QObject *parent = 0);

    int rowCount(const QModelIndex &parent = QModelIndex()) const;
    int columnCount() const  { return 2; }

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const;

    bool setData(const QModelIndex & index, const QVariant & value, int role = Qt::EditRole);
    Qt::ItemFlags flags(const QModelIndex & index) const ;


    bool setViewer(pcl::visualization::PCLVisualizer::Ptr viewer) {_viewer = viewer; return true;}
    bool setPCList(QList<PointCloudT::Ptr> PCList) {_PCList = PCList; return true;}

    pcl::visualization::PCLVisualizer::Ptr  getViewer()     {return _viewer;}
    QList<PointCloudT::Ptr>  getPCList()        {return _PCList;}
    PointCloudT::Ptr         getPC(int i)       {return _PCList.at(i);}
    std::string              getPCname(int i)   {return _PCList.at(i)->header.frame_id;}

    void emitCurrentPC() {emit setCurrentPC(selectedPC);}

    PointCloudT::Ptr selectedPC;
    QUndoStack *undoStack;

signals:
    void updateViewer();
    void setCurrentPC(PointCloudT::Ptr);
    void setCurrentPCList(QList<PointCloudT::Ptr>);

public slots:
    //Single Point CLoud Operation
    void addPointCloud(PointCloudT::Ptr PC);
    bool removePointCloud(PointCloudT::Ptr PC);
    void replacePointCloud(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC, QString ElabName = QString("Elaboration"));

    bool saveCurrentPointCloud(bool binary = true, bool normal = true);
    bool removeCurrentPointCloud();
    bool updateCurrentPointCloudToViewer();

    // Allcloud operation
    void saveAllPointClouds(bool binary = true, bool normal = true);
    void removeAllPointClouds();
    void hideAllPointClouds();
    void showAllPointClouds();
    void mergeAllPointClouds();
    void exportPoses(QString filename);



    void addPointCloudToModel(PointCloudT::Ptr PC);
    void addPointCloudToViewer(PointCloudT::Ptr PC, bool showNormal = true, bool showCoordinateSystem = true);
    bool removePointCloudFromModel(PointCloudT::Ptr PC);
    bool removePointCloudFromViewer(PointCloudT::Ptr PC);
    void replacePointCloudToModel(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC);
    void replacePointCloudToViewer(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC);

private:
    pcl::visualization::PCLVisualizer::Ptr _viewer;
    QList<PointCloudT::Ptr> _PCList;



};


class LoadCommand : public QUndoCommand
{
public:
    LoadCommand(PointCloudT::Ptr PC, ModelPointCloudList *PointCloudListModel)
    {
       _PC = PC;
       _PointCloudListModel = PointCloudListModel;
       QString ID = QString::fromStdString(_PC->header.frame_id).section("/",-1,-1);
       setText(QString("Load PC: %1").arg(ID));
    }

    void undo()  { _PointCloudListModel->removePointCloud(_PC); }
    void redo()  { _PointCloudListModel->addPointCloud(_PC); }

private:
    PointCloudT::Ptr _PC;
    ModelPointCloudList *_PointCloudListModel;
};

class RemoveCommand : public QUndoCommand
{
public:
    RemoveCommand(PointCloudT::Ptr PC, ModelPointCloudList *PointCloudListModel)
    {
       _PC = PC;
       _PointCloudListModel = PointCloudListModel;
       QString ID = QString::fromStdString(_PC->header.frame_id).section("/",-1,-1);
       setText(QString("Remove PC: %1").arg(ID));
    }

    void undo()  { _PointCloudListModel->addPointCloud(_PC); }
    void redo()  { _PointCloudListModel->removePointCloud(_PC); }

private:
    PointCloudT::Ptr _PC;
    ModelPointCloudList *_PointCloudListModel;
};

class CopyCommand : public QUndoCommand
{
public:
    CopyCommand(PointCloudT::Ptr PC, ModelPointCloudList *PointCloudListModel)
    {
     //  _PC = PC;
       _PointCloudListModel = PointCloudListModel;
       QString ID = QString::fromStdString(PC->header.frame_id).section("/",-1,-1);
       setText(QString("Copy PC: %1").arg(ID));
       appendix = QString::fromStdString(" (Copy).pcd");

       _PCcopy.reset(new PointCloudT);
       pcl::copyPointCloud(*PC, *_PCcopy);

       QString name = QString::fromStdString(PC->header.frame_id);
       name.chop(4);
       _PCcopy->header.frame_id = name.append(appendix).toStdString();
    }

    void undo()  { _PointCloudListModel->removePointCloud(_PCcopy); }

    void redo()  { _PointCloudListModel->addPointCloud(_PCcopy); }

private:
  //  PointCloudT::Ptr _PC;
    PointCloudT::Ptr _PCcopy;
    ModelPointCloudList *_PointCloudListModel;
    QString appendix;
};

class Elaboration_Command : public QUndoCommand
{
public:
    Elaboration_Command(ModelPointCloudList *m, PointCloudT::Ptr myold, PointCloudT::Ptr mynew, QString ElabName)
    {
        _m = m;
        _old = myold;
        _new = mynew;

     /*   _old.reset(new PointCloudT);
        pcl::copyPointCloud(*myold, *_old);

        _new.reset(new PointCloudT);
        pcl::copyPointCloud(*mynew, *_new);*/

        setText(QString("[%1] %2").arg(ElabName).arg(QString::fromStdString(myold->header.frame_id).section("/",-1,-1)));
    }

    void undo()
    {
        _m->replacePointCloudToViewer(_new, _old);
        _m->replacePointCloudToModel(_new, _old);
    }
    void redo()
    {
        _m->replacePointCloudToViewer(_old, _new);
        _m->replacePointCloudToModel(_old, _new);
    }

private:
    ModelPointCloudList *_m;
    PointCloudT::Ptr _old;
    PointCloudT::Ptr _new;
};



#endif // MODELPOINTCLOUDLIST_H

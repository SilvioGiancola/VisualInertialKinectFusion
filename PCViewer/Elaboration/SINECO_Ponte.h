#ifndef SINECO_Ponte_H
#define SINECO_Ponte_H

#include <QWidget>
#include <define.h>

//Back-End
#include <pcl/registration/lum.h>
#include <pcl/registration/elch.h>

#include <QElapsedTimer>

namespace Ui {
class SINECO_Ponte;
}

class SINECO_Ponte : public QWidget
{
    Q_OBJECT

public:
    explicit SINECO_Ponte(QWidget *parent = 0);
    ~SINECO_Ponte();

private:
    Ui::SINECO_Ponte *ui;
    QList<PointCloudT::Ptr> CurrentPCList;


public slots:
    void setCurrentPCList(QList<PointCloudT::Ptr> pcList) { CurrentPCList = pcList;}

signals:
    void replace(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC, QString ElabName);


private slots:
    void on_pushButton_compute_LUM_clicked();
    void on_pushButton_compute_ELCH_clicked();
};

#endif // SINECO_Ponte_H

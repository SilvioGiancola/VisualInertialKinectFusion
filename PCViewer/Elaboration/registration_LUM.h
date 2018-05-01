#ifndef REGISTRATION_LUM_H
#define REGISTRATION_LUM_H

#include <QWidget>
#include <define.h>

//Back-End
#include <pcl/registration/lum.h>
#include <pcl/registration/elch.h>

#include <QElapsedTimer>

namespace Ui {
class Registration_LUM;
}

class Registration_LUM : public QWidget
{
    Q_OBJECT

public:
    explicit Registration_LUM(QWidget *parent = 0);
    ~Registration_LUM();

private:
    Ui::Registration_LUM *ui;
    QList<PointCloudT::Ptr> CurrentPCList;


public slots:
    void setCurrentPCList(QList<PointCloudT::Ptr> pcList) { CurrentPCList = pcList;}

signals:
    void replace(PointCloudT::Ptr oldPC, PointCloudT::Ptr newPC, QString ElabName);


private slots:
    void on_pushButton_compute_LUM_clicked();
    void on_pushButton_compute_ELCH_clicked();
};

#endif // REGISTRATION_LUM_H

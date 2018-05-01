#ifndef KinFuBatch_H
#define KinFuBatch_H

#include <QMainWindow>





namespace Ui
{
class KinFuBatch;
}

class KinFuBatch : public QMainWindow
{
    Q_OBJECT

public:
    explicit KinFuBatch (QWidget *parent = 0);
    ~KinFuBatch ();


private slots:
    void on_pushButton_RunSingleKinFu_clicked();

private:
    Ui::KinFuBatch *ui;

};

#endif // KinFuBatch_H

#include "KinFuBatch.h"
#include "ui_KinFuBatch.h"




KinFuBatch::KinFuBatch (QWidget *parent) : QMainWindow (parent), ui (new Ui::KinFuBatch)
{
    ui->setupUi (this); 
}

KinFuBatch::~KinFuBatch ()
{
    delete ui;
}

#include <QApplication>

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);

  KinFuBatch w;
  w.show ();

  return a.exec ();
}

void KinFuBatch::on_pushButton_RunSingleKinFu_clicked()
{
    Q
}

#include "transformationmatrixspinbox.h"
#include "ui_transformationmatrixspinbox.h"

TransformationMatrixSpinBox::TransformationMatrixSpinBox(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TransformationMatrixSpinBox)
{
    ui->setupUi(this);
}

TransformationMatrixSpinBox::~TransformationMatrixSpinBox()
{
    delete ui;
}



Eigen::Matrix4f TransformationMatrixSpinBox::getMatrix()
{
    Eigen::Matrix4f mat;
    mat << ui->dSB00->value() , ui->dSB01->value() , ui->dSB02->value() , ui->dSB03->value() ,
           ui->dSB10->value() , ui->dSB11->value() , ui->dSB12->value() , ui->dSB13->value() ,
           ui->dSB20->value() , ui->dSB21->value() , ui->dSB22->value() , ui->dSB23->value() ,
           ui->dSB30->value() , ui->dSB31->value() , ui->dSB32->value() , ui->dSB33->value();
    return mat;
}

void TransformationMatrixSpinBox::setMatrix(Eigen::Matrix4f mat)
{
    ui->dSB00->setValue(mat(0,0));
    ui->dSB01->setValue(mat(0,1));
    ui->dSB02->setValue(mat(0,2));
    ui->dSB03->setValue(mat(0,3));

    ui->dSB10->setValue(mat(1,0));
    ui->dSB11->setValue(mat(1,1));
    ui->dSB12->setValue(mat(1,2));
    ui->dSB13->setValue(mat(1,3));

    ui->dSB20->setValue(mat(2,0));
    ui->dSB21->setValue(mat(2,1));
    ui->dSB22->setValue(mat(2,2));
    ui->dSB23->setValue(mat(2,3));

    ui->dSB30->setValue(mat(3,0));
    ui->dSB31->setValue(mat(3,1));
    ui->dSB32->setValue(mat(3,2));
    ui->dSB33->setValue(mat(3,3));
}

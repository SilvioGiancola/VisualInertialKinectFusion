#ifndef TRANSFORMATIONMATRIXSPINBOX_H
#define TRANSFORMATIONMATRIXSPINBOX_H

#include <QWidget>
#include <Eigen/Dense>

namespace Ui {
class TransformationMatrixSpinBox;
}

class TransformationMatrixSpinBox : public QWidget
{
    Q_OBJECT

public:
    explicit TransformationMatrixSpinBox(QWidget *parent = 0);
    ~TransformationMatrixSpinBox();

    Eigen::Matrix4f getMatrix();
    void setMatrix(Eigen::Matrix4f mat);

private:
    Ui::TransformationMatrixSpinBox *ui;
};

#endif // TRANSFORMATIONMATRIXSPINBOX_H

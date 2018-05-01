#include "filters_others.h"
#include "ui_filters_others.h"

Filters_Others::Filters_Others(QWidget *parent) :
    QWidget(parent), ui(new Ui::Filters_Others)
{
    ui->setupUi(this);
}

Filters_Others::~Filters_Others()
{
    delete ui;
}



void Filters_Others::on_pushButton_RemoveNan_clicked()
{        
    PointCloudT::Ptr input(CurrentPC);
    PointCloudT::Ptr output(new PointCloudT);
    pcl::copyPointCloud(*input, *output);

    std::vector<int> ind;
    pcl::removeNaNFromPointCloud(*input, *output, ind);

    emit replace(input, output, QString("Remove Nan"));
}

void Filters_Others::on_pushButton_Scale_clicked()
{
    PointCloudT::Ptr input(CurrentPC);
    PointCloudT::Ptr output(new PointCloudT);
    pcl::copyPointCloud(*input, *output);

    // From factor
    double from = 1;
    if (ui->comboBox_from->currentText() == "m")
        from = 1;
    else if (ui->comboBox_from->currentText() == "mm")
        from = 1000;

    // To Factor
    double to = 1;
    if (ui->comboBox_to->currentText() == "m")
        to = 1;
    else if (ui->comboBox_to->currentText() == "mm")
        to = 1000;



    for (int i = 0; i < CurrentPC->size(); i++)
    {
        output->at(i).x = input->at(i).x * to / from;
        output->at(i).y = input->at(i).y * to / from;
        output->at(i).z = input->at(i).z * to / from;
    }


    emit replace(input, output, QString("Scaling"));

    return;
}

void Filters_Others::on_pushButton_SetColor_clicked()
{
    PointCloudT::Ptr input(CurrentPC);
    PointCloudT::Ptr output(new PointCloudT);
    pcl::copyPointCloud(*input, *output);


    bool ok;
    QRgb rgb = QColorDialog::getRgba(lastColor, &ok);

    if (ok)
    {
        lastColor = rgb;
        for (int i = 0; i <output->size(); i++)
        {
            output->at(i).r = qRed(rgb);
            output->at(i).g = qGreen(rgb);
            output->at(i).b = qBlue(rgb);
            output->at(i).a = qAlpha(rgb);
        }
    }


    emit replace(input, output, QString("Scaling"));

    return;
}

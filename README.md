# Integration of Absolute Orientation Measurements in the KinectFusion Reconstruction Pipeline

CVPR Workshop on Visual Odometry and Computer Vision Applications Based on Location Clues 2018

Available at [openaccess.thecvf.com](http://openaccess.thecvf.com/content_cvpr_2018_workshops/papers/w30/Giancola_Integration_of_Absolute_CVPR_2018_paper.pdf)

```
@InProceedings{Giancola_2018_CVPR_Workshops,
  author = {Giancola, Silvio and Schneider, Jens and Wonka, Peter and Ghanem, Bernard S.},
  title = {Integration of Absolute Orientation Measurements in the KinectFusion Reconstruction Pipeline},
  booktitle = {The IEEE Conference on Computer Vision and Pattern Recognition (CVPR) Workshops},
  month = {June},
  year = {2018}
}
```
# Implementation

Tested with:
 - VTK 6.2, 
 - Boost 1.58, 
 - Eigen 3,
 - FLANN,
 - Qt 5.9

## Install a modified version of PCL
```
mkdir pcl-build && cd pcl-build/
cmake -DBUILD_GPU=ON -DBUILD_CUDA=ON -DWITH_QT=OFF ../pcl
make -j8
sudo make install
cd ..
```


## Install a modified version of PCViewer
```
mkdir PCViewer-build && cd PCViewer-build
cmake ../PCViewer
make -j8 testKinFuGUI
cd ..
```



## Create dataset
 - Download the sequences from the [freiburg dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset/download)
 - Extract the sequences into the data folder
 - Activate the conda environment 
`conda env create -f environment.yml && conda activate TUMdataset`
 - Copy python script into sequence folder 
 `cp *.py {sequencename} && cd {sequencename}` 
 with {sequencename} being the name of the folder for a given sequence
 - Extract the orientated point cloud 
 `mkdir pointclouds && python generate_registered_pointcloud_organized.py rgb.txt depth.txt groundtruth.txt pointclouds/PC --pcd_format` 
 cd ../..

## Run Kinect Fusion
```
cd PCViewer
./testKinFuGUI
```

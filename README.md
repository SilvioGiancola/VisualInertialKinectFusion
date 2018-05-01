# VisualInertialKinectFusion
Integration of Absolute Orientation Measurements in the KinectFusion Reconstruction pipeline



tested with:
 - VTK 6.2, 
 - Boost 1.58, 
 - Eigen 3,
 - FLANN,
 - Qt5.9

## Install a modified version of PCL
```
mkdir pcl-build && cd pcl-build/
cmake -DBUILD_GPU=ON -DBUILD_CUDA=ON -DWITH_QT=OFF ../pcl
make -j8
sudo make install
cd ..
```


## Install s odified version of PCViewer
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

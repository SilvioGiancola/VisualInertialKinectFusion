# VisualInertialKinectFusion
Integration of Absolute Orientation Measurements in the KinectFusion Reconstruction pipeline



need VTK 6.2, Boost1.58, Eigen3, FLANN, Qt5.9
identify the architecture of your GPU

cd pcl
mkdir build
cd build/

cmake -DBUILD_GPU=ON -DBUILD_CUDA=ON -DWITH_QT=OFF -DCUDA_ARCH_BIN="3.0 3.5 5.0 5.2 6.0 6.1" ..

make -j8

sudo make install




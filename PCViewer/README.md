# PCViewer PROJECT
This project aims to unleash the tools provided by the PCL library in a GUI.


# PCViewer
Point Cloud Visualizer
Provide simple tools for visualizing and editing point clouds, as well as a grabber for the Kinect V2
./PCViewer

Top MenuBar:
-> Open / Close / Copy / Save point clouds
-> Show / Hide / Merge point clouds, reference system, ground
-> experimental tools

Left Tab:
-> Point Cloud List for tuning point cloud visualization
-> UndoStacked Widget for undo/redo framework

Right Tab:
-> Kinect2: Point Cloud Grabber wiht Adafruit/myAHRS+/NexCave poses
-> MLS: Moving Least Square: Smooth the point cloud
-> Outliers: filter outliers based on Radius/Statistical outliers (see Radu Rusu PhDThesis)
-> Voxel Grid Tool: 
-> Others: Scaling factor / Remove NaN Point / Change Color Point cloud
-> Transformation: Set pose (4x4 matrix of selected point cloud)
-> Normal: compute normals for point clouds
-> Keypoints: extract keypoint from selected point cloud

-> ICP: Align 2 point clouds using the ICP pipeline:
	* Select 2 point clouds (Model / Point Cloud to align)
	* Select Downsampling (Random Sampling / VoxelGrid / Full Point Cloud / [BRISK] / BRISK quadratic interpolation / AGAST / SIFT 3D)
	* Select a Transformation method ([SVD-based] / Levenberg-Marquardt / Point-to-Plane / Linearized Point-to-Plane / Translation only)
	* Select a Correspondences estimation ([Base] / Back projection / Normal Shooting / Organized Projection) and activate the reciprocal correspondences
	* Select a Correspondences rejection ([Max Distances=0.100] / [One-to-One] / Median Distance / Sample Consensus / Trimmed / VarTrimmed) 
	* Select the number of iteration and start elaboration (note: doing 2 ICP with 10 iteration is equivalent to doing 1 ICP with 20 iteration)
	* Can save times and metrics while aligning

-> RANSAC : Align 2 point clouds using RANSAC pipeline:
	* Select 2 point clouds (Model / Point Cloud to align)
	* Select Downsampling (Random Sampling / VoxelGrid / Full Point Cloud / [BRISK] / BRISK quadratic interpolation / AGAST / SIFT 3D)
	* Select Description: not setup yet: Currently using PFHRGB and need to recompiling for changing it
	* Select a Correspondences estimation ([Base] / Back projection / Normal Shooting / Organized Projection) and activate the reciprocal correspondences
	* Select a Correspondences rejection ([Max Distances=0.100] / [One-to-One] / Median Distance / Sample Consensus / Trimmed / VarTrimmed) 
	* Select the maximum numer of iteration and threshold for point to be consider as inliers (0.020m / 100iters)
	* Select the RANSAC Model ([SVD] / TranslationOnly)

-> LUM : Minimize overall error between multiple point clouds
	* Select correpondences esitmation and rejection ([Base] / [MaxCorres=0.1]) and [Brisk] keypoints
	* Start with 1 iteration and compute LUM until convergence (no moe visible refinement)

-> Adafruit: Buffer to save poses for calibration
	* Grab point clouds
	* Save IMU poses
	* Align furthermore (manually + LUM)
	* Save registration Poses
	* Go to matlab








# KinFu
Provide a single KinFu reconstruction
./testKinFu <opt>
opt:
--help, -h                                         : print help
--viz <bool> (default:1)                           : enable visualisation
--volume_size <size_in_meters> (default:3)         : define integration volume size
--resolution <voxel_number_per_side> (default:512) : define voxel resolution
--icp1 <iteration number> (default:10)             : define coarsest icp iteration number
--icp2 <iteration number> (default:5)              : define intermediate icp iteration number
--icp3 <iteration number> (default:4)              : define fine icp iteration number
--IMU <bool> (default:1)                           : enable IMU


original KinFu: 
  ./testKinFu --viz 1 --IMU 0 --volume_size 3 --resolution 512 --icp1 10 --icp2 5 --icp3 4

IMU-Initialized KinFu: 
  ./testKinFu --viz 1 --IMU 1 --volume_size 3 --resolution 512 --icp1 10 --icp2 5 --icp3 4


Original KinFu timing (no visualization):
  ./testKinFu --viz 0 --IMU 0 --volume_size 3 --resolution 512 --icp1 10 --icp2 5 --icp3 4

IMU-Initialized Kinfu timing (Real-Time on a laptop):
  ./testKinFu --viz 0 --IMU 1 --volume_size 3 --resolution 512 --icp1 3 --icp2 2 --icp3 2



High-resolution Original Kinfu:
  ./testKinFu --viz 1 --IMU 0 --volume_size 1 --resolution 1024 --icp1 10 --icp2 5 --icp3 4

High-resolution IMU-Initialized Kinfu:
  ./testKinFu --viz 1 --IMU 1 --volume_size 1 --resolution 1024 --icp1 10 --icp2 5 --icp3 4



Shortcut:
 a,A : Display mesh from TSDF (download from GPU to CPU)
 p,P : Pause KinFu intergration
 i,I : Unlock point of view (on: allow movement in 3D, off: allow raycasting in 2D)
 d,D : switch IMU mode
   7 : Save current model (as seen in 3D viewer) in a file (same folder than executable)

To get the complete command list, press 'h' in one of the windows











# Double KinFu
Provide 2 KinFu reconstruction on the same machine, fed with the same depth stream.
./test2KinFu <opt>
opt:
--help, -h                                         : print help
--viz <bool> (default:1)                           : enable visualisation
--volume_size <size_in_meters> (default:3)         : define integration volume size
--resolution <voxel_number_per_side> (default:512) : define voxel resolution
--icp1 <iteration number> (default:10)             : define coarsest icp iteration number
--icp2 <iteration number> (default:5)              : define intermediate icp iteration number
--icp3 <iteration number> (default:4)              : define fine icp iteration number

Note: in order to test different icp iterations between the 2 KinFu, it needs to be recompiled changing parameters in KinFu/test2KinFu.cpp line 717:
	kinfuDEV_.setICPiteration(icp1_,icp2_,icp3_); 	// Iteration for IMU-initialized KinFu
	kinfu_.setICPiteration(icp1_,icp2_,icp3_);	// Iteration for Original KinFu



# Adafruit IMU
Show the orientation of the Adafruit IMU in real time
./testAdafruit


# Kinect 2 live
Show a live streaming of the Kinect 2 
./testKinect2

# Kinect 2 live + Adafruit orientation
Show a live streaming of the Kinect 2 with the adafruit orientation
./testKinect2Adafruit


# NexCave pose
Show a live streaming of any NexCave tracker
./testNexCave




# Compilation
Require: PCL 1.8 / OpenCV 3.1.0 / VTK 6.2 / libfreenect2 / CUDA 7.0 / VRPN / QT5
cd PCViewer/..
mkdir PCViewer-build-Release && cd PCViewer-build-Release
cmake -DCMAKE_BUILD_TYPE=Release 
make -j4





#CUDA_ADD_EXECUTABLE(testKinFuGUI testKinFuGUI.cpp
#KinFuWindow.cpp KinFuWindow.h KinFuWindow.ui
#../src/tsdf_volume.h ../src/tsdf_volume.cpp ../src/tsdf_volume.hpp
#raycaster.h raycaster.cpp
#marching_cubes.h marching_cubes.cpp
#safe_call.hpp
#internal.h
#camera_pose.h
#KinFuApp.h
#Devices/adafruit_uart.h)
#TARGET_LINK_LIBRARIES(testKinFuGUI ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} )



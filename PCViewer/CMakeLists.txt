cmake_minimum_required (VERSION 2.8.12 FATAL_ERROR)

# DEFINE THE PROJECT
PROJECT(PCViewer)









# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
# Executables fail to build with Qt 5 in the default configuration
# without -fPIE. We add that here.
#set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
#set(CMAKE_CXX_FLAGS "${Qt5SerialPort_EXECUTABLE_COMPILE_FLAGS}")

SET(CMAKE_CXX_FLAGS "-std=c++11")






# Widgets finds its own dependencies (QtGui and QtCore).
find_package(Qt5Widgets REQUIRED)
find_package(Qt5SerialPort REQUIRED)


# find freenect2
find_package(freenect2 REQUIRED)
#include_directories(${freenect2_INCLUDE_DIR})
#link_directories(${freenect2_LIBRARY})
#add_definitions(${PCL_DEFINITIONS})


find_package(OpenCV 2.4.9 REQUIRED) # m ay be 3.1.0
include_directories(${OpenCV_INCLUDE_DIRS})
#set(OpenCV_LIBS opencv_core opencv_imgproc opencv_calib3d opencv_video opencv_features2d opencv_ml opencv_highgui opencv_objdetect opencv_contrib opencv_legacy opencv_gpu)
IF(OpenCV_FOUND)
   MESSAGE(STATUS "Found OpenCV: ${OpenCV_INCLUDE_DIRS} VERSION : ${OpenCV_VERSION}")
    MESSAGE(STATUS ${OpenCV_LIBS})
ENDIF(OpenCV_FOUND)
link_directories(${OpenCV_INCLUDE_DIRS})
add_definitions(${OpenCV_DEFINITIONS})


# find VTK
set(VTK_DIR "/usr/local/lib")
find_package(VTK 6.2 REQUIRED)
include(${VTK_USE_FILE})

# find PCL
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



# find CUDA
find_package(CUDA 7.5 REQUIRED)
#Include the FindCUDA script
INCLUDE(FindCUDA)
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARIES})
add_definitions(${CUDA_DEFINITIONS})
#include_directories(/usr/local/cuda-7.5/include/)
#link_directories(/usr/local/cuda-7.5/lib64/)
#add_definitions(${CUDA_DEFINITIONS})


# find VRPN
set(VRPN_ROOT_DIR "/usr/local/lib")
#find_package(VRPN REQUIRED)

set(VRPN_ROOT_DIR
 "${VRPN_ROOT_DIR}"
 CACHE
 PATH
 "/usr/local/lib")
find_path(VRPN_INCLUDE_DIR
 NAMES
 vrpn_Connection.h
 PATH_SUFFIXES
 include
 include/vrpn
 HINTS
 "${VRPN_ROOT_DIR}"
 PATHS
 "${_progfiles}/VRPN")

find_library(VRPN_LIBRARY
 NAMES
 vrpn
 PATH_SUFFIXES
 ${_libsuffixes}
 HINTS
 "${VRPN_ROOT_DIR}"
 PATHS
 "${_progfiles}/VRPN")

find_library(VRPN_SERVER_LIBRARY
 NAMES
 vrpnserver
 PATH_SUFFIXES
 ${_libsuffixes}
 HINTS
 "${VRPN_ROOT_DIR}"
 PATHS
 "${_progfiles}/VRPN")

include_directories(${VRPN_INCLUDE_DIRS})
link_directories(${VRPN_LIBRARY_DIRS})
add_definitions(${VRPN_DEFINITIONS})



# include project
include_directories(../PCViewer) # include the project in se
IF("${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CMAKE_BUILD_TYPE Release)
ENDIF()
IF("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    include_directories(../PCViewer-build-Debug) # include the project in se
ENDIF()
IF("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    include_directories(../PCViewer-build-Release) # include the project in se
ENDIF()



#include_directories("/usr/local/MATLAB/R2015b/extern/include") # include matalb
#include_directories("/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms")










# ALL WIDGETS
FILE(GLOB Widgets_SOURCES Widgets/*.cpp Widgets/*.h)
FILE(GLOB Widgets_FORMS Widgets/*.ui)
QT5_WRAP_UI(Widgets_FORMS_HEADERS ${Widgets_FORMS})

# ALL ELABORATION
FILE(GLOB Elaboration_SOURCES Elaboration/*.cpp Elaboration/*.h Elaboration/*.hpp Elaboration/*/*.cpp Elaboration/*/*.h)
FILE(GLOB Elaboration_FORMS Elaboration/*.ui Elaboration/*/*.ui)
QT5_WRAP_UI(Elaboration_FORMS_HEADERS ${Elaboration_FORMS})

#ALL DEVICES
FILE(GLOB Devices_SOURCES Devices/*.cpp Devices/*.h Devices/*.hpp)
FILE(GLOB Devices_FORMS Devices/*.ui)
QT5_WRAP_UI(Devices_FORMS_HEADERS ${Devices_FORMS})

#ALL RESOURCES
SET(Icon_RESOURCES Icon/Icon.qrc)
QT5_ADD_RESOURCES(Icon_RESOURCES_RCC ${Icon_RESOURCES})

# PCVIEWER
FILE(GLOB PCViewer_SOURCES PCViewer/*.cpp PCViewer/*.h PCViewer/*.hpp)
FILE(GLOB PCViewer_FORMS PCViewer/*.ui)
QT5_WRAP_UI(PCViewer_FORMS_HEADERS ${PCViewer_FORMS})

# PCVIEWER
FILE(GLOB PCViewer2_SOURCES PCViewer2/*.cpp PCViewer2/*.h PCViewer2/*.hpp)
FILE(GLOB PCViewer2_FORMS PCViewer2/*.ui)
QT5_WRAP_UI(PCViewer2_FORMS_HEADERS ${PCViewer2_FORMS})


# GENERAL CALIBRATION
FILE(GLOB Calibration_SOURCES Calibration/*.cpp Calibration/*.h Calibration/*.hpp)
FILE(GLOB Calibration_FORMS Calibration/*.ui)
QT5_WRAP_UI(Calibration_FORMS_HEADERS ${Calibration_FORMS})

# ADAFRUIT CALIBRATION
FILE(GLOB AdaFruitCalibration_SOURCES AdaFruitCalibration/*.cpp AdaFruitCalibration/*.h AdaFruitCalibration/*.hpp)
FILE(GLOB AdaFruitCalibration_FORMS AdaFruitCalibration/*.ui)
QT5_WRAP_UI(AdaFruitCalibration_FORMS_HEADERS ${AdaFruitCalibration_FORMS})

# KINFU GUI
FILE(GLOB KinFuGUI_SOURCES KinFu/*.cpp KinFu/*.h KinFu/*.hpp)
FILE(GLOB KinFuGUI_FORMS KinFu/KinFuWindow.ui)
QT5_WRAP_UI(KinFuGUI_FORMS_HEADERS ${KinFuGUI_FORMS})

## KINFU GUI 2
#FILE(GLOB KinFuGUI2_SOURCES KinFu/*.cpp KinFu/*.h KinFu/*.hpp)
#FILE(GLOB KinFuGUI2_FORMS KinFu/KinFuWindowLight.ui)
#QT5_WRAP_UI(KinFuGUI2_FORMS_HEADERS ${KinFuGUI2_FORMS})


#FILE(GLOB PCViewer2_SOURCES PCViewer/*.cpp PCViewer2/*.h PCViewer2/*.hpp)
#FILE(GLOB PCViewer2_FORMS PCViewer2/*.ui)
#QT5_WRAP_UI(PCViewer2_FORMS_HEADERS ${PCViewer2_FORMS})






#find_package(Qt5 REQUIRED COMPONENTS Widgets Gui Core SerialPort) # component of QT5
#QT5_WRAP_CPP(PCViewer_HEADERS_MOC ${PCViewer_HEADERS})

#QT5_USE_MODULES(PCViewer Widgets Core Gui SerialPort)



# MAIN APPLICATION
ADD_EXECUTABLE(PCViewer define.h
${Widgets_SOURCES} ${Widgets_FORMS_HEADERS}
${Elaboration_SOURCES} ${Elaboration_FORMS_HEADERS}
${Devices_SOURCES} ${Devices_FORMS_HEADERS}
${Icon_RESOURCES_RCC}
${PCViewer_SOURCES} ${PCViewer_FORMS_HEADERS} )

TARGET_LINK_LIBRARIES(PCViewer ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARY} ${VRPN_LIBRARY} ${OpenCV_LIBS})
#/usr/local/MATLAB/R2015b/bin/glnxa64/libmx.so /usr/local/MATLAB/R2015b/bin/glnxa64/libeng.so	)



# MAIN APPLICATION
ADD_EXECUTABLE(PCViewer2 define.h
${Widgets_SOURCES} ${Widgets_FORMS_HEADERS}
${Elaboration_SOURCES} ${Elaboration_FORMS_HEADERS}
${Devices_SOURCES} ${Devices_FORMS_HEADERS}
${Icon_RESOURCES_RCC}
${PCViewer2_SOURCES} ${PCViewer2_FORMS_HEADERS} )

TARGET_LINK_LIBRARIES(PCViewer2 ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARY} ${VRPN_LIBRARY} ${OpenCV_LIBS})




## MAIN APPLICATION 2
#ADD_EXECUTABLE(PCViewer2
#${Widgets_SOURCES} ${Widgets_FORMS_HEADERS}
#${Elaboration_SOURCES} ${Elaboration_FORMS_HEADERS}
#${Devices_SOURCES} ${Devices_FORMS_HEADERS}
#${Icon_RESOURCES_RCC}
#${PCViewer2_SOURCES} ${PCViewer2_FORMS_HEADERS} )

#TARGET_LINK_LIBRARIES(PCViewer2 ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARY} ${OpenCV_LIBS})





# ADAFRUIT TEST
ADD_EXECUTABLE(testAdafruit Tests/testAdaFruit.cpp Devices/adafruit_uart.h )
TARGET_LINK_LIBRARIES(testAdafruit ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES})


# MYAHRS TEST
ADD_EXECUTABLE(testmyAHRS Tests/testMyAHRS.cpp Devices/myahrs_plus.hpp )
TARGET_LINK_LIBRARIES(testmyAHRS ${Qt5Widgets_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES})


# NEXCAVE Test
ADD_EXECUTABLE(testNexCave Tests/testNexCave.cpp)
TARGET_LINK_LIBRARIES(testNexCave ${Qt5Widgets_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${VRPN_LIBRARY})



# Kinect1 Test
ADD_EXECUTABLE(testKinect Tests/testKinect.cpp)
TARGET_LINK_LIBRARIES(testKinect ${PCL_LIBRARIES})


# Kinect1 Image Viewer Test
ADD_EXECUTABLE(testKinect2D Tests/testKinect2D.cpp)
TARGET_LINK_LIBRARIES(testKinect2D ${PCL_LIBRARIES})


# Kinect1 GUI Test
QT5_WRAP_UI(testKinectGUI_FORMS_HEADERS Tests/KinectViewer.ui)
ADD_EXECUTABLE(testKinectGUI Tests/testKinectGUI.cpp Tests/KinectViewer.cpp Tests/KinectViewer.h ${testKinectGUI_FORMS_HEADERS})
TARGET_LINK_LIBRARIES(testKinectGUI ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES})


# Kinect2 Test
ADD_EXECUTABLE(testKinect2 Tests/testKinect2.cpp Devices/kinect.h Devices/kinect.cpp Devices/adafruit_uart.h)
TARGET_LINK_LIBRARIES(testKinect2 ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARY} ${OpenCV_LIBS})

# Kinect2 + Adafruit Test
ADD_EXECUTABLE(testKinect2Adafruit Tests/testKinect2Adafruit.cpp Devices/kinect.h Devices/kinect.cpp Devices/adafruit_uart.h)
TARGET_LINK_LIBRARIES(testKinect2Adafruit ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARY} ${OpenCV_LIBS})


# MultipleKinect2 Test
ADD_EXECUTABLE(testMultipleKinect2 Tests/testMultipleKinect2.cpp Devices/kinect.h Devices/kinect.cpp Devices/adafruit_uart.h)
TARGET_LINK_LIBRARIES(testMultipleKinect2 ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARY} ${OpenCV_LIBS})

# MultipleKinect2 Test
ADD_EXECUTABLE(testBRISK Tests/testBRISK.cpp)
TARGET_LINK_LIBRARIES(testBRISK ${PCL_LIBRARIES})



## KinFu double TSDF
CUDA_ADD_EXECUTABLE(test2KinFu KinFu/test2KinFu.cpp
KinFu/kinfu.cpp KinFu/kinfu.h
KinFu/tsdf_volume.h KinFu/tsdf_volume.cpp KinFu/tsdf_volume.hpp
KinFu/raycaster.h KinFu/raycaster.cpp
KinFu/marching_cubes.h KinFu/marching_cubes.cpp
KinFu/safe_call.hpp
KinFu/internal.h
KinFu/camera_pose.h
Devices/adafruit_uart.h)
TARGET_LINK_LIBRARIES(test2KinFu ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES})


## KinFu simple cube
CUDA_ADD_EXECUTABLE(testKinFu KinFu/testKinFu.cpp
KinFu/kinfu.cpp KinFu/kinfu.h
KinFu/tsdf_volume.h KinFu/tsdf_volume.cpp KinFu/tsdf_volume.hpp
KinFu/raycaster.h KinFu/raycaster.cpp
KinFu/marching_cubes.h KinFu/marching_cubes.cpp
KinFu/safe_call.hpp
KinFu/internal.h
KinFu/camera_pose.h
Devices/adafruit_uart.h)
TARGET_LINK_LIBRARIES(testKinFu ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES})




## KinFu simple cube with KinFuApp
CUDA_ADD_EXECUTABLE(testKinFu2 KinFu/testKinFu2.cpp
KinFu/kinfu.cpp KinFu/kinfu.h
KinFu/tsdf_volume.h KinFu/tsdf_volume.cpp KinFu/tsdf_volume.hpp
KinFu/raycaster.h KinFu/raycaster.cpp
KinFu/marching_cubes.h KinFu/marching_cubes.cpp
KinFu/safe_call.hpp
KinFu/internal.h
KinFu/camera_pose.h
KinFu/KinFuApp.h
Devices/adafruit_uart.h)
TARGET_LINK_LIBRARIES(testKinFu2 ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES})




## KinFu GUI
CUDA_ADD_EXECUTABLE(testKinFuGUI KinFu/testKinFuGUI.cpp
KinFu/KinFuWindow.cpp KinFu/KinFuWindow.h KinFu/KinFuWindow.ui
#KinFu/kinfu.cpp KinFu/kinfu.h
KinFu/tsdf_volume.h KinFu/tsdf_volume.cpp KinFu/tsdf_volume.hpp
KinFu/raycaster.h KinFu/raycaster.cpp
KinFu/marching_cubes.h KinFu/marching_cubes.cpp
KinFu/safe_call.hpp
KinFu/internal.h
KinFu/camera_pose.h
KinFu/KinFuApp.h
Devices/adafruit_uart.h)
TARGET_LINK_LIBRARIES(testKinFuGUI ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES}  /usr/local/lib/librealsense.so)





## KinFu GUI 2
CUDA_ADD_EXECUTABLE(testKinFuGUI2 KinFu/testKinFuGUI2.cpp
KinFu/KinFuWindowLight.cpp KinFu/KinFuWindowLight.h KinFu/KinFuWindowLight.ui
KinFu/kinfu.cpp KinFu/kinfu.h
KinFu/tsdf_volume.h KinFu/tsdf_volume.cpp KinFu/tsdf_volume.hpp
KinFu/raycaster.h KinFu/raycaster.cpp
KinFu/marching_cubes.h KinFu/marching_cubes.cpp
KinFu/safe_call.hpp
KinFu/internal.h
KinFu/camera_pose.h
Devices/adafruit_uart.h)
TARGET_LINK_LIBRARIES(testKinFuGUI2 ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES})





### SLAM
#IF("${OpenCV_VERSION}" STREQUAL "2.4.8")
#    ADD_EXECUTABLE(testSLAM SLAM/testSLAM.cpp
#    Devices/adafruit_uart.h)
#    TARGET_LINK_LIBRARIES(testSLAM ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARY} ${OpenCV_LIBS})
#ENDIF ()



### KinFu2
#CUDA_ADD_EXECUTABLE(myKinFu KinFu2/myKinFu.cpp KinFu2/evaluation.cpp KinFu2/evaluation.h KinFu2/kinfu.cpp KinFu2/kinfu.h Devices/adafruit_uart.h)
#TARGET_LINK_LIBRARIES(myKinFu ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES})




# GPU TEst
#CUDA_ADD_EXECUTABLE(testGPU Tests/testGPU.cpp )
#TARGET_LINK_LIBRARIES(testGPU ${PCL_LIBRARIES})


# General Calibration
ADD_EXECUTABLE(Calibration
${Widgets_SOURCES} ${Widgets_FORMS_HEADERS}
${Calibration_SOURCES} ${Calibration_FORMS_HEADERS}
${Devices_SOURCES})
TARGET_LINK_LIBRARIES(Calibration ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${VRPN_LIBRARY} ${freenect2_LIBRARY} ${OpenCV_LIBS})



## AdaFruit Calibration
#ADD_EXECUTABLE(AdaFruitCalibration ${AdaFruitCalibration_SOURCES} ${AdaFruitCalibration_FORMS_HEADERS} Devices/adafruit_uart.h Devices/kinect.h Devices/kinect.cpp)
#TARGET_LINK_LIBRARIES(AdaFruitCalibration ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARY} ${OpenCV_LIBS})



# MEtrological Qualification
ADD_EXECUTABLE(metrologicalQualification Tests/metrologicalQualification.cpp Devices/adafruit_uart.h)
TARGET_LINK_LIBRARIES(metrologicalQualification ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${VRPN_LIBRARY})







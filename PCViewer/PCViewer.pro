#-------------------------------------------------
#
# Project created by QtCreator 2014-12-03T17:04:42
#
#-------------------------------------------------

QT       += core gui widgets serialport #uitools
#LIBS += -lQtGui -lQtCore -lQtOpenGL

TARGET = PCViewer
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

QMAKE_CXX = ccache g++

SOURCES += mainwindow.cpp\
    Kinect/kinecttab.cpp\
    Kinect/kinect.cpp\
    Widgets/transformationmatrixspinbox.cpp\
    Elaboration/Filters/filters_mls.cpp\
    Elaboration/Filters/filters_roi.cpp\
    Elaboration/Filters/filters_voxel_grid.cpp\
    Elaboration/Filters/filters_outliers.cpp\
    Elaboration/Filters/filters_others.cpp\
    modelpointcloudlist.cpp\
    Elaboration/transformation.cpp\
    Elaboration/registration_icp.cpp\
    Elaboration/features_normal.cpp\
    Elaboration/features_keypoints.cpp\
    Elaboration/elaboration_lf_rec.cpp\
    Elaboration/elaboration_papazov.cpp\
    Elaboration/registration_keypoints.cpp\
    Elaboration/TransformationEstimationTranslationOnly.hpp

HEADERS  +=   mainwindow.h\
    define.h\
    Kinect/kinecttab.h\
    Kinect/kinect.h\
    Kinect/adafruit_uart.h\
    Widgets/transformationmatrixspinbox.h\
    Elaboration/Filters/filters_mls.h\
    Elaboration/Filters/filters_roi.h\
    Elaboration/Filters/filters_voxel_grid.h\
    Elaboration/Filters/filters_outliers.h\
    Elaboration/Filters/filters_others.h\
    modelpointcloudlist.h\
    Elaboration/transformation.h\
    Elaboration/registration_icp.h\
    Elaboration/features_normal.h\
    Elaboration/features_keypoints.h\
    Elaboration/elaboration_lf_rec.h\
    Elaboration/elaboration_papazov.h\
    Elaboration/registration_keypoints.h\
    Elaboration/TransformationEstimationTranslationOnly.h

FORMS    +=     mainwindow.ui\
    Kinect/kinecttab.ui\
    Widgets/transformationmatrixspinbox.ui\
    Elaboration/Filters/filters_mls.ui\
    Elaboration/Filters/filters_roi.ui\
    Elaboration/Filters/filters_voxel_grid.ui\
    Elaboration/Filters/filters_outliers.ui\
    Elaboration/Filters/filters_others.ui\
    Elaboration/transformation.ui\
    Elaboration/registration_icp.ui\
    Elaboration/features_normal.ui\
    Elaboration/features_keypoints.ui\
    Elaboration/elaboration_lf_rec.ui\
    Elaboration/elaboration_papazov.ui\
    Elaboration/registration_keypoints.ui




CONFIG += link_pkgconfig
PKGCONFIG += /home/silvio/git/libfreenect2/depends/libusb/lib/pkgconfig/libusb-1.0.pc

#################
## Libfreenect2
################


LIBS += -L/usr/local/lib -lfreenect2

INCLUDEPATH += /usr/local/include
DEPENDPATH += /usr/local/include
INCLUDEPATH += /usr/local/include/libfreenect2/tinythread
DEPENDPATH += /usr/local/include/libfreenect2/tinythread




###########################
## HOW TO ADD LIBRARY
###########################
## win32: significa che la lib sara caricata unicamente in ambito Windows
## unix: significa che la lib sara caricata unicamente in ambito Linux
## CONFIG(release, debug|release): significa che la lib sara caricata unicamente con un build di tipo release
## CONFIG(debug, debug|release): significa che la lib sara caricata unicamente con un build di tipo debug
## -L definisce il percoso dove trovare la lib
## -l definisce la libreria
##\ permette di andare alla linea senza rompere la riga di codice
## INCLUDEPATH serve per cercare il file header (.h) della libreria
## DEPENDPATH serve per cercare il file header (.h) della libreria



##################
## LIB PCL
##################
## find lib of PCL 1.8 for release
win32:CONFIG(release, debug|release): LIBS += -L"C://Program Files//PCL//lib//"\
-lpcl_common_release -lpcl_features_release -lpcl_filters_release -lpcl_io_release -lpcl_io_ply_release -lpcl_kdtree_release -lpcl_keypoints_release\
-lpcl_ml_release -lpcl_octree_release -lpcl_outofcore_release -lpcl_people_release -lpcl_recognition_release -lpcl_registration_release\
-lpcl_sample_consensus_release -lpcl_search_release -lpcl_segmentation_release -lpcl_stereo_release -lpcl_surface_release -lpcl_tracking_release\
-lpcl_visualization_release
# and for debug
else:win32:CONFIG(debug, debug|release): LIBS += -L"C://Program Files//PCL//lib//"\
-lpcl_common_debug -lpcl_features_debug -lpcl_filters_debug -lpcl_io_debug -lpcl_io_ply_debug -lpcl_kdtree_debug -lpcl_keypoints_debug -lpcl_ml_debug\
-lpcl_octree_debug -lpcl_outofcore_debug -lpcl_people_debug -lpcl_recognition_debug -lpcl_registration_debug -lpcl_sample_consensus_debug\
-lpcl_search_debug -lpcl_segmentation_debug -lpcl_stereo_debug -lpcl_surface_debug -lpcl_tracking_debug -lpcl_visualization_debug
#for linux
else:unix: LIBS += -L/usr/local/lib/\
-lpcl_common -lpcl_features -lpcl_filters -lpcl_io -lpcl_io_ply -lpcl_kdtree -lpcl_keypoints -lpcl_ml\
-lpcl_octree -lpcl_outofcore -lpcl_people -lpcl_recognition -lpcl_registration -lpcl_sample_consensus\
-lpcl_search -lpcl_segmentation -lpcl_stereo -lpcl_surface -lpcl_tracking -lpcl_visualization #\
#-lpcl_gpu_containers -lpcl_gpu_features -lpcl_gpu_utils -lpcl_cuda_features -lpcl_gpu_octree -lpcl_gpu_segmentation #\
#-lpcl_gpu_surface -lpcl_gpu_tracking -lpcl_gpu_kinfu -lpcl_gpu_kinfu_large_scale -lpcl_cuda_io

# path dei file header (.h) di PCL
win32:INCLUDEPATH += "C://Program Files//PCL//include//pcl-1.8"
else:unix:INCLUDEPATH += /usr/local/include/pcl-1.8

win32:DEPENDPATH += "C://Program Files//PCL//include//pcl-1.8"
else:unix:INCLUDEPATH += /usr/local/include/pcl-1.8





##################
## LIB Eigen
##################
## Eigen è una libreria "header only" cioè non ci sono .lib, tutto è scritto nel file header
## path dei file header (.h) di Eigen
win32:INCLUDEPATH += "C://Program Files (x86)//Eigen//include"
else:unix:INCLUDEPATH += /usr/include/eigen3
win32:DEPENDPATH += "C://Program Files (x86)//Eigen//include"
else:unix:DEPENDPATH += /usr/include/eigen3



#################
# LIB FLANN
#################
LIBS += -L/usr/lib/ -lflann_cpp
INCLUDEPATH += /usr/include/flann
DEPENDPATH += /usr/include/flann





#################
# LIB Boost
#################
# find lib of Boost 1.56 for release
win32:CONFIG(release, debug|release): LIBS += -L"C://Program Files//Boost//lib//" -llibboost_timer-vc120-mt-1_56 -llibboost_system-vc120-mt-1_56
# and for debug
else:win32:CONFIG(debug, debug|release): LIBS += -L"C://Program Files//Boost//lib//" -llibboost_timer-vc120-mt-gd-1_56 -llibboost_system-vc120-mt-gd-1_56
# for linux
else:unix: LIBS += -L/usr/lib/ -lboost_thread -lboost_system -lboost_signals

# path dei file header (.h) di Boost
win32:INCLUDEPATH += "C://Program Files//Boost//include"
else:unix:INCLUDEPATH += /usr/include/boost
win32:DEPENDPATH += "C://Program Files//Boost//include"
else:unix:DEPENDPATH += /usr/include/boost








#################
# LIB VTK
#################
# find lib of VTK 6.1 for release AND debug
win32:LIBS += -L"C://Program Files//VTK//lib//"\
-lvtkChartsCore-6.1\
-lvtkCommonColor-6.1\
-lvtkCommonComputationalGeometry-6.1\
-lvtkCommonCore-6.1\
-lvtkCommonDataModel-6.1\
-lvtkCommonExecutionModel-6.1\
-lvtkCommonMath-6.1\
-lvtkCommonMisc-6.1\
-lvtkCommonSystem-6.1\
-lvtkCommonTransforms-6.1\
-lvtkDICOMParser-6.1\
-lvtkDomainsChemistry-6.1\
-lvtkFiltersAMR-6.1\
-lvtkFiltersCore-6.1\
-lvtkFiltersExtraction-6.1\
-lvtkFiltersFlowPaths-6.1\
-lvtkFiltersGeneral-6.1\
-lvtkFiltersGeneric-6.1\
-lvtkFiltersGeometry-6.1\
-lvtkFiltersHybrid-6.1\
-lvtkFiltersHyperTree-6.1\
-lvtkFiltersImaging-6.1\
-lvtkFiltersModeling-6.1\
-lvtkFiltersParallel-6.1\
-lvtkFiltersParallelImaging-6.1\
-lvtkFiltersProgrammable-6.1\
-lvtkFiltersSMP-6.1\
-lvtkFiltersSelection-6.1\
-lvtkFiltersSources-6.1\
-lvtkFiltersStatistics-6.1\
-lvtkFiltersTexture-6.1\
-lvtkFiltersVerdict-6.1\
-lvtkGUISupportQt-6.1\
-lvtkGUISupportQtOpenGL-6.1\
-lvtkGUISupportQtSQL-6.1\
-lvtkGUISupportQtWebkit-6.1\
-lvtkGeovisCore-6.1\
-lvtkIOAMR-6.1\
-lvtkIOCore-6.1\
-lvtkIOEnSight-6.1\
-lvtkIOExodus-6.1\
-lvtkIOExport-6.1\
-lvtkIOGeometry-6.1\
-lvtkIOImage-6.1\
-lvtkIOImport-6.1\
-lvtkIOInfovis-6.1\
-lvtkIOLSDyna-6.1\
-lvtkIOLegacy-6.1\
-lvtkIOMINC-6.1\
-lvtkIOMovie-6.1\
-lvtkIONetCDF-6.1\
-lvtkIOPLY-6.1\
-lvtkIOParallel-6.1\
-lvtkIOSQL-6.1\
-lvtkIOVideo-6.1\
-lvtkIOXML-6.1\
-lvtkIOXMLParser-6.1\
-lvtkImagingColor-6.1\
-lvtkImagingCore-6.1\
-lvtkImagingFourier-6.1\
-lvtkImagingGeneral-6.1\
-lvtkImagingHybrid-6.1\
-lvtkImagingMath-6.1\
-lvtkImagingMorphological-6.1\
-lvtkImagingSources-6.1\
-lvtkImagingStatistics-6.1\
-lvtkImagingStencil-6.1\
-lvtkInfovisCore-6.1\
-lvtkInfovisLayout-6.1\
-lvtkInteractionImage-6.1\
-lvtkInteractionStyle-6.1\
-lvtkInteractionWidgets-6.1\
-lvtkLocalExample-6.1\
-lvtkNetCDF-6.1\
-lvtkNetCDF_cxx-6.1\
-lvtkParallelCore-6.1\
-lvtkRenderingAnnotation-6.1\
-lvtkRenderingContext2D-6.1\
-lvtkRenderingCore-6.1\
-lvtkRenderingFreeType-6.1\
-lvtkRenderingFreeTypeOpenGL-6.1\
-lvtkRenderingGL2PS-6.1\
-lvtkRenderingImage-6.1\
-lvtkRenderingLIC-6.1\
-lvtkRenderingLOD-6.1\
-lvtkRenderingLabel-6.1\
-lvtkRenderingOpenGL-6.1\
-lvtkRenderingQt-6.1\
-lvtkRenderingVolume-6.1\
-lvtkRenderingVolumeAMR-6.1\
-lvtkRenderingVolumeOpenGL-6.1\
-lvtkTestingRendering-6.1\
-lvtkViewsContext2D-6.1\
-lvtkViewsCore-6.1\
-lvtkViewsGeovis-6.1\
-lvtkViewsInfovis-6.1\
-lvtkViewsQt-6.1\
-lvtkalglib-6.1\
-lvtkexoIIc-6.1\
-lvtkexpat-6.1\
-lvtkfreetype-6.1\
-lvtkftgl-6.1\
-lvtkgl2ps-6.1\
-lvtkhdf5-6.1\
-lvtkhdf5_hl-6.1\
-lvtkjpeg-6.1\
-lvtkjsoncpp-6.1\
-lvtklibxml2-6.1\
-lvtkmetaio-6.1\
-lvtkoggtheora-6.1\
-lvtkpng-6.1\
-lvtkproj4-6.1\
-lvtksqlite-6.1\
-lvtksys-6.1\
-lvtktiff-6.1\
-lvtkverdict-6.1\
-lvtkzlib-6.1
else:unix: LIBS += -L/usr/lib/\
-lvtkCommonDataModel-6.3\
-lvtkCommonMath-6.3\
-lvtkCommonCore-6.3\
-lvtkGUISupportQt-6.3\
-lvtkRenderingCore-6.3\
-lvtkRenderingLOD-6.3

# path dei file header (.h) di VTK
win32:INCLUDEPATH += "C://Program Files//VTK//include//vtk-6.1"
win32:DEPENDPATH += "C://Program Files//VTK//include//vtk-6.1"
else:unix:INCLUDEPATH += /usr/local/include/vtk-6.3
else:unix:DEPENDPATH += /usr/local/include/vtk-6.3




################
# KINECT 2
###############
win32:LIBS += -L"C://Program Files//Microsoft SDKs//Kinect//v2.0-PublicPreview1409//Lib//x64" -lKinect20

win32:INCLUDEPATH += "C://Program Files//Microsoft SDKs//Kinect//v2.0-PublicPreview1409//inc"
win32:DEPENDPATH += "C://Program Files//Microsoft SDKs//Kinect//v2.0-PublicPreview1409//inc"



################
# CUDA
###############
#LIBS += -L/usr/local/cuda-7.0/lib64 -lcublas -lcudart -lcufft -lcufftw

#INCLUDEPATH += /usr/local/cuda-7.0/include
#DEPENDPATH += /usr/local/cuda-7.0/include


################
# OpenNI
###############
#LIBS += -L/usr/lib -lOpenNI
#INCLUDEPATH += /usr/include/ni
#DEPENDPATH += /usr/include/ni

RESOURCES +=  Icon/Icon.qrc

#OTHER_FILES +=\
#    Icon/trashcan-128 (1).png

OTHER_FILES +=  CMakeLists.txt

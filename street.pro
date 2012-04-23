######################################################################
# Automatically generated by qmake (2.01a) Mon Apr 23 20:16:52 2012
######################################################################

TEMPLATE = app
TARGET = street
DEPENDPATH += . src
INCLUDEPATH += . src

CONFIG += console
CONFIG -= app_bundle

OBJECTS_DIR = build

#image plugins
QMAKE_LFLAGS += -L$$[QT_INSTALL_PLUGINS]/imageformats

# osg
LIBS += /usr/local/lib/libosg*
LIBS += /usr/local/lib/libOpenThreads.2.4.0.dylib

# ogl
#QT += opengl

# opencv
INCLUDEPATH += /usr/local/include/opencv/
LIBS += /usr/local/lib/libcv.a
LIBS += /usr/local/lib/libcvaux.a
LIBS += /usr/local/lib/libcvhaartraining.a
LIBS += /usr/local/lib/libcxcore.a
LIBS += /usr/local/lib/libhighgui.a
LIBS += /usr/local/lib/libml.a

# ann
INCLUDEPATH += /usr/local/include/ann/
LIBS += /usr/local/lib/libANN.a

# libnoise
INCLUDEPATH += /usr/local/include/noise
#LIBS += -L/usr/local/lib/libnoise.a -lnoise
LIBS += -L/usr/local/lib/ -lnoise

# boost
INCLUDEPATH += /opt/local/include
LIBS += -L/opt/local/lib -lboost_program_options
LIBS += -L/opt/local/lib -lboost_filesystem
LIBS += -L/opt/local/lib -lboost_system

# Input
HEADERS += src/AdvancedVolume.h \
           src/BDL.h \
           src/BDLCamera.h \
           src/BDLPointGraph.h \
           src/BDLSG2Bezier.h \
           src/BDLSkeletonElementLoader.h \
           src/BDLSkeletonLoader.h \
           src/BDLTreeParser.h \
           src/BillboardTree.h \
           src/CameraInfo.h \
           src/ConvexHull2D.h \
           src/eigen.h \
           src/FastCache.h \
           src/GenericLeafGrower.h \
           src/GPCA.h \
           src/GPCA_global.h \
           src/HoughTransform3d.h \
           src/ImageContour.h \
           src/ISPLoader.h \
           src/LaserCamera.h \
           src/LaserCameraInfo.h \
           src/LaserProjector.h \
           src/LaserPruner.h \
           src/LaserSkeletonGrower.h \
           src/LeafGrower.h \
           src/Library.h \
           src/LibraryElement.h \
           src/LineModel.h \
           src/OrientedCircle.h \
           src/OrientedCircle2.h \
           src/osgModeler.h \
           src/PalmParameter.h \
           src/PointModel.h \
           src/RealisticLeafGrower.h \
           src/RectangleModel.h \
           src/RectPlacement.h \
           src/SimplePruner.h \
           src/SimpleSkeletonGrower.h \
           src/SimpleVolume.h \
           src/SimpleVolumeFloat.h \
           src/SingleImagePalm.h \
           src/SkeletonSimplifier.h \
           src/SLink.h \
           src/ThresholdGraph.h \
           src/TopologicalNode.h \
           src/TopologicalSkeleton.h \
           src/Transformer.h \
           src/Triangle.h \
           src/Veronese.h \
           src/Volume.h \
           src/VolumeSurface.h
SOURCES += src/AdvancedVolume.cpp \
           src/BDL.cpp \
           src/BDLCamera.cpp \
           src/BDLPointGraph.cpp \
           src/BDLSG2Bezier.cpp \
           src/BDLSkeletonElementLoader.cpp \
           src/BDLSkeletonLoader.cpp \
           src/BDLTreeParser.cpp \
           src/BillboardTree.cpp \
           src/ConvexHull2D.cpp \
           src/eigen.cpp \
           src/FastCache.cpp \
           src/GenericLeafGrower.cpp \
           src/GPCA.cpp \
           src/GPCA_global.cpp \
           src/HoughTransform3d.cpp \
           src/ImageContour.cpp \
           src/ISPLoader.cpp \
           src/LaserCamera.cpp \
           src/LaserCameraInfo.cpp \
           src/LaserProjector.cpp \
           src/LaserPruner.cpp \
           src/LaserSkeletonGrower.cpp \
           src/LeafGrower.cpp \
           src/Library.cpp \
           src/LibraryElement.cpp \
           src/LineModel.cpp \
           src/main.cpp \
           src/OrientedCircle.cpp \
           src/OrientedCircle2.cpp \
           src/osgModeler.cpp \
           src/PalmParameter.cpp \
           src/PointModel.cpp \
           src/RealisticLeafGrower.cpp \
           src/RectangleModel.cpp \
           src/RectPlacement.cpp \
           src/SimplePruner.cpp \
           src/SimpleSkeletonGrower.cpp \
           src/SimpleVolume.cpp \
           src/SimpleVolumeFloat.cpp \
           src/SingleImagePalm.cpp \
           src/SkeletonSimplifier.cpp \
           src/SLink.cpp \
           src/ThresholdGraph.cpp \
           src/TopologicalNode.cpp \
           src/TopologicalSkeleton.cpp \
           src/Transformer.cpp \
           src/Triangle.cpp \
           src/Veronese.cpp \
           src/Volume.cpp \
           src/VolumeSurface.cpp

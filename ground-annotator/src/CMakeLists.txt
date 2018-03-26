cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (ground_annotator)
find_package (Qt4 REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.7.1 REQUIRED)
find_package(but_velodyne REQUIRED)

find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

include_directories (${PCL_INCLUDE_DIRS} ${ButVELODYNE_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS} ${ButVELODYNE_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

FIND_PACKAGE(Boost 1.40 COMPONENTS program_options REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

set  (project_SOURCES main.cpp pclviewer.cpp annotationarea.cpp ringfiller.cpp)
set  (project_HEADERS pclviewer.h annotationarea.h ringfiller.h labeling.h)
set  (project_FORMS   pclviewer.ui)
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (ground_annotator ${project_SOURCES}
                                  ${project_FORMS_HEADERS}
                                  ${project_HEADERS_MOC})


set(VELODYNE_MODEL "64" CACHE FILEPATH "Model of the Velodyne LiDAR (16, 32 or 64=default)")
add_definitions(-DVELODYNE_MODEL=${VELODYNE_MODEL})

TARGET_LINK_LIBRARIES (ground_annotator ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${ButVELODYNE_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})


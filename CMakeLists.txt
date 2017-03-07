cmake_minimum_required(VERSION 2.8.3)
project(asr_mild_calibration_tool)

#use c++ 11
add_definitions(-std=gnu++0x)

set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  asr_msgs
  asr_flir_ptu_controller
  tf
  eigen_conversions
  tf_conversions
  asr_sick_lms_400
  sensor_msgs
  image_transport
  image_geometry
  cv_bridge
  asr_aruco_marker_recognition
)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtOpenGL QtSvg QtXml)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

#QWT Stuff
find_path ( QWT_INCLUDE_DIR NAMES qwt_plot.h HINTS ${QT_INCLUDE_DIR} PATH_SUFFIXES qwt qwt-qt4)
set ( QWT_INCLUDE_DIRS ${QWT_INCLUDE_DIR} )
find_library ( QWT_LIBRARY NAMES qwt qwt-qt4)
set ( QWT_LIBRARIES ${QWT_LIBRARY} )

catkin_package(
  INCLUDE_DIRS include)

include(
${QT_USE_FILE}
)

include_directories(include 
    ${catkin_INCLUDE_DIRS} 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${QWT_INCLUDE_DIR}
    )

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
#file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.h)

SET (MOC_HEADERS
        include/Laserscanner/abstract_laserscanner.h
        include/Laserscanner/laserscanner_mild.h
        include/Laserscanner/laserscanner_lms400.h
        include/Transformation/transformation_publisher.h
        include/CameraUtils/ptu_manager.h
        include/CameraUtils/marker_manager.h
        include/CameraUtils/camerathread.h
        include/GUI/mainwindow.h
        include/GUI/ptuwindow.h
        include/GUI/frameeditorwindow.h
        include/laserscanthread.h
        include/VisualisationUtils/camerawidget.h
        include/VisualisationUtils/laserscanwidget.h
        include/VisualisationUtils/tfvisualizer.h
    )

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_CPP(QT_MOC_HPP ${MOC_HEADERS})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)


add_library(calibrationTool_core
    src/calibration_object.cpp
    src/Laserscanner/laserscanner_mild.cpp
    src/Laserscanner/laserscanner_lms400.cpp
    src/Laserscanner/abstract_laserscanner.cpp
    src/Transformation/abstract_transformationfile_manager.cpp
    src/Transformation/transformationfile_manager_data.cpp
    src/Transformation/transformationfile_manager_XML.cpp
    src/Transformation/transformation_data.cpp
    src/Transformation/transformation_publisher.cpp
    src/MathHelpers/Resectionsolver/resectionsolver.cpp
    src/MathHelpers/Resectionsolver/poly34.cpp
    src/MathHelpers/Resectionsolver/feasibilitychecker.cpp
    src/MathHelpers/linearApproximator.cpp
    src/CameraUtils/ptu_manager.cpp
    src/CameraUtils/marker_manager.cpp
    src/CameraUtils/camerathread.cpp
    src/VisualisationUtils/laserscanwidget.cpp
    src/VisualisationUtils/markerpublisher.cpp
    src/VisualisationUtils/polarpointseriesdata.cpp
    src/VisualisationUtils/camerawidget.cpp
    src/VisualisationUtils/tfvisualizer.cpp
    src/laserscanthread.cpp
    src/GUI/frameeditorwindow.cpp
    src/GUI/mainwindow.cpp
    src/GUI/ptuwindow.cpp
    ${QT_RESOURCES_CPP}
    ${QT_FORMS_HPP}
    ${QT_MOC_HPP}
  )

target_link_libraries(calibrationTool_core
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
    ${QT_LIBRARIES}
    ${ROS_LIBRARIES}
    ${Boost_LIBRARIES}
    ${QWT_LIBRARY}
    PTUControllerCPP
    asr_sick_lms_400
 )

set (BUILD_CALIBRATION_TOOL_TEST FALSE)

if (${BUILD_CALIBRATION_TOOL_TEST})
add_executable(calibrationTool_tests
    src/testcases_main.cpp
  )

target_link_libraries(calibrationTool_tests
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
    ${QT_LIBRARIES}
    ${ROS_LIBRARIES}
    ${Boost_LIBRARIES}
    ${QWT_LIBRARY}
    PTUControllerCPP
    asr_sick_lms_400
    calibrationTool_core
    #${CMAKE_CURRENT_LIST_DIR}/../QWT/lib/libqwt.so
    #${CMAKE_CURRENT_LIST_DIR}/../QWT_Polar/lib/libqwtpolar.so
 )
endif ()


add_executable(calibrationTool
    src/calibration_main.cpp
  )

target_link_libraries(calibrationTool
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
    ${QT_LIBRARIES}
    ${ROS_LIBRARIES}
    ${Boost_LIBRARIES}
    ${QWT_LIBRARY}
    PTUControllerCPP
    asr_sick_lms_400
    calibrationTool_core
    #${CMAKE_CURRENT_LIST_DIR}/../QWT/lib/libqwt.so
    #${CMAKE_CURRENT_LIST_DIR}/../QWT_Polar/lib/libqwtpolar.so
 )
 
 
add_executable(frame_editor
    src/frameeditor_main.cpp
  )

target_link_libraries(frame_editor
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
    ${QT_LIBRARIES}
    ${ROS_LIBRARIES}
    ${Boost_LIBRARIES}
    ${QWT_LIBRARY}
    PTUControllerCPP
    asr_sick_lms_400
    calibrationTool_core
    #${CMAKE_CURRENT_LIST_DIR}/../QWT/lib/libqwt.so
    #${CMAKE_CURRENT_LIST_DIR}/../QWT_Polar/lib/libqwtpolar.so
 )



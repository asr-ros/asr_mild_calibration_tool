/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef PTUWINDOW_H
#define PTUWINDOW_H
#include "CameraUtils/ptu_manager.h"
#include "CameraUtils/marker_manager.h"
#include "CameraUtils/camerathread.h"
#include <QDialog>
#include <QBitmap>
#include <QLabel>

#include <QPushButton>
#include "calibration_object.h"
#include "VisualisationUtils/camerawidget.h"
#include "VisualisationUtils/markerpublisher.h"
#include <Transformation/transformation_data.h>
#include <Transformation/transformation_publisher.h>
#include <Transformation/transformationfile_manager_data.h>
#ifndef Q_MOC_RUN
#include <sensor_msgs/Image.h>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#endif


class PTUWindow : public QDialog
{
    Q_OBJECT
public:
    explicit PTUWindow(QWidget *parent = 0);
    PTUWindow( boost::shared_ptr<MarkerPublisher> markerManager, boost::shared_ptr<Calibration_Object> calibrationObject, boost::shared_ptr<Transformation_Publisher> transformationPublisher, const Eigen::Matrix4d calObjTransformation, QWidget *parent =0);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    void init(boost::shared_ptr<MarkerPublisher> markerManager, boost::shared_ptr<Calibration_Object> calibrationObject, boost::shared_ptr<Transformation_Publisher> transformationPublisher, const Eigen::Matrix4d calObjTransformation);
    void startCapture();
    void stopMarkerCapturing();
    void setUI_Elements();
    void calculateAllAverageDataFrames();
    Eigen::Matrix4d calculateAverageDataFrame(std::vector<Eigen::Matrix4d , Eigen::aligned_allocator<Eigen::Matrix4d > > frames);
    PTU_Manager * ptuManager;
    Marker_Manager * markerManager;

    QPushButton *btnStartStopPTUSearch;
    QPushButton *btnSkipFramePTUSearch;
    QPushButton *btnExportCameraPoses;
    QPushButton *btnExportRelativeMarkerPoses;
    QPushButton *btnExportCameraOffset;

    QLabel *lblCameraFrameCount;
    QLabel *lblRelativeMarkerFrameCount;

    CameraWidget * cameraLeft;
    CameraWidget * cameraRight;

    bool slotEnabled_PTU;
    bool slotEnabled_Marker;
    bool stopSignal;

    //Defines if raw marker position or averaged postions over all markers per frame are used
    bool useAveragedMarkerData;

    //Used to publish the current results to ros
    boost::shared_ptr<MarkerPublisher> markerPublisher;

    double marker_min_distance;
    double marker_max_distance;
    double tilt_min_angle;
    double tilt_max_angle;
    double pan_min_angle;
    double pan_max_angle;

    tf::TransformListener * mTFlistener;

    //Publishes transformation frames to tf
    boost::shared_ptr<Transformation_Publisher> transformationPublisher;

    //Contains all the parameters of the calibration object
    boost::shared_ptr<Calibration_Object> calibrationObject;

    Eigen::Matrix4d transformation_LaserScanner;
    std::vector<colouredCameraFrame, Eigen::aligned_allocator<colouredCameraFrame> > colouredCameraFrames;
    std::vector<Transformation_Data> transformationData;

    std::vector<Transformation_Data> tempTransformationData;

    std::vector<Eigen::Matrix4d , Eigen::aligned_allocator<Eigen::Matrix4d > > relativeMarkerPoses;
    std::vector<Eigen::Matrix4d , Eigen::aligned_allocator<Eigen::Matrix4d > > markerRightPoses;
    std::vector<Eigen::Matrix4d , Eigen::aligned_allocator<Eigen::Matrix4d > > markerLeftPoses;
    //Used to save collected data for left and right frame in order to calculte the average frames afterwards
    std::vector<Eigen::Matrix4d , Eigen::aligned_allocator<Eigen::Matrix4d > > tempMarkerRightPoses;
    std::vector<Eigen::Matrix4d , Eigen::aligned_allocator<Eigen::Matrix4d > > tempMarkerLeftPoses;
    //Used for evaluation
    PanTiltOffsetTupleList cameraOffsetParameters;

    void handleNewTransformationData(Transformation_Data &data);
    void resizeEvent(QResizeEvent * event);
    bool markerPoseWithinBounds(const Eigen::Matrix4d& transform);
signals:
    void newTransform_Camera(const Eigen::Matrix4d& transform, Markertype markertype);
public slots:
    void btnStartStopPTUSearch_clicked();
    void btnExportCameraPoses_clicked();
    void btnExportRelativeMarkerPoses_clicked();
    void btnSkipFramePTUSearch_clicked();
    void btnExportCameraOffset_clicked();
    void ptu_moved(double pan, double tilt);
    void markerFound(std::string markerNumber, const Eigen::Matrix4d &transformation);
};

#endif // PTUWINDOW_H

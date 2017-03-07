/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "GUI/ptuwindow.h"
#include "QImage"
#include "QRgb"
#include <QFileDialog>


PTUWindow::PTUWindow(QWidget *parent) :
    QDialog(parent)
{
    transformationData.clear();
    boost::shared_ptr<MarkerPublisher> myMarkerPublisher(new MarkerPublisher(calibrationObject));
    boost::shared_ptr<Transformation_Publisher> myTransformationPublisher(new Transformation_Publisher(calibrationObject));
    this->init(myMarkerPublisher, calibrationObject, myTransformationPublisher, Eigen::Matrix4d::Identity());
}

PTUWindow::PTUWindow(boost::shared_ptr<MarkerPublisher> markerPublisher, boost::shared_ptr<Calibration_Object> calibrationObject, boost::shared_ptr<Transformation_Publisher> transformationPublisher,const Eigen::Matrix4d calObjTransformation, QWidget *parent) :
    QDialog(parent)
{
    this->init(markerPublisher, calibrationObject, transformationPublisher, calObjTransformation);
}

void PTUWindow::init(boost::shared_ptr<MarkerPublisher> markerPublisher, boost::shared_ptr<Calibration_Object> calibrationObject, boost::shared_ptr<Transformation_Publisher> transformationPublisher, const Eigen::Matrix4d calObjTransformation)
{
    this->setWindowTitle( "PTU Calibration");
    ros::NodeHandle mNodeHandle(ros::this_node::getName());
    double pan_min_angle, pan_max_angle, tilt_min_angle, tilt_max_angle;
    double marker_recognizer_timeout,marker_recognizer_early_abort_timeout, marker_min_distance, marker_max_distance;
    int pan_step_count, tilt_step_count;
    bool useAveragedMarkerData;
    mNodeHandle.getParam("pan_min_angle", pan_min_angle);
    mNodeHandle.getParam("pan_max_angle", pan_max_angle);
    mNodeHandle.getParam("tilt_min_angle", tilt_min_angle);
    mNodeHandle.getParam("tilt_max_angle", tilt_max_angle);
    mNodeHandle.getParam("pan_step_count", pan_step_count);
    mNodeHandle.getParam("tilt_step_count", tilt_step_count);
    mNodeHandle.getParam("marker_recognizer_timeout", marker_recognizer_timeout);
    mNodeHandle.getParam("marker_recognizer_early_abort_timeout", marker_recognizer_early_abort_timeout);
    mNodeHandle.getParam("marker_min_distance", marker_min_distance);
    mNodeHandle.getParam("marker_max_distance", marker_max_distance);
    mNodeHandle.getParam("useAveragedMarkerData", useAveragedMarkerData);
    this->marker_min_distance = marker_min_distance;
    this->marker_max_distance = marker_max_distance;
    this->tilt_min_angle = tilt_min_angle;
    this->tilt_max_angle = tilt_max_angle;
    this->pan_min_angle = pan_min_angle;
    this->pan_max_angle = pan_max_angle;
    this->useAveragedMarkerData = useAveragedMarkerData;
    ptuManager = new PTU_Manager(pan_min_angle, pan_max_angle, tilt_min_angle, tilt_max_angle, pan_step_count, tilt_step_count);
    ROS_INFO("Manager started");

    ptuManager->setNeutralPose();

    this->markerPublisher = markerPublisher;
    this->calibrationObject = calibrationObject;
    this->transformationPublisher = transformationPublisher;
    this->transformation_LaserScanner = calObjTransformation;
    this->markerManager = new Marker_Manager(marker_recognizer_timeout, marker_recognizer_early_abort_timeout);

    markerPublisher->publishARMarkers(false, false, transformation_LaserScanner);

    btnStartStopPTUSearch = new QPushButton(this);
    btnStartStopPTUSearch->resize(100, 25);
    btnStartStopPTUSearch->setVisible(true);
    btnStartStopPTUSearch->setText("Start");

    btnExportCameraPoses = new QPushButton(this);
    btnExportCameraPoses->resize(100, 25);
    btnExportCameraPoses->setVisible(true);
    btnExportCameraPoses->setText("Export Camera");
    btnExportCameraPoses->setEnabled(false);

    btnExportRelativeMarkerPoses = new QPushButton(this);
    btnExportRelativeMarkerPoses->resize(100, 25);
    btnExportRelativeMarkerPoses->setVisible(true);
    btnExportRelativeMarkerPoses->setText("Export Marker");
    btnExportCameraPoses->setEnabled(false);

    btnSkipFramePTUSearch = new QPushButton(this);
    btnSkipFramePTUSearch->resize(100, 25);
    btnSkipFramePTUSearch->setVisible(true);
    btnSkipFramePTUSearch->setText("Skip Frame");
    btnSkipFramePTUSearch->setEnabled(false);

    btnExportCameraOffset = new QPushButton(this);
    btnExportCameraOffset->resize(100, 25);
    btnExportCameraOffset->setVisible(true);
    btnExportCameraOffset->setText("Export Offset");
    btnExportCameraOffset->setEnabled(false);

    lblCameraFrameCount = new QLabel(this);
    lblCameraFrameCount->resize(200, 30);
    lblCameraFrameCount->setVisible(true);

    lblRelativeMarkerFrameCount = new QLabel(this);
    lblRelativeMarkerFrameCount->resize(350, 30);
    lblRelativeMarkerFrameCount->setVisible(true);

    CameraThread * camThreadLeft = new CameraThread(0, "/stereo/left/image_raw");
    cameraLeft = new CameraWidget(this, camThreadLeft);

    cameraLeft->resize(320,240);
    cameraLeft->move(50,50);

    CameraThread * camThreadRight = new CameraThread(0, "/stereo/right/image_raw");
    cameraRight = new CameraWidget(this, camThreadRight);
    cameraRight->resize(320,240);
    cameraRight->move(400,50);

    connect(btnStartStopPTUSearch, SIGNAL(clicked()),this, SLOT(btnStartStopPTUSearch_clicked()));
    connect(btnExportCameraPoses, SIGNAL(clicked()),this, SLOT(btnExportCameraPoses_clicked()));
    connect(btnExportRelativeMarkerPoses, SIGNAL(clicked()),this, SLOT(btnExportRelativeMarkerPoses_clicked()));
    connect(btnExportCameraOffset, SIGNAL(clicked()),this, SLOT(btnExportCameraOffset_clicked()));
    connect(btnSkipFramePTUSearch, SIGNAL(clicked()),this, SLOT(btnSkipFramePTUSearch_clicked()));

    connect(this, SIGNAL(newTransform_Camera(const Eigen::Matrix4d, Markertype)),this->transformationPublisher.get(), SLOT(newTransform_Camera(const Eigen::Matrix4d, Markertype)));

    ROS_INFO("PTU Window started");
    stopSignal = true;

    mTFlistener = new tf::TransformListener();

    setUI_Elements();
}

void PTUWindow::setUI_Elements()
{
    btnExportCameraPoses->setEnabled(transformationData.size() > 0);
    btnExportRelativeMarkerPoses->setEnabled(relativeMarkerPoses.size() > 0);
    btnExportCameraOffset->setEnabled(relativeMarkerPoses.size() > 0);

    QString text = "Camera datasets found: ";
    text += QString::number(transformationData.size());
    lblCameraFrameCount->setText(text);

    text = "Relative marker frames found: ";
    text += QString::number(relativeMarkerPoses.size());
    //Calculate standard deviation
    if (relativeMarkerPoses.size() > 0)
    {
        double standardDeviation = 0.0;
        for (unsigned int i = 0; i < relativeMarkerPoses.size(); i++)
        {
            Eigen::Vector3d averageTranslation(relativeMarkerPoses.at(i)(0,3), relativeMarkerPoses.at(i)(1,3),relativeMarkerPoses.at(i)(2,3));
            standardDeviation += pow(averageTranslation.norm(), 2.0);
        }
        standardDeviation = sqrt(standardDeviation/(double)relativeMarkerPoses.size());
        text += ", standarddeviation: ";
        text += QString::number(standardDeviation);
    }

    lblRelativeMarkerFrameCount->setText(text);
}

void PTUWindow::resizeEvent(QResizeEvent * event)
{
    cameraLeft->resize(this->width()/2-60,this->height()-150);
    cameraLeft->move(30,50);
    cameraRight->resize(this->width()/2-60,this->height()-150);
    cameraRight->move(this->width()/2+30,50);
    btnSkipFramePTUSearch->move(this->width()-540,this->height()-40);
    btnStartStopPTUSearch->move(this->width()-435,this->height()-40);
    btnExportCameraPoses->move(this->width()-330,this->height()-40);
    btnExportRelativeMarkerPoses->move(this->width()-225,this->height()-40);
    btnExportCameraOffset->move(this->width()-120,this->height()-40);
    lblCameraFrameCount->move(30,this->height()-75);
    lblRelativeMarkerFrameCount->move(30,this->height()-45);
    QDialog::resizeEvent(event);
}

void PTUWindow::btnSkipFramePTUSearch_clicked()
{
    if(!stopSignal && slotEnabled_Marker)
    {
        stopMarkerCapturing();
    }
}

void PTUWindow::btnStartStopPTUSearch_clicked()
{
    if (stopSignal)
    {
        ROS_INFO("PTU: Starting iteration..");
        btnStartStopPTUSearch->setText("Stop");
        stopSignal = false;
        startCapture();
    }
    else
    {
        disconnect(this, SLOT(ptu_moved(double, double)));
        disconnect(this, SLOT(markerFound(std::string, const Eigen::Matrix4d)));
        btnStartStopPTUSearch->setText("Start");
        stopSignal = true;
    }
}

void PTUWindow::btnExportCameraOffset_clicked()
{
    if (cameraOffsetParameters.size() > 0)
    {
        QString fileName = QFileDialog::getSaveFileName(this, "Save file", "CameraOffset.off", tr("Offsetfile (*.off)"));
        if (fileName.length() > 0)
        {
            ROS_INFO_STREAM("File: " << fileName.toStdString());
            TransformationFile_Manager_Data fileManager(fileName.toStdString());
            fileManager.writeCameraOffsetToFile(cameraOffsetParameters);
        }
    }

}

void PTUWindow::btnExportCameraPoses_clicked()
{
    if (transformationData.size() > 0)
    {
        QString fileName = QFileDialog::getSaveFileName(this, "Save file", "CalibrationData.data", tr("Calibrationfile (*.data)"));
        if (fileName.length() > 0)
        {
            ROS_INFO_STREAM("File: " << fileName.toStdString());
            TransformationFile_Manager_Data fileManager(fileName.toStdString());
            fileManager.writeToFile(transformationData);
        }
    }
}

void PTUWindow::btnExportRelativeMarkerPoses_clicked()
{
    if (relativeMarkerPoses.size() > 0)
    {
        QString fileName = QFileDialog::getSaveFileName(this, "Save file", "CalibrationData.data", tr("Calibrationfile (*.data)"));
        if (fileName.length() > 0)
        {
            ROS_INFO_STREAM("File: " << fileName.toStdString());
            TransformationFile_Manager_Data fileManager(fileName.toStdString());
            fileManager.writeTransformationToFile(relativeMarkerPoses);
        }
    }
}

void PTUWindow::startCapture()
{
    ptuManager->setStartPose();
    colouredCameraFrames.clear();
    markerLeftPoses.clear();
    markerRightPoses.clear();
    tempMarkerLeftPoses.clear();
    tempMarkerRightPoses.clear();
    cameraOffsetParameters.clear();
    connect(ptuManager, SIGNAL(ptu_moved(double, double)),this, SLOT(ptu_moved(double, double)));
    connect(markerManager, SIGNAL(markerFound(std::string,const Eigen::Matrix4d)),this, SLOT(markerFound(std::string, const Eigen::Matrix4d)));
    ROS_INFO("PTU: Starting iteration..");
    if (this->useAveragedMarkerData)
    {
        ROS_INFO("Averaging PTU frame ON");
    }
    else
    {
        ROS_INFO("Averaging PTU frame OFF");
    }
    slotEnabled_PTU = true;
    slotEnabled_Marker = false;
    btnSkipFramePTUSearch->setEnabled(true);
    if (!ptuManager->nextPose())
    {
        ROS_INFO("PTU: No iteration possible.");
        btnStartStopPTUSearch->setText("Stop");
        disconnect(this, SLOT(ptu_moved(double, double)));
        disconnect(this, SLOT(markerFound(std::string, const Eigen::Matrix4d)));
        stopSignal = true;
        btnSkipFramePTUSearch->setEnabled(false);
    }
}

void PTUWindow::handleNewTransformationData(Transformation_Data &data)
{
    //Create visualization markers
    colouredCameraFrame cCF;
    cCF.pose = data.PTU_Frame;
    cCF.color.r = 1.0;
    cCF.color.g = (data.pan - pan_min_angle) / (pan_max_angle - pan_min_angle);
    cCF.color.b = (data.tilt - tilt_min_angle) / (tilt_max_angle - tilt_min_angle);
    cCF.color.a = 1.0;
    colouredCameraFrames.push_back(cCF);
    //Store dataset
    transformationData.push_back(data);

    //Publish visualization
    markerPublisher->publishColouredCameraFrames(&colouredCameraFrames);
}

void PTUWindow::markerFound(std::string markerNumber, const Eigen::Matrix4d &transformation)
{
    if (slotEnabled_Marker && !stopSignal)
    {
        if (markerNumber != "")
        {
            ROS_INFO_STREAM(markerNumber);
            bool markerLeft = (markerNumber == calibrationObject->marker_id_left);
            bool markerRight = (markerNumber == calibrationObject->marker_id_right);
            markerPublisher->publishARMarkers(markerLeft, markerRight, transformation_LaserScanner);
            if (markerRight ||  markerLeft)
            {
                Eigen::Matrix4d markerBase;
                // Constant transformation frame that is multiplied to the detected marker frame due to recent changes in the markers coordinate system
                Eigen::Affine3d cameraCorrectionMatrix;
                cameraCorrectionMatrix = Eigen::Affine3d (Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d(1.0,0.0,0.0)));

                Eigen::Affine3d markerFrame(transformation);
                Eigen::Matrix4d markerToCamera = markerFrame.inverse().matrix();
                markerToCamera = markerToCamera;
                if (markerToCamera(1,3) < 0)
                {
                    Markertype markertype;
                    if (markerRight)
                    {
                        if (this->useAveragedMarkerData)
                        {
                            tempMarkerRightPoses.push_back(transformation);
                        }
                        else
                        {
                            markerRightPoses.push_back(transformation);
                        }
                        markerBase = transformation_LaserScanner * calibrationObject->frame_marker_right*cameraCorrectionMatrix.matrix();
                        markertype = Markertype::RIGHT;
                        ROS_INFO_STREAM("Marker right found");
                    }
                    else
                    {
                        if (this->useAveragedMarkerData)
                        {
                            tempMarkerLeftPoses.push_back(transformation);
                        }
                        else
                        {
                            markerLeftPoses.push_back(transformation);
                        }
                        markerBase = transformation_LaserScanner * calibrationObject->frame_marker_left*cameraCorrectionMatrix.matrix();
                        markertype = Markertype::LEFT;
                        ROS_INFO_STREAM("Marker left found");
                    }

                    Eigen::Matrix4d scannerToPTU = markerBase * markerToCamera;
                    Eigen::Vector3d startPoint(markerBase(0,3), markerBase(1,3),markerBase(2,3));
                    Eigen::Vector3d endPoint(scannerToPTU(0,3), scannerToPTU(1,3),scannerToPTU(2,3));
                    if (markerPoseWithinBounds(transformation))
                    {
                        double pan = ptuManager->getPan();
                        double tilt = ptuManager->getTilt();

                        //Publish tf
                        this->newTransform_Camera(markerToCamera, markertype);

                        Transformation_Data dataSet;
                        dataSet.pan = pan;
                        dataSet.tilt = tilt;
                        dataSet.PTU_Frame = scannerToPTU;
                        dataSet.LaserScan_Frame = transformation_LaserScanner;
                        if (this->useAveragedMarkerData)
                        {
                            tempTransformationData.push_back(dataSet);
                        }
                        else
                        {
                            handleNewTransformationData(dataSet);
                        }

                        markerPublisher->publishCameraFramePointer(startPoint, endPoint, true);

                        //Publish difference to camera model
                        if (mTFlistener->waitForTransform("/calibration_center", "/camera_right_frame", ros::Time(), ros::Duration(4.0)))
                        {
                            //Assume that tf is alive and lookups will be successful
                            tf::StampedTransform cameraOffsetTF, cameraOffsetOldTF;
                            Eigen::Affine3d cameraOffsetEigen, cameraOffsetOldEigen;
                            mTFlistener->lookupTransform("/calibration_center", "/camera_right_frame", ros::Time(0), cameraOffsetTF);
                            mTFlistener->lookupTransform("/calibration_center", "/camera_right_frame_old", ros::Time(0), cameraOffsetOldTF);
                            tf::poseTFToEigen(cameraOffsetTF, cameraOffsetEigen);
                            tf::poseTFToEigen(cameraOffsetOldTF, cameraOffsetOldEigen);
                            cameraOffsetEigen = cameraOffsetEigen.inverse() * scannerToPTU;
                            cameraOffsetOldEigen = cameraOffsetOldEigen.inverse() * scannerToPTU;
                            ROS_INFO_STREAM("Offset between camera frames (new): " << cameraOffsetEigen(0,3) << ", "<< cameraOffsetEigen(1,3) << ", "<< cameraOffsetEigen(2,3) << " for (Pan: " << pan <<", Tilt: " << tilt << ")");
                            ROS_INFO_STREAM("Offset between camera frames (old): " << cameraOffsetOldEigen(0,3) << ", "<< cameraOffsetOldEigen(1,3) << ", "<< cameraOffsetOldEigen(2,3) << " for (Pan: " << pan <<", Tilt: " << tilt << ")");
                            PanTiltOffsetTuple offsetParameter = std::make_tuple (pan,tilt, cameraOffsetEigen.matrix(), cameraOffsetOldEigen.matrix());
                            cameraOffsetParameters.push_back(offsetParameter);
                        }
                        else
                        {
                            ROS_ERROR("TF lookup timed out. Is the transformation publisher running?");
                        }
                    }
                    else
                    {
                        markerPublisher->publishCameraFramePointer(startPoint, endPoint, false);
                        ROS_ERROR_STREAM("Marker " << markerNumber << " was out of bounds.");
                    }
                }
                else
                {
                    ROS_ERROR_STREAM("Marker " << markerNumber << ": unexpected position.");
                }
            }
            else
            {
                ROS_ERROR_STREAM("Unexpected marker: " << markerNumber);
            }
        }
        else
        {
            stopMarkerCapturing();
        }
    }
    setUI_Elements();
}

void PTUWindow::stopMarkerCapturing()
{
    slotEnabled_Marker = false;
    markerPublisher->publishARMarkers(false, false, transformation_LaserScanner);
    markerManager->terminate();

    if (this->useAveragedMarkerData)
    {
        calculateAllAverageDataFrames();
    }

    for(unsigned int i = 0; i < markerRightPoses.size(); i++)
    {
        for(unsigned int j = 0; j < markerLeftPoses.size(); j++)
        {
            Eigen::Affine3d markerLeftReferencePose(markerLeftPoses.at(j));
            Eigen::Affine3d markerRightReferencePose(markerRightPoses.at(i));
            markerRightReferencePose = markerLeftReferencePose.inverse() * markerRightReferencePose;
            relativeMarkerPoses.push_back(markerRightReferencePose.matrix());
        }
    }
    markerLeftPoses.clear();
    markerRightPoses.clear();
    btnSkipFramePTUSearch->setEnabled(false);
    if (ptuManager->nextPose())
    {
        slotEnabled_PTU = true;
        ROS_INFO("PTU: Moving to next pose..");
    }
    else
    {
        ROS_INFO("PTU: Finished");
        btnStartStopPTUSearch->setText("Start");
        stopSignal = true;
        disconnect(this, SLOT(ptu_moved(double, double)));
        disconnect(this, SLOT(markerFound(std::string, Eigen::Matrix4d)));
    }
}

void PTUWindow::calculateAllAverageDataFrames()
{
    //Calculate average camera frame
    if (tempTransformationData.size() > 0)
    {
        Transformation_Data dataSet;
        dataSet.pan = tempTransformationData.at(0).pan;
        dataSet.tilt = tempTransformationData.at(0).tilt;
        std::vector<Eigen::Matrix4d , Eigen::aligned_allocator<Eigen::Matrix4d > > tempFrames;
        for (unsigned int i = 0; i < tempTransformationData.size(); i++)
        {
            tempFrames.push_back(tempTransformationData.at(i).PTU_Frame);
        }
        dataSet.PTU_Frame = calculateAverageDataFrame(tempFrames);
        handleNewTransformationData(dataSet);
    }
    //Calculate marker frames of left and right marker
    if (this->tempMarkerLeftPoses.size() > 0)
    {
        Eigen::Matrix4d averageLeftFrame = calculateAverageDataFrame(this->tempMarkerLeftPoses);
        markerLeftPoses.push_back(averageLeftFrame);
    }
    if (this->tempMarkerRightPoses.size() > 0)
    {
        Eigen::Matrix4d averageRightFrame = calculateAverageDataFrame(this->tempMarkerRightPoses);
        markerRightPoses.push_back(averageRightFrame);
    }
    tempTransformationData.clear();
    tempMarkerLeftPoses.clear();
    tempMarkerRightPoses.clear();
}

Eigen::Matrix4d PTUWindow::calculateAverageDataFrame(std::vector<Eigen::Matrix4d , Eigen::aligned_allocator<Eigen::Matrix4d > > frames)
{
    Eigen::Affine3d averageFrame;
    if (frames.size() > 0)
    {
        Eigen::Vector3d averageTranslation(frames.at(0)(0,3), frames.at(0)(1,3),frames.at(0)(2,3));
        Eigen::Affine3d transformation(frames.at(0));
        Eigen::Quaterniond averageRotation((transformation.rotation().matrix()));

        for (unsigned int i = 1; i < frames.size(); i++)
        {
            Eigen::Affine3d tempTransformation(frames.at(i));
            Eigen::Quaterniond tempRotation((tempTransformation.rotation().matrix()));
            Eigen::Vector3d tempTranslation(tempTransformation(0,3), tempTransformation(1,3), tempTransformation(2,3));
            averageTranslation = averageTranslation + tempTranslation;
            averageRotation = averageRotation.slerp((double)i/(i+1.0), tempRotation);
        }
        averageTranslation = averageTranslation * 1.0/(double)frames.size();

        averageFrame = Eigen::Affine3d(Eigen::Translation3d(averageTranslation));
        averageFrame = averageFrame * averageRotation;
    }
    return averageFrame.matrix();
}

bool PTUWindow::markerPoseWithinBounds(const Eigen::Matrix4d& transform)
{
    Eigen::Vector3d vec3(transform(0,3), transform(1,3), transform(2,3));

    double length = vec3.norm();
    if (length < marker_min_distance)
    {
        ROS_ERROR_STREAM("Length (" << length << ") was smaller than min distance");
        return false;
    }
    if (length > marker_max_distance)
    {
        ROS_ERROR_STREAM("Length (" << length << ") was greater than max distance");
        return false;
    }
    if (vec3(1) > 0.0)
    {
        //ROS_ERROR_STREAM("Vector is going upwards");
        //return false;
    }
    return true;
}

void PTUWindow::ptu_moved(double pan, double tilt)
{
    if (slotEnabled_PTU && !stopSignal)
    {
        slotEnabled_PTU = false;
        ROS_DEBUG_STREAM("PTU: Moved to " << pan << ", " << tilt);
        markerManager->start();
        btnSkipFramePTUSearch->setEnabled(true);
        slotEnabled_Marker = true;
    }
}

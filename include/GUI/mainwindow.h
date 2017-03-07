/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QCheckBox>
#include <QRadioButton>
#include <QLabel>
#include <QPushButton>
#include <QListWidget>
#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#endif

#include "VisualisationUtils/markerpublisher.h"
#include "VisualisationUtils/laserscanwidget.h"
#include "laserscanthread.h"
#include "Laserscanner/laserscanner_mild.h"
#include "Laserscanner/laserscanner_lms400.h"
#include <GUI/ptuwindow.h>
#include <calibration_object.h>
#include <MathHelpers/Resectionsolver/resectionsolver.h>
#include <MathHelpers/Resectionsolver/feasibilitychecker.h>
#include <Transformation/transformation_publisher.h>
#include <VisualisationUtils/tfvisualizer.h>

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
    unsigned int showData;
    bool isCapturing;
    QCheckBox *chk_rawdata;
    QCheckBox *chk_avgData;
    QCheckBox *chk_filteredData;
    QCheckBox *chk_segments;
    QCheckBox *chk_edges;

    //Shows the coordinates of all triangles currently detected by the scanner
    QLabel *lblTriangles;
    //Shows the number of solutions of the p3p problem for the current triangle
    QLabel *lblSolutions;

    QLabel *lblCurrentTriangle;

    QLabel *lblSolutionMatrix;

    QLabel *lblPossibleSolutions_Position;

    //Shows the variance over all captured poses of the objects top point
    QLabel *lblTopPoseVariance;

    QPushButton *btnCapture;
    QPushButton *btnPossibleSolutions_Left;
    QPushButton *btnPossibleSolutions_Right;
    QPushButton *btnStartPTUCapture;
    QPushButton *btnAddTopPose;
    QPushButton *btnDeleteTopPose;
    QPushButton *btnClearTopPose;

    QListWidget * lstTopPose;

    int possibleSolutions_Position;

    LaserScanThread *currentThread;

    QRadioButton *radio_polar;
    QRadioButton *radio_flat;
    LaserScanWidget *widget;

    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > *currentTriangles;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > *selectedTriangle;

    //
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > *capturedTopPoses;

    //All the possible solutions for the current triangle
    std::vector<Eigen::Matrix4d,  Eigen::aligned_allocator<Eigen::Matrix4d> > * solutionFrames;

    boost::shared_ptr<ResectionSolver> solver;

    //Used to publish the current results to ros
    boost::shared_ptr<MarkerPublisher> markerPublisher;

    //Contains all the parameters of the calibration object
    boost::shared_ptr<Calibration_Object> calibrationObject;

    boost::shared_ptr<FeasibilityChecker> feasibilityChecker;

    boost::shared_ptr<Transformation_Publisher> transformationPublisher;

    //Used to project the tf frames into the camera image
    boost::shared_ptr<TF_Visualizer> tfVisualizer;

    void addMatrixRow(QString* text, double value);
    void showPossibleSolutions();
    void resizeEvent(QResizeEvent * event);
    void toggleCaptureMode();
    void calculateTransformationFrames();
signals:
    void setCurves(unsigned int showData = 1);
    void newTransform_Laserscan(const Eigen::Matrix4d& transform, double distanceToTop);

public slots:
    void trianglesFound(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > *triangles);

    void chk_rawData_stateChanged(bool state);
    void chk_avgData_stateChanged(bool state);
    void chk_filteredData_stateChanged(bool state);
    void chk_segments_stateChanged(bool state);
    void chk_edges_stateChanged(bool state);

    void radio_polar_stateChanged(bool state);
    void radio_flat_stateChanged(bool state);

    void btnCapture_clicked();

    void btnPossibleSolutions_Left_clicked();
    void btnPossibleSolutions_Right_clicked();
    void btnStartPTUCapture_clicked();

    void btnAddTopPose_clicked();
    void btnDeleteTopPose_clicked();
    void btnClearTopPose_clicked();

};

#endif // MAINWINDOW_H

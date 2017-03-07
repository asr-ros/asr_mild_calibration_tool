/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "GUI/mainwindow.h"
#include <QLabel>
#include <string>

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
    calibrationObject = boost::shared_ptr<Calibration_Object>(new Calibration_Object());
    ros::NodeHandle mNodeHandle(ros::this_node::getName());
    double top_angle_ab, top_angle_bc, top_angle_ca;
    double side_a, side_b, side_c;
    double marker_edge_size;
    std::string marker_id_left, marker_id_right;
    double marker_left_rotation_z, marker_left_transformation_z, marker_left_transformation_x,marker_left_rotation_x;
    double laserscanner_segmentation_lambda, laserscanner_maxVariance, laserscanner_maxAngleDeviation, laserscanner_variationStep;
    double marker_right_rotation_z, marker_right_transformation_z, marker_right_transformation_x,marker_right_rotation_x;
    std::string tf_visualizer_camera_input_topic, tf_visualizer_camera_output_topic, tf_visualizer_frames;
    int laserscanner_variationStepCount;
    mNodeHandle.getParam("angle_ab", top_angle_ab);
    mNodeHandle.getParam("angle_bc", top_angle_bc);
    mNodeHandle.getParam("angle_ca", top_angle_ca);
    mNodeHandle.getParam("side_a", side_a);
    mNodeHandle.getParam("side_b", side_b);
    mNodeHandle.getParam("side_c", side_c);
    mNodeHandle.getParam("marker_edge_size", marker_edge_size);
    mNodeHandle.getParam("marker_ID_left", marker_id_left);
    mNodeHandle.getParam("marker_ID_right", marker_id_right);
    mNodeHandle.getParam("marker_left_rotation_z", marker_left_rotation_z);
    mNodeHandle.getParam("marker_left_transformation_z", marker_left_transformation_z);
    mNodeHandle.getParam("marker_left_transformation_x", marker_left_transformation_x);
    mNodeHandle.getParam("marker_left_rotation_x", marker_left_rotation_x);
    mNodeHandle.getParam("marker_right_rotation_z", marker_right_rotation_z);
    mNodeHandle.getParam("marker_right_transformation_z", marker_right_transformation_z);
    mNodeHandle.getParam("marker_right_transformation_x", marker_right_transformation_x);
    mNodeHandle.getParam("marker_right_rotation_x", marker_right_rotation_x);
    mNodeHandle.getParam("laserscanner_segmentation_lambda", laserscanner_segmentation_lambda);
    mNodeHandle.getParam("laserscanner_maxVariance", laserscanner_maxVariance);
    mNodeHandle.getParam("laserscanner_maxAngleDeviation", laserscanner_maxAngleDeviation);
    mNodeHandle.getParam("laserscanner_variationStep", laserscanner_variationStep);
    mNodeHandle.getParam("tf_visualizer_camera_input_topic", tf_visualizer_camera_input_topic);
    mNodeHandle.getParam("tf_visualizer_camera_output_topic", tf_visualizer_camera_output_topic);
    mNodeHandle.getParam("tf_visualizer_frames", tf_visualizer_frames);
    mNodeHandle.getParam("laserscanner_variationStepCount", laserscanner_variationStepCount);
    calibrationObject->top_angle_ab = top_angle_ab;
    calibrationObject->top_angle_bc = top_angle_bc;
    calibrationObject->top_angle_ca = top_angle_ca;
    calibrationObject->side_a = side_a;
    calibrationObject->side_b = side_b;
    calibrationObject->side_c = side_c;
    calibrationObject->marker_edge_size = marker_edge_size;
    calibrationObject->marker_id_left = marker_id_left;
    calibrationObject->marker_id_right = marker_id_right;

    calibrationObject->marker_left_rotation_z = marker_left_rotation_z;
    calibrationObject->marker_left_transformation_z = marker_left_transformation_z;
    calibrationObject->marker_left_transformation_x = marker_left_transformation_x;
    calibrationObject->marker_left_rotation_x = marker_left_rotation_x;

    calibrationObject->marker_right_rotation_z = marker_right_rotation_z;
    calibrationObject->marker_right_transformation_z = marker_right_transformation_z;
    calibrationObject->marker_right_transformation_x = marker_right_transformation_x;
    calibrationObject->marker_right_rotation_x = marker_right_rotation_x;

    calibrationObject->calculateTransformationFrames();

    ROS_INFO_STREAM("MarkerIDs: " << marker_id_left << ", " << marker_id_right);

    double maxAngleDeviation;
    mNodeHandle.getParam("feasibilityChecker_maxAngleDeviation", maxAngleDeviation);
    feasibilityChecker = boost::shared_ptr<FeasibilityChecker>(new FeasibilityChecker(calibrationObject, Eigen::Vector3d(0.0,0.0,1.0), maxAngleDeviation));
    solver = boost::shared_ptr<ResectionSolver>(new ResectionSolver(calibrationObject, feasibilityChecker));

    transformationPublisher = boost::shared_ptr<Transformation_Publisher>(new Transformation_Publisher(calibrationObject));
    transformationPublisher->start();

    //tf visualization
    std::vector<std::string> frame_ids;
    frame_ids.push_back(tf_visualizer_frames);
    tfVisualizer = boost::shared_ptr<TF_Visualizer>(new TF_Visualizer(frame_ids, tf_visualizer_camera_input_topic, tf_visualizer_camera_output_topic));
    tfVisualizer->start();

    //Laserscan-thread starten
    std::string laserscanner_type;
    mNodeHandle.getParam("laserscanner_type", laserscanner_type);
    ROS_INFO_STREAM("Using laserscanner: " << laserscanner_type);
    if (laserscanner_type == "PLS101")
    {
        currentThread = new LaserScanThread(new Laserscanner_MILD(), laserscanner_segmentation_lambda,laserscanner_maxVariance,laserscanner_maxAngleDeviation,laserscanner_variationStep, laserscanner_variationStepCount);
    }
    else if (laserscanner_type == "LMS400")
    {
        std::string hostname = "192.168.0.1";
        currentThread = new LaserScanThread(new LaserScanner_LMS400(hostname, 10), laserscanner_segmentation_lambda,laserscanner_maxVariance,laserscanner_maxAngleDeviation,laserscanner_variationStep, laserscanner_variationStepCount);
    }
    else
    {
        ROS_ERROR_STREAM("Unknown laserscanner type: " << laserscanner_type);
        return;
    }
    //std::string port = "/dev/ttyS1";
    currentThread->start();

    solutionFrames = new std::vector<Eigen::Matrix4d,  Eigen::aligned_allocator<Eigen::Matrix4d> >();
    selectedTriangle = new std::vector<Eigen::Vector2d,  Eigen::aligned_allocator<Eigen::Vector2d> >();

    showData = 1;
    widget = new LaserScanWidget(this, 0, showData, currentThread);
    widget->move(10,40);

    //Set labels
    lblSolutions = new QLabel(this);
    lblSolutions->setText("");

    lblTriangles = new QLabel(this);
    lblTriangles->setText("<span style='color:red'>Waiting for data...</span>");
    lblTriangles->setWordWrap(true);
    lblTriangles->setAlignment(Qt::AlignTop);

    lblSolutionMatrix = new QLabel(this);
    lblSolutionMatrix->setText("");
    lblSolutionMatrix->setWordWrap(true);
    lblSolutionMatrix->setAlignment(Qt::AlignTop);

    lblPossibleSolutions_Position = new QLabel(this);
    lblPossibleSolutions_Position->setText("");
    lblPossibleSolutions_Position->setWordWrap(true);
    lblPossibleSolutions_Position->setAlignment(Qt::AlignTop);

    lblCurrentTriangle = new QLabel(this);
    lblCurrentTriangle->setText("");
    lblCurrentTriangle->setWordWrap(true);
    lblCurrentTriangle->setAlignment(Qt::AlignTop);

    lblTriangles->resize(200, 60);
    lblSolutions->resize(200, 20);
    lblSolutionMatrix->resize(200, 150);
    lblCurrentTriangle->resize(200, 60);

    //Set buttons
    btnCapture = new QPushButton(this);
    btnCapture->setText("Start capture");
    btnCapture->resize(100, 25);
    btnCapture->setEnabled(false);

    btnPossibleSolutions_Left = new QPushButton(this);
    btnPossibleSolutions_Left->setText("<");
    btnPossibleSolutions_Left->resize(20, 25);
    btnPossibleSolutions_Left->setVisible(false);

    btnPossibleSolutions_Right = new QPushButton(this);
    btnPossibleSolutions_Right->setText(">");
    btnPossibleSolutions_Right->resize(20, 25);
    btnPossibleSolutions_Right->setVisible(false);

    btnStartPTUCapture = new QPushButton(this);
    btnStartPTUCapture->setText("Start PTU");
    btnStartPTUCapture->resize(80, 25);
    btnStartPTUCapture->setVisible(true);

    btnAddTopPose = new QPushButton(this);
    btnAddTopPose->setText("Add");
    btnAddTopPose->resize(60, 25);
    btnAddTopPose->setVisible(true);
    btnAddTopPose->setEnabled(false);

    btnDeleteTopPose = new QPushButton(this);
    btnDeleteTopPose->setText("Delete");
    btnDeleteTopPose->resize(60, 25);
    btnDeleteTopPose->setVisible(true);
    btnDeleteTopPose->setEnabled(false);

    btnClearTopPose = new QPushButton(this);
    btnClearTopPose->setText("Clear");
    btnClearTopPose->resize(60, 25);
    btnClearTopPose->setVisible(true);
    btnClearTopPose->setEnabled(false);

    //Set Listboxes
    lstTopPose = new QListWidget(this);
    lstTopPose->resize(180, 200);
    lstTopPose->setVisible(true);
    lstTopPose->setEnabled(false);

    //Set checkboxes
    chk_rawdata = new QCheckBox(this);
    chk_avgData = new QCheckBox(this);
    chk_filteredData = new QCheckBox(this);
    chk_segments = new QCheckBox(this);
    chk_edges = new QCheckBox(this);
    chk_avgData->setText("Average data");
    chk_rawdata->setText("Raw data");
    chk_filteredData->setText("Filtered data");
    chk_segments->setText("Segments");
    chk_edges->setText("Edges");
    chk_rawdata->setChecked(true);

    //Set radio buttons
    radio_polar = new QRadioButton(this);
    radio_flat = new QRadioButton(this);
    radio_polar->setText("Coordinates");
    radio_flat->setText("Distance");
    radio_polar->resize(180, 30);
    radio_flat->resize(180,30);
    radio_flat->setChecked(true);

    isCapturing = false;

    connect(currentThread, SIGNAL(trianglesFound(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > *)),this, SLOT(trianglesFound(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > *)));

    connect(this, SIGNAL(newTransform_Laserscan(const Eigen::Matrix4d, double)), transformationPublisher.get(), SLOT(newTransform_Laserscan(const Eigen::Matrix4d, double)));

    connect(btnCapture, SIGNAL(clicked()),this, SLOT(btnCapture_clicked()));
    connect(btnPossibleSolutions_Left, SIGNAL(clicked()),this, SLOT(btnPossibleSolutions_Left_clicked()));
    connect(btnPossibleSolutions_Right, SIGNAL(clicked()),this, SLOT(btnPossibleSolutions_Right_clicked()));
    connect(btnStartPTUCapture, SIGNAL(clicked()),this, SLOT(btnStartPTUCapture_clicked()));

    connect(this, SIGNAL(setCurves(unsigned int)),widget, SLOT(setCurves(unsigned int)));

    connect(radio_flat, SIGNAL(clicked(bool)),this, SLOT(radio_flat_stateChanged(bool)));

    connect(radio_polar,SIGNAL(clicked(bool)),this, SLOT(radio_polar_stateChanged(bool)));
    connect(radio_flat, SIGNAL(clicked(bool)),this, SLOT(radio_flat_stateChanged(bool)));

    connect(chk_rawdata, SIGNAL(clicked(bool)),this, SLOT(chk_rawData_stateChanged(bool)));
    connect(chk_avgData, SIGNAL(clicked(bool)),this, SLOT(chk_avgData_stateChanged(bool)));
    connect(chk_filteredData, SIGNAL(clicked(bool)),this, SLOT(chk_filteredData_stateChanged(bool)));
    connect(chk_segments, SIGNAL(clicked(bool)),this, SLOT(chk_segments_stateChanged(bool)));
    connect(chk_edges, SIGNAL(clicked(bool)),this, SLOT(chk_edges_stateChanged(bool)));

    markerPublisher = boost::shared_ptr<MarkerPublisher>(new MarkerPublisher(calibrationObject));
}
void MainWindow::resizeEvent(QResizeEvent * event)
{
    widget->resize(this->width()-220, this->height()-60);
    radio_polar->move(this->width()-200,40);
    radio_flat->move(this->width()-200,65);

    chk_rawdata->move(this->width()-200,100);
    chk_avgData->move(this->width()-200,120);
    chk_filteredData->move(this->width()-200,140);
    chk_segments->move(this->width()-200,160);
    chk_edges->move(this->width()-200,180);

    lblTriangles->move(this->width()-200,220);
    btnCapture->move(this->width()-200,290);
    lblSolutions->move(this->width()-200,320);

    lblSolutionMatrix->move(this->width()-200,340);


    btnPossibleSolutions_Left->move(this->width()-97,424);
    btnPossibleSolutions_Right->move(this->width()-77,424);
    lblPossibleSolutions_Position->move(this->width()-200,427);
    lblCurrentTriangle->move(this->width()-200,470);

    btnStartPTUCapture->move(this->width()-200,530);

    btnAddTopPose->move(this->width()-200,580);
    btnDeleteTopPose->move(this->width()-140,580);
    btnClearTopPose->move(this->width()-80,580);

    lstTopPose->move(this->width()-200,620);

    QMainWindow::resizeEvent(event);
}

void MainWindow::chk_rawData_stateChanged(bool state)
{
    if (state == Qt::Unchecked)  {showData = showData - (showData & 1);}
    else {showData = showData | 1;}
    setCurves(showData);
}
void MainWindow::chk_avgData_stateChanged(bool state)
{
    if (state == Qt::Unchecked) {showData = showData - (showData & 2);}
    else {showData = showData | 2;}
    setCurves(showData);
}
void MainWindow::chk_filteredData_stateChanged(bool state)
{
    if (state == Qt::Unchecked) {showData = showData - (showData & 4);}
    else {showData = showData | 4;}
    setCurves(showData);
}
void MainWindow::chk_segments_stateChanged(bool state)
{
    if (state == Qt::Unchecked) {showData = showData - (showData & 8);}
    else {showData = showData | 8;}
    setCurves(showData);
}
void MainWindow::chk_edges_stateChanged(bool state)
{
    if (state == Qt::Unchecked) {showData = showData - (showData & 16);}
    else {showData = showData | 16;}
    setCurves(showData);
}
void MainWindow::radio_polar_stateChanged(bool state)
{
    if (state == true) widget->setGraph(1, showData);
}
void MainWindow::radio_flat_stateChanged(bool state)
{
   if (state == true) widget->setGraph(0, showData);
}

void MainWindow::btnCapture_clicked()
{
    toggleCaptureMode();
}

void MainWindow::btnAddTopPose_clicked()
{
    if (solutionFrames->size() > 0)
    {
        Eigen::Matrix4d *temp;
        temp = &solutionFrames->at(possibleSolutions_Position);
        QListWidgetItem *newItem = new QListWidgetItem;
        QString text = QString::number((*temp)(0,3)) + ", " + QString::number((*temp)(1,3)) + ", " + QString::number((*temp)(2,3)) ;
        newItem->setText(text);
        lstTopPose->insertItem(lstTopPose->count(), newItem);
    }
}

void MainWindow::btnDeleteTopPose_clicked()
{
    QList<QListWidgetItem *> selected = lstTopPose->selectedItems();
    if (selected.count() > 0)
    {
        for(int i = 0; i < selected.count(); i++)
        {
            //lstTopPose->takeItem(lstTopPose->indexFromItem(selected[i]));
        }
    }
}

void MainWindow::btnClearTopPose_clicked()
{
    lstTopPose->clear();
}

void MainWindow::btnPossibleSolutions_Left_clicked()
{
    possibleSolutions_Position--;
    showPossibleSolutions();
}

void MainWindow::btnPossibleSolutions_Right_clicked()
{
    possibleSolutions_Position++;
    showPossibleSolutions();
}

void MainWindow::btnStartPTUCapture_clicked()
{
    //Clean up
    ROS_INFO("Main Window halting");
    currentThread->stop();
    if (isCapturing) toggleCaptureMode();

    if (possibleSolutions_Position < 0 || possibleSolutions_Position >= (int)solutionFrames->size()) {possibleSolutions_Position = 0;}
    Eigen::Matrix4d temp = solutionFrames->at(possibleSolutions_Position);
    PTUWindow ptuWindow(markerPublisher, calibrationObject, transformationPublisher, temp, this);
    ptuWindow.resize( 1000, 600 );
    ptuWindow.exec();
    //Resume capturing when window is closed
    ROS_INFO("Main Window resuming");
    currentThread->start();
}

void MainWindow::toggleCaptureMode()
{
    if (isCapturing)
    {
        btnCapture->setText("Start capturing");
        if (solutionFrames->size() > 0)
        {
            unsigned int size =  solutionFrames->size();
            QString si;

            if (size == 0)
            {
                si = "<span style='color:red'>No solutions found.";
            }
            else if (size == 1)
            {
                si = "<span style='color:green'>One solution found:";
            }
            else
            {
                si = "<span style='color:yellow'>";
                si +=  QString::number(size);
                si += " solutions found.";
            }
             si += "</span>";

            lblSolutions->setText(si);
            btnStartPTUCapture->setVisible(size > 0);
            possibleSolutions_Position = 0;
            showPossibleSolutions();
            si = "<span>Selected triangle: ";
            for (unsigned int i = 0; i < selectedTriangle->size()/3; i++)
            {
                si = si + "<br/>";
                for (unsigned int j = 0; j < 3; j++)
                {
                    si += "[";
                    si += QString::number(selectedTriangle->at(i*3+j)[0],'.', 2);
                    si += ", ";
                    si += QString::number(selectedTriangle->at(i*3+j)[1],'.', 2);
                    si += "] ";
                }
                si = si + "<br/>";
                double len = sqrt(pow(selectedTriangle->at(i*3)[0]-selectedTriangle->at(i*3+1)[0],2) + pow(selectedTriangle->at(i*3)[1]-selectedTriangle->at(i*3+1)[1],2));
                si += QString::number(len,'.', 3);
                si = si + " ";
                len = sqrt(pow(selectedTriangle->at(i*3+2)[0]-selectedTriangle->at(i*3+1)[0],2) + pow(selectedTriangle->at(i*3+2)[1]-selectedTriangle->at(i*3+1)[1],2));
                si += QString::number(len,'.', 3);
            }
            si += "</span>";
            lblCurrentTriangle->setText(si);
        }
        isCapturing = false;
    }
    else
    {
        btnPossibleSolutions_Right->setVisible(false);
        btnPossibleSolutions_Left->setVisible(false);
        lblPossibleSolutions_Position->setVisible(false);
        lblSolutionMatrix->setText("");
        possibleSolutions_Position = 0;
        solutionFrames->clear();
        lblSolutions->setText("Capturing...");
        lblCurrentTriangle->setText("");
        btnCapture->setText("Stop capturing");
        isCapturing = true;
        ROS_INFO("Capturing...");
    }
}

void MainWindow::calculateTransformationFrames()
{
    Eigen::Matrix4d temp;
    solutionFrames->clear();
    solver->solve(currentTriangles->at(0), currentTriangles->at(1), currentTriangles->at(2));
    for (unsigned i = 0; i< solver->solutions.size(); i++)
    {
        temp = solver->calculateTransformationMatrix(currentTriangles->at(0), currentTriangles->at(1), currentTriangles->at(2), solver->solutions[i]);
        if (feasibilityChecker->checkFeasibility_pose(temp))
        {
            ROS_INFO_STREAM("Triangle " << i << ": Feasability ok");
            solutionFrames->push_back(temp);
        }
    }
    if (solutionFrames->size() > 0)
    {
        ROS_INFO_STREAM("Found solution");
        selectedTriangle->clear();
        selectedTriangle->push_back(currentTriangles->at(0));
        selectedTriangle->push_back(currentTriangles->at(1));
        selectedTriangle->push_back(currentTriangles->at(2));
        toggleCaptureMode();
    }
}

void MainWindow::showPossibleSolutions()
{
    QString siMatrix, siSolutionCounter;
    if (solutionFrames->size() > 0)
    {
        if (possibleSolutions_Position < 0 || possibleSolutions_Position >= (int)solutionFrames->size()) {possibleSolutions_Position = 0;}
        Eigen::Matrix4d *temp;
        siMatrix = "<span> <table style='width:100%'>";
        temp = &solutionFrames->at(possibleSolutions_Position);
        siMatrix += "<tr>";
        addMatrixRow (&siMatrix, (*temp)(0,0) );
        addMatrixRow (&siMatrix, (*temp)(0,1) );
        addMatrixRow (&siMatrix, (*temp)(0,2) );
        addMatrixRow (&siMatrix, (*temp)(0,3) );
        siMatrix += "</tr>";
        siMatrix += "<tr>";
        addMatrixRow (&siMatrix, (*temp)(1,0) );
        addMatrixRow (&siMatrix, (*temp)(1,1) );
        addMatrixRow (&siMatrix, (*temp)(1,2) );
        addMatrixRow (&siMatrix, (*temp)(1,3) );
        siMatrix += "</tr>";
        siMatrix += "<tr>";
        addMatrixRow (&siMatrix, (*temp)(2,0) );
        addMatrixRow (&siMatrix, (*temp)(2,1) );
        addMatrixRow (&siMatrix, (*temp)(2,2) );
        addMatrixRow (&siMatrix, (*temp)(2,3) );
        siMatrix += "</tr>";
        siMatrix += "<tr>";
        addMatrixRow (&siMatrix, (*temp)(3,0) );
        addMatrixRow (&siMatrix, (*temp)(3,1) );
        addMatrixRow (&siMatrix, (*temp)(3,2) );
        addMatrixRow (&siMatrix, (*temp)(3,3) );
        siMatrix += "</tr>";
        siMatrix += "</table></span>";
        //Show distance to top point
        siMatrix = siMatrix + "<br/>";
        siMatrix = siMatrix + "Distance to top: ";
        double distance1 = pow(selectedTriangle->at(0)[0] -(*temp)(0,3), 2);
        distance1 += pow(selectedTriangle->at(0)[1] - (*temp)(1,3), 2);
        distance1 += pow((*temp)(2,3), 2);
        distance1 = sqrt(distance1);
	double distance2 = pow(selectedTriangle->at(1)[0] -(*temp)(0,3), 2);
        distance2 += pow(selectedTriangle->at(1)[1] - (*temp)(1,3), 2);
        distance2 += pow((*temp)(2,3), 2);
        distance2 = sqrt(distance2);
        double distance3 = pow(selectedTriangle->at(2)[0] -(*temp)(0,3), 2);
        distance3 += pow(selectedTriangle->at(2)[1] - (*temp)(1,3), 2);
        distance3 += pow((*temp)(2,3), 2);
        distance3 = sqrt(distance3);

        siMatrix += QString::number(distance1,'.', 3);
	siMatrix += " ";
	siMatrix += QString::number(distance2,'.', 3);
	siMatrix += " ";
	siMatrix += QString::number(distance3,'.', 3);

        btnPossibleSolutions_Right->setVisible((solutionFrames->size() > 1) && possibleSolutions_Position < (int)solutionFrames->size()-1);
        btnPossibleSolutions_Left->setVisible((solutionFrames->size() > 1) && possibleSolutions_Position > 0);
        lblPossibleSolutions_Position->setVisible((solutionFrames->size() > 1));
        siSolutionCounter = "Solution ";
        siSolutionCounter += QString::number(possibleSolutions_Position+1);
        siSolutionCounter += " \\ ";
        siSolutionCounter += QString::number(solutionFrames->size());

        this->newTransform_Laserscan(*temp, distance1);

        markerPublisher->publishTetrahedon(selectedTriangle->at(0), selectedTriangle->at(1), selectedTriangle->at(2),  Eigen::Vector3d((*temp)(0,3), (*temp)(1,3), (*temp)(2,3)));
    }
    lblPossibleSolutions_Position->setText(siSolutionCounter);
    lblSolutionMatrix->setText(siMatrix);
}

void MainWindow::addMatrixRow(QString* text, double value)
{
    (*text) += "<td style='width:25%'>";
    (*text) +=  QString::number(value, ' ', 3);
    (*text) += "</td>";
}

void MainWindow::trianglesFound(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > *triangles)
{
    currentTriangles = triangles;
    unsigned int size =  triangles->size();
    if (size % 3 == 0) {
        size = size / 3;
        QString si;

        if (size == 0)
        {
            si = "<span style='color:red'>No triangles detected.";
            btnCapture->setEnabled(false);
        }
        else if (size == 1)
        {
            si = "<span style='color:green'>One triangle detected:";
            btnCapture->setEnabled(true);
        }
        else
        {
            si = "<span style='color:yellow'>";
            si +=  QString::number(size);
            si += " triangles found.";
            btnCapture->setEnabled(false);
        }
        markerPublisher->publishTriangles(triangles);
        for (unsigned int i = 0; i < size; i++)
        {
            ROS_DEBUG_STREAM("Triangle found at [" << QString::number(triangles->at(i*3)[0]).toStdString() << ", " << QString::number(triangles->at(i*3)[1]).toStdString() << "], "
            << "[" << QString::number(triangles->at(i*3+1)[0]).toStdString() << ", " << QString::number(triangles->at(i*3+1)[1]).toStdString() << "], "
            << "[" << QString::number(triangles->at(i*3+2)[0]).toStdString() << ", " << QString::number(triangles->at(i*3+2)[1]).toStdString() << "]");
            si = si + "<br/>";
            for (unsigned int j = 0; j < 3; j++)
            {
                si += "[";
                si += QString::number(triangles->at(i*3+j)[0],'.', 2);
                si += ", ";
                si += QString::number(triangles->at(i*3+j)[1],'.', 2);
                si += "] ";
            }
        }
        si += "</span>";
        lblTriangles->setText(si);
        if (isCapturing)
        {
            calculateTransformationFrames();
        }
    }
}


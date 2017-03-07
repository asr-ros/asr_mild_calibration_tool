/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef LASERSCANTHREAD_H
#define LASERSCANTHREAD_H
#include <QThread>
#include <vector>

#ifndef Q_MOC_RUN
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <ros/init.h>
#include <Eigen/Dense>
#endif

#include <Laserscanner/abstract_laserscanner.h>
#include <MathHelpers/linearApproximator.h>

using namespace std;
class LaserScanThread : public QThread
{
    Q_OBJECT
public:
    explicit LaserScanThread(Abstract_LaserScanner* scanner, double segmentation_lambda, double maxVarianceFilter, double maxAngleDeviation, double variationStep, int variationStepCount);
    ~LaserScanThread();
    unsigned int data_field_size;
    float angle_spread;
    float starting_angle;
    double *dx;
    double *data_raw;
    double *data_avg;
    double *data_filtered;
    double *data_segments;
    double *data_edges;
    void stop();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:

    static constexpr unsigned int GAUSSIANFIELDSIZE = 20;
    static constexpr  unsigned int AVERAGINGWINDOW = 450;
    static constexpr  unsigned int DETECTIONWINDOW = 10;
    static constexpr double sigma = 1.5;
    //static const double sigma = 0.798;

    unsigned int stepNumber;
    std::vector<float> segments;
    std::vector<int> segmentsizes;
    std::vector<int> segmentdistance;
    double *averagedRanges;
    double *rotatingAccumulationWindow[AVERAGINGWINDOW];
    double *filteredRanges;
    Eigen::Vector2d *coordinates;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > edgesfound;
    double feld[2*GAUSSIANFIELDSIZE+1];
    double gaussFactor;
    double maxVarianceFilter = 0.15;
    double maxAngleDeviation;
    double variationStep = 0.006;
    int variationStepCount = 2;
    double segmentation_lambda = 1.5;

    double markersize = 0.02;

    Abstract_LaserScanner* scanner;

    LinearApproximator * linearApproximator;

    ros::NodeHandle nh;
    ros::Publisher scanDataPublisher;
    ros::Publisher segmentDataPublisher;
    ros::Publisher segmentBorderPublisher;

    void calculateGaussianMatrix();
    void launchSICK();
    void run();
signals:
    void valueChanged();                                                    //Is called everytime new data is available
    void trianglesFound(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > *triangles);           //Is called everytime a triangle is detected
    
public slots:
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);    //Is called everytime the laserscanner has new data
};


#endif // LASERSCANTHREAD_H

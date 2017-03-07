/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <laserscanthread.h>
#include <math.h>
#include <iostream>
#include <limits>
#include <MathHelpers/Resectionsolver/resectionsolver.h>
#ifndef Q_MOC_RUN
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#endif

LaserScanThread::LaserScanThread(Abstract_LaserScanner* scanner, double segmentation_lambda, double maxVarianceFilter, double maxAngleDeviation, double variationStep, int variationStepCount)
{
    this->scanner = scanner;
    data_field_size = scanner->calibration_scan_values;
    angle_spread = scanner->calibration_angle_spread;
    starting_angle = scanner->calibration_starting_angle;
    dx = new double[data_field_size];
    data_raw = new double[data_field_size];
    data_avg = new double[data_field_size];
    data_filtered = new double[data_field_size];
    data_segments = new double[data_field_size];
    data_edges = new double[data_field_size];

    averagedRanges = new double[data_field_size];
    for (unsigned int i = 0; i< AVERAGINGWINDOW; i++)
    {
        rotatingAccumulationWindow[i] = new double[data_field_size];
        for (unsigned int j = 0; j< data_field_size; j++)
        {
            rotatingAccumulationWindow[i][j] = 0.0;
        }
    }
    filteredRanges = new double[data_field_size];
    coordinates = new Eigen::Vector2d[data_field_size];

    for(unsigned int i = 0; i< data_field_size; i++)
    {
        dx[i] = starting_angle + (double)i * (double)angle_spread/data_field_size;
    }

    scanDataPublisher = nh.advertise<sensor_msgs::LaserScan>("average_scan_data", 1000);
    segmentDataPublisher = nh.advertise<visualization_msgs::MarkerArray>("segmented_scan_data", 1000);
    segmentBorderPublisher = nh.advertise<visualization_msgs::MarkerArray>("segment_border_scan_data", 1000);

    linearApproximator = new LinearApproximator();
    this->variationStepCount = variationStepCount;

    ROS_DEBUG_STREAM("segmentation_lambda : " << segmentation_lambda);
    ROS_DEBUG_STREAM("maxVarianceFilter : " << maxVarianceFilter);
    ROS_DEBUG_STREAM("maxAngleDeviation : " << maxAngleDeviation);
    ROS_DEBUG_STREAM("variationStep : " << variationStep);
    ROS_DEBUG_STREAM("variationStepCount : " << variationStepCount);

    this->segmentation_lambda = segmentation_lambda;
    this->maxVarianceFilter = maxVarianceFilter;
    this->maxAngleDeviation = maxAngleDeviation;
    this->variationStep = variationStep;
    calculateGaussianMatrix();
    ROS_INFO("Thread started");
}

LaserScanThread::~LaserScanThread()
{
    for (unsigned int i = 0; i< AVERAGINGWINDOW; i++)
    {
        delete [] rotatingAccumulationWindow[i];
        rotatingAccumulationWindow[i] = NULL;
    }
    delete [] dx;
    dx = NULL;
    delete [] data_raw;
    data_raw = NULL;
    delete [] data_avg;
    data_avg = NULL;
    delete [] data_filtered;
    data_filtered = NULL;
    delete [] data_segments;
    data_segments = NULL;
    delete [] data_edges;
    data_edges = NULL;
    delete [] averagedRanges;
    averagedRanges = NULL;
    delete [] filteredRanges;
    filteredRanges = NULL;
    delete [] coordinates;
    coordinates = NULL;
}

void LaserScanThread::stop()
{
    ROS_INFO("Laserscan thread stopping...");
    scanner->stop();
    ROS_INFO("Lascerscan thread stopped..");
}

void LaserScanThread::calculateGaussianMatrix()
{
    double x;
    gaussFactor = 0;
    for (unsigned int i = 0; i < 2*GAUSSIANFIELDSIZE+1; i++)
    {
            x = (double)(i - (int)((2*GAUSSIANFIELDSIZE+1) / 2));

            feld[i] = exp((-1.0f) * (x * x) / (2 * sigma * sigma));
            feld[i] = feld[i] / (2 * M_PI * sigma * sigma);
            gaussFactor += feld[i];
    }
    gaussFactor = 1/gaussFactor;
}

void LaserScanThread::run()
{
    launchSICK();
}

void LaserScanThread::launchSICK()
{
    ROS_INFO_STREAM("Scanner field size: " << data_field_size);
    stepNumber = 1;
    connect(scanner, SIGNAL(newData(const sensor_msgs::LaserScan::ConstPtr&)), this, SLOT(laserScanCallback(const sensor_msgs::LaserScan::ConstPtr&)));
    ROS_INFO("Scanner connecting");
    scanner->run();
}

void LaserScanThread::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (msg->ranges.size() > 0)
    {
        double oldValue, newValue;
        //Ausgabe Rohdaten & Mittelung
        for(unsigned int i=0;i<msg->ranges.size();i++) {
          //Da die Daten spiegelverkehrt geliefert werden müssen zudem die Werte ungedreht werden
          data_raw[i] = (double)msg->ranges[msg->ranges.size() - i - 1];
          //Mittelung
          oldValue =  rotatingAccumulationWindow[stepNumber][i];
          newValue = data_raw[i];
          averagedRanges[i] = averagedRanges[i] + (newValue - oldValue) / (double)(AVERAGINGWINDOW);
          //Speicherung der neuen Werte im rotierenden Fenster
          rotatingAccumulationWindow[stepNumber][i] = newValue;
        }

        if (stepNumber % 5 == 0)
        {
            /*for(unsigned int i=0;i<msg->ranges.size();i++)
            {
                averagedRanges[i] = averagedRanges[i] -

                averagedRanges[i] = 0;
                for(unsigned int j=0;j<AVERAGINGWINDOW;j++)
                {
                    averagedRanges[i] += rotatingAccumulationWindow[j][i];
                }
                averagedRanges[i] /= (float)AVERAGINGWINDOW;
            }*/


            for(unsigned int i=0;i<msg->ranges.size();i++)
            {
              averagedRanges[i] = 0;
              for(unsigned int j=0;j<AVERAGINGWINDOW;j++)
              {
                 averagedRanges[i] += rotatingAccumulationWindow[j][i];
              }
              averagedRanges[i] /= (float)AVERAGINGWINDOW;
            }
          //Datenfenster verarbeiten
          segments.clear();
          segmentsizes.clear();
          segmentdistance.clear();

          sensor_msgs::LaserScan avgScan = *msg;
          //Ausgabe gemittelte Daten
          for(unsigned int i=0;i<msg->ranges.size();i++) {
            data_avg[i] = (double)averagedRanges[i];
            avgScan.ranges[i] = data_avg[msg->ranges.size() - i - 1];
          }
          scanDataPublisher.publish(avgScan);


          //Gaußfilterung
          double x;
          for(unsigned int i=0;i<GAUSSIANFIELDSIZE;i++) {filteredRanges[i] = averagedRanges[i];}
          for(unsigned int i=GAUSSIANFIELDSIZE;i<data_field_size -GAUSSIANFIELDSIZE;i++) {
              x = 0;
              for (int j = -(int)GAUSSIANFIELDSIZE; j<=(int)GAUSSIANFIELDSIZE; j++)
              {
                  x += gaussFactor*averagedRanges[i+j]*feld[j+GAUSSIANFIELDSIZE];
              }

              filteredRanges[i] = x;
          }
          for(unsigned int i=data_field_size-GAUSSIANFIELDSIZE;i<data_field_size;i++) {filteredRanges[i] = averagedRanges[i];}
          //Ausgabe gefilterte Daten
          for(unsigned int i=0;i<msg->ranges.size();i++) {
              data_filtered[i] = filteredRanges[i];

              //TEST: Gaußfilter ignorieren
              filteredRanges[i] = averagedRanges[i];

              data_segments[i]  = 0;
              data_edges[i]  = 0;
          }
          //Koordinatentransformation
          for(unsigned int i=0;i<data_field_size; i++)
          {
            coordinates[i] = Eigen::Vector2d(sin(msg->angle_increment*i), cos(msg->angle_increment*i)) * filteredRanges[i];
          }

          //Segmentierung
          visualization_msgs::MarkerArray myArray;

          std::vector<unsigned int> segments;
          float distance;
          unsigned int currentPointCloudIndex = 0;
          for (unsigned int i=0;i<data_field_size-1;i++)
          {
              visualization_msgs::Marker marker;
              marker.header.frame_id = "base_laser";
              marker.header.stamp = ros::Time();
              marker.ns = "segmentation";
              marker.id = myArray.markers.size();
              marker.type = visualization_msgs::Marker::SPHERE;
              marker.action = visualization_msgs::Marker::ADD;
              marker.pose.position.x = coordinates[i][0];
              marker.pose.position.y = coordinates[i][1];
              marker.pose.position.z = 0;
              marker.scale.x = markersize;
              marker.scale.y = markersize;
              marker.scale.z = markersize;
              marker.color.a = 1.0;
              marker.color.r = (double)(currentPointCloudIndex % 3 == 0);
              marker.color.g = (double)(currentPointCloudIndex % 3 == 1);
              marker.color.b = (double)(currentPointCloudIndex % 3 == 2);
              myArray.markers.push_back(marker);
              distance = sqrt(pow(coordinates[i][0] - coordinates[i+1][0], 2) +pow(coordinates[i][1]-coordinates[i+1][1], 2));
              //distance = std::abs(filteredRanges[i] * sin(data_field_size*msg->angle_increment) - filteredRanges[i+1]);
              if (distance > segmentation_lambda) {
                  segments.push_back(i);
                  segments.push_back(i+1);
                  data_segments[i] = filteredRanges[i];
                  data_segments[i+1] = filteredRanges[i+1];
                  currentPointCloudIndex ++;
              }
          }
          segmentDataPublisher.publish(myArray);

          edgesfound.clear();
          double varianceLeft, varianceRight, varianceTotal, currentBestVariance;
          bool segmentFound;
          Eigen::Vector2d p1, p2, p3;

          visualization_msgs::MarkerArray borderArray;
          //Suche nach rechtem Winkel
          if (segments.size()> 1)
          {
              for (unsigned int j=0; j<segments.size()-1; j++)
              {
                  if (segments[j+1] - segments[j] >= DETECTIONWINDOW*2+1)       //Nur Bereiche untersuchen, die ausreichend groß sind
                  {
                      currentBestVariance = std::numeric_limits<double>::max();
                      segmentFound = false;
                      //Publish border elements
                      for (unsigned int i = 0;i < DETECTIONWINDOW;i++)
                      {
                          visualization_msgs::Marker leftPoint, rightPoint;
                          leftPoint.header.frame_id = rightPoint.header.frame_id = "base_laser";
                          leftPoint.header.stamp = rightPoint.header.stamp = ros::Time();
                          leftPoint.ns = rightPoint.ns = "segmentation";
                          leftPoint.id =  borderArray.markers.size();
                          rightPoint.id = borderArray.markers.size() + 1;
                          leftPoint.type = rightPoint.type = visualization_msgs::Marker::SPHERE;
                          leftPoint.action = rightPoint.action = visualization_msgs::Marker::ADD;
                          leftPoint.pose.position.x = coordinates[i+segments[j]][0];
                          leftPoint.pose.position.y = coordinates[i+segments[j]][1];
                          rightPoint.pose.position.x = coordinates[-i+segments[j+1]][0];
                          rightPoint.pose.position.y = coordinates[-i+segments[j+1]][1];
                          rightPoint.pose.position.z = leftPoint.pose.position.z = 0;
                          leftPoint.scale.x = rightPoint.scale.x = markersize;
                          leftPoint.scale.y  = rightPoint.scale.y = markersize;
                          leftPoint.scale.z =  rightPoint.scale.z = markersize;
                          leftPoint.color.r = rightPoint.color.r = 0.0;
                          leftPoint.color.g = rightPoint.color.g = 0.0;
                          leftPoint.color.b = rightPoint.color.b = 0.0;
                          leftPoint.color.a = rightPoint.color.a = 1.0;
                          borderArray.markers.push_back(leftPoint);
                          borderArray.markers.push_back(rightPoint);
                      }

                      for (unsigned int i=DETECTIONWINDOW+segments[j];i<segments[j+1]-DETECTIONWINDOW;i++)
                      {
                          visualization_msgs::Marker midPoint;
                          midPoint.header.frame_id =  "base_laser";
                          midPoint.header.stamp = ros::Time();
                          midPoint.ns = "segmentation";
                          midPoint.id =  borderArray.markers.size();
                          midPoint.type = visualization_msgs::Marker::SPHERE;
                          midPoint.action = visualization_msgs::Marker::ADD;
                          midPoint.pose.position.x = coordinates[i][0];
                          midPoint.pose.position.y = coordinates[i][1];
                          midPoint.pose.position.z = 0;
                          midPoint.scale.x = markersize;
                          midPoint.scale.y = markersize;
                          midPoint.scale.z =  markersize;
                          midPoint.color.r = 1.0;
                          midPoint.color.g = 1.0;
                          midPoint.color.b = 1.0;
                          midPoint.color.a = 1.0;
                          borderArray.markers.push_back(midPoint);

                          Eigen::Vector2d center;
                          for (int varHorizontal = -variationStepCount; varHorizontal <= variationStepCount; varHorizontal++)
                          {
                              for (int varVertical = -variationStepCount; varVertical <= variationStepCount; varVertical++)
                              {
                                  center = Eigen::Vector2d((double)varHorizontal*variationStep, (double)varVertical*variationStep) + coordinates[i];
                                  Eigen::Vector2d left, right;
                                  ApproximationResult approximationLeft;
                                  approximationLeft = linearApproximator->calculateApproximation(coordinates, segments[j], i, center);
                                  varianceLeft = approximationLeft.variance;
                                  if (varianceLeft < maxVarianceFilter) //Berechnung der linken Seite
                                  {
                                      left = approximationLeft.approximatedVector;
                                      ApproximationResult approximationRight;
                                      approximationRight = linearApproximator->calculateApproximation(coordinates, i+1, segments[j+1]+1, center);
                                      varianceRight = approximationRight.variance;

                                      varianceTotal = varianceLeft* (i - segments[j]) + varianceRight * (segments[j+1] - i);
                                      varianceTotal /= (double)(segments[j+1] - segments[j] - 1);
                                      if (varianceRight < maxVarianceFilter && currentBestVariance > varianceTotal)
                                      {
                                          right = approximationRight.approximatedVector;
                                          if (fabs(left.dot(right)/(left.norm()*right.norm())) < maxAngleDeviation) {   //Rechter Winkel gefunden
                                              // Nur vom Scanner weg ausgerichtete Winkel erfassen
                                              if (filteredRanges[(int)segments[j]] < filteredRanges[i] && filteredRanges[(int)segments[j+1]] < filteredRanges[i])
                                              {
                                                  currentBestVariance = varianceTotal;
                                                  segmentFound = true;
                                                  p1 = center + left;
                                                  p2 = center;
                                                  p3 = center + right;

                                                  for (unsigned int k = segments[j]; k <= segments[j+1]; k++)
                                                  {
                                                     data_edges[k]  = filteredRanges[k];
                                                  }
                                              }
                                          }
                                      }
                                  }
                              }
                          }
                      }
                      //Wenn Dreieck für aktuelles Segment gefunden wurde, trage dies in die Lösungsliste ein
                      if (segmentFound)
                      {
                          edgesfound.push_back(p1);
                          edgesfound.push_back(p2);
                          edgesfound.push_back(p3);
                      }
                 }
              }

          }
          segmentBorderPublisher.publish(borderArray);
          trianglesFound(&(edgesfound));
        }

        stepNumber ++;
        stepNumber = stepNumber % AVERAGINGWINDOW;
        valueChanged();
    }
}


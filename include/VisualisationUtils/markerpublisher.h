/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef MARKERPUBLISHER_H
#define MARKERPUBLISHER_H
#define NAMESPACE "calibrationObject"
#define NAMESPACEOBJECT "calibrationObject/Object"   //The namespace to which the calibration Object is published
#define NAMESPACEMARKER "calibrationObject/Marker"   //The namespace to which the marker positions are published
#define NAMESPACECAMERA "calibrationObject/Camera"   //The namespace to which thecamera poses are published



#include "calibration_object.h"
#ifndef Q_MOC_RUN
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/init.h>
#include <boost/shared_ptr.hpp>
#endif

struct colouredCameraFrame
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix4d pose;
    std_msgs::ColorRGBA color;
};

//Class used to publish a tetrahedon using markers
class MarkerPublisher
{
public:
    MarkerPublisher(boost::shared_ptr<Calibration_Object> calibrationObject);

    void publishTriangles(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > *triangles);
    void publishTetrahedon(const Eigen::Vector2d &baseA, const Eigen::Vector2d &baseB, const Eigen::Vector2d &baseC, const Eigen::Vector3d &top);
    void publishARMarkers(bool activeLeft, bool activeRight, const Eigen::Matrix4d topFrame);
    void publishColouredCameraFrames(std::vector<colouredCameraFrame, Eigen::aligned_allocator<colouredCameraFrame> > * camFrames);
    void publishCameraFramePointer(const Eigen::Vector3d &startPoint, const Eigen::Vector3d &endPoint, bool isValid);
protected:
    void initialize();
    //Used to publish the active tetrahedon
    ros::Publisher pubObjectPosition;
    ros::Publisher pubMarkerPosition;
    ros::Publisher pubCameraPosition;
    visualization_msgs::Marker markerTop;
    visualization_msgs::Marker markerBottom;
    visualization_msgs::Marker markerTriangles;

    visualization_msgs::Marker markerARMarker_Left;
    visualization_msgs::Marker markerARMarker_Right;

    visualization_msgs::Marker markerCameraFrame;

    std::vector<visualization_msgs::Marker> markerCameraFrames;

    boost::shared_ptr<Calibration_Object> calibrationObject;

    double LINESCALE;
};



#endif // MARKERPUBLISHER_H

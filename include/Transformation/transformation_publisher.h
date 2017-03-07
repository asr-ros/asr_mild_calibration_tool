/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef TRANSFORMATION_PUBLISHER_H
#define TRANSFORMATION_PUBLISHER_H
#include <QThread>

#include <calibration_object.h>

#ifndef Q_MOC_RUN
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/init.h>
#include <boost/shared_ptr.hpp>
#endif

enum Markertype
{
    LEFT,
    RIGHT
};

class Transformation_Publisher : public QThread
{
    Q_OBJECT
public:
    explicit Transformation_Publisher(boost::shared_ptr<Calibration_Object> object);

    void stop();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:

    ros::NodeHandle nh;
    ros::Publisher joint_pub;
    tf::TransformBroadcaster broadcaster;

    std::vector<geometry_msgs::TransformStamped> staticTransforms;
    geometry_msgs::TransformStamped mapToTop;
    geometry_msgs::TransformStamped topToScanFrame;
    geometry_msgs::TransformStamped markerLeftToCamera;
    geometry_msgs::TransformStamped markerRightToCamera;

    Markertype currentMarkertype;

    void run();
    geometry_msgs::Transform matrixToTransform(const Eigen::Matrix4d& matrix);

public slots:
    void newTransform_Laserscan(const Eigen::Matrix4d& transform, double distanceToTop);
    void newTransform_Camera(const Eigen::Matrix4d& transform, Markertype markertype);
};


#endif // TRANSFORMATION_PUBLISHER_H

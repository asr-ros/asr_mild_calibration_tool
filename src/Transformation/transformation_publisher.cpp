/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <Transformation/transformation_publisher.h>
#ifndef Q_MOC_RUN
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#endif

Transformation_Publisher::Transformation_Publisher(boost::shared_ptr<Calibration_Object> object)
{
    joint_pub = nh.advertise<sensor_msgs::JointState>("calibration_transforms", 1);
    geometry_msgs::TransformStamped topToBase;
    geometry_msgs::TransformStamped topToMarkerLeft;
    geometry_msgs::TransformStamped topToMarkerRight;

    topToBase.header.frame_id = "object_top";
    topToBase.child_frame_id = "object_base";
    topToMarkerLeft.header.frame_id = "object_top";
    topToMarkerLeft.child_frame_id = "marker_left";
    topToMarkerRight.header.frame_id = "object_top";
    topToMarkerRight.child_frame_id = "marker_right";
    mapToTop.header.frame_id = "calibration_center";
    mapToTop.child_frame_id = "object_top";
    topToScanFrame.header.frame_id = "object_top";
    topToScanFrame.child_frame_id = "object_scan_frame";
    markerLeftToCamera.header.frame_id = "marker_left";
    markerLeftToCamera.child_frame_id = "camera_frame";
    markerRightToCamera.header.frame_id = "marker_right";
    markerRightToCamera.child_frame_id = "camera_frame";

    topToBase.transform.translation.z = -object->side_b;
    topToBase.transform.rotation =  tf::createQuaternionMsgFromYaw(0.0);
    Eigen::Affine3d eigenTransform = Eigen::Affine3d(object->frame_marker_left);
    tf::Transform tfTransform;
    geometry_msgs::Transform gmTransform;
    tf::transformEigenToTF(eigenTransform, tfTransform);
    tf::transformTFToMsg(tfTransform, gmTransform);
    topToMarkerLeft.transform = gmTransform;
    eigenTransform = Eigen::Affine3d(object->frame_marker_right);
    tf::transformEigenToTF(eigenTransform, tfTransform);
    tf::transformTFToMsg(tfTransform, gmTransform);
    topToMarkerRight.transform = gmTransform;
    mapToTop.transform.rotation =  tf::createQuaternionMsgFromYaw(0.0);
    topToScanFrame.transform.rotation =  tf::createQuaternionMsgFromYaw(0.0);

    staticTransforms.push_back(topToBase);
    staticTransforms.push_back(topToMarkerLeft);
    staticTransforms.push_back(topToMarkerRight);
}

void Transformation_Publisher::newTransform_Laserscan(const Eigen::Matrix4d& transform, double distanceToTop)
{
    Eigen::Affine3d eigenTransform = Eigen::Affine3d(transform);
    tf::Transform tfTransform;
    geometry_msgs::Transform gmTransform;
    tf::transformEigenToTF(eigenTransform, tfTransform);
    tf::transformTFToMsg(tfTransform, gmTransform);
    mapToTop.transform = gmTransform;

    topToScanFrame.transform.translation.z = -distanceToTop;
    topToScanFrame.transform.rotation =  tf::createQuaternionMsgFromYaw(0.0);
}

void Transformation_Publisher::newTransform_Camera(const Eigen::Matrix4d& transform, Markertype markertype)
{
    Eigen::Affine3d eigenTransform = Eigen::Affine3d(transform);
    tf::Transform tfTransform;
    geometry_msgs::Transform gmTransform;
    tf::transformEigenToTF(eigenTransform, tfTransform);
    tf::transformTFToMsg(tfTransform, gmTransform);

    if (markertype == Markertype::LEFT)
    {
        ROS_INFO("Got new camera frame from left marker.");
        markerLeftToCamera.transform = gmTransform;
    }
    else if (markertype == Markertype::RIGHT)
    {
        ROS_INFO("Got new camera frame from right marker.");
        markerRightToCamera.transform = gmTransform;
    }
    currentMarkertype = markertype;
}

void Transformation_Publisher::run()
{
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        //Publish static transforms
        for (unsigned int i = 0; i < staticTransforms.size(); i++)
        {
            staticTransforms[i].header.stamp = ros::Time::now();
            broadcaster.sendTransform(staticTransforms[i]);
        }

        mapToTop.header.stamp = ros::Time::now();
        broadcaster.sendTransform(mapToTop);

        topToScanFrame.header.stamp = ros::Time::now();
        broadcaster.sendTransform(topToScanFrame);

        try
        {
            if (currentMarkertype == Markertype::LEFT)
            {
                markerLeftToCamera.header.stamp = ros::Time::now();
                broadcaster.sendTransform(markerLeftToCamera);
            }
            else if (currentMarkertype == Markertype::RIGHT)
            {
                markerRightToCamera.header.stamp = ros::Time::now();
                broadcaster.sendTransform(markerRightToCamera);
            }
        }
        catch (...)
        {
            ROS_ERROR("Something went wrong in the tf broadcaster");
        }

        loop_rate.sleep();
    }
}

geometry_msgs::Transform Transformation_Publisher::matrixToTransform(const Eigen::Matrix4d& matrix)
{
    Eigen::Affine3d eigenTransform = Eigen::Affine3d(matrix);
    tf::Transform tfTransform;
    geometry_msgs::Transform gmTransform;
    tf::transformEigenToTF(eigenTransform, tfTransform);
    tf::transformTFToMsg(tfTransform, gmTransform);
    return gmTransform;
}

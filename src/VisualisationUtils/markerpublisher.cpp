/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "VisualisationUtils/markerpublisher.h"


MarkerPublisher::MarkerPublisher(boost::shared_ptr<Calibration_Object> calibrationObject)
{
    this->calibrationObject = calibrationObject;
    this->initialize();
    this->LINESCALE = 0.01;
}

void MarkerPublisher::initialize()
{
    char *argv[0];
    int x = 0;
    ros::init(x, argv, NAMESPACE);
    ros::NodeHandle n;
    pubObjectPosition = n.advertise<visualization_msgs::Marker>(NAMESPACEOBJECT, 0);
    pubMarkerPosition = n.advertise<visualization_msgs::Marker>(NAMESPACEMARKER, 0);
    pubCameraPosition = n.advertise<visualization_msgs::Marker>(NAMESPACECAMERA, 0);

    markerTop.header.frame_id = "calibration_center";
    markerTop.ns = NAMESPACEOBJECT;
    markerTop.id = 0;
    markerTop.type = visualization_msgs::Marker::LINE_LIST;
    markerTop.action = visualization_msgs::Marker::ADD;

    markerTop.color.a = 1.0f;
    markerTop.color.r = 1.0f;
    markerTop.color.g = 1.0f;
    markerTop.color.b = 0.0f;
    markerTop.scale.x = 2.0*LINESCALE;
    markerTop.pose.orientation.w = 1.0;

    markerBottom.header.frame_id = "calibration_center";
    markerBottom.ns = NAMESPACEOBJECT;
    markerBottom.id = 1;
    markerBottom.type = visualization_msgs::Marker::LINE_LIST;
    markerBottom.action = visualization_msgs::Marker::ADD;

    markerBottom.color.a = 1.0f;
    markerBottom.color.r = 0.0f;
    markerBottom.color.g = 1.0f;
    markerBottom.color.b = 0.0f;
    markerBottom.scale.x = 2.0*LINESCALE;
    markerBottom.pose.orientation.w = 1.0;

    markerTriangles.header.frame_id = "calibration_center";
    markerTriangles.ns = NAMESPACEOBJECT;
    markerTriangles.id = 2;
    markerTriangles.type = visualization_msgs::Marker::LINE_LIST;
    markerTriangles.action = visualization_msgs::Marker::ADD;

    markerTriangles.color.a = 1.0f;
    markerTriangles.color.r = 1.0f;
    markerTriangles.color.g = 0.0f;
    markerTriangles.color.b = 0.0f;
    markerTriangles.scale.x = LINESCALE;
    markerTriangles.pose.orientation.w = 1.0;

    markerARMarker_Left.header.frame_id = "calibration_center";
    markerARMarker_Left.ns = NAMESPACEMARKER;
    markerARMarker_Left.id = 0;
    markerARMarker_Left.type = visualization_msgs::Marker::LINE_LIST;
    markerARMarker_Left.action = visualization_msgs::Marker::ADD;

    markerARMarker_Left.color.a = 1.0f;
    markerARMarker_Left.color.r = 1.0f;
    markerARMarker_Left.color.g = 0.0f;
    markerARMarker_Left.color.b = 0.0f;
    markerARMarker_Left.scale.x = LINESCALE;
    markerARMarker_Left.pose.orientation.w = 1.0;

    markerARMarker_Right.header.frame_id = "calibration_center";
    markerARMarker_Right.ns = NAMESPACEMARKER;
    markerARMarker_Right.id = 1;
    markerARMarker_Right.type = visualization_msgs::Marker::LINE_LIST;
    markerARMarker_Right.action = visualization_msgs::Marker::ADD;

    markerARMarker_Right.color.a = 1.0f;
    markerARMarker_Right.color.r = 1.0f;
    markerARMarker_Right.color.g = 0.0f;
    markerARMarker_Right.color.b = 0.0f;
    markerARMarker_Right.scale.x = LINESCALE;
    markerARMarker_Right.pose.orientation.w = 1.0;

    //Delete old markers
    visualization_msgs::Marker deleteMarker;
    deleteMarker.action = visualization_msgs::Marker::DELETE;
    deleteMarker.ns = NAMESPACEOBJECT;
    deleteMarker.header.frame_id = "calibration_center";
    deleteMarker.header.stamp = ros::Time();
    deleteMarker.id = 0;
    pubObjectPosition.publish(deleteMarker);
    deleteMarker.id = 1;
    pubObjectPosition.publish(deleteMarker);
    deleteMarker.id = 2;
    pubObjectPosition.publish(deleteMarker);

    visualization_msgs::Marker deleteMarker2;
    deleteMarker2.action = visualization_msgs::Marker::DELETE;
    deleteMarker2.header.frame_id = "calibration_center";
    deleteMarker2.ns = NAMESPACEMARKER;
    deleteMarker2.header.stamp = ros::Time();
    deleteMarker2.id = 0;
    pubMarkerPosition.publish(deleteMarker2);
    deleteMarker2.id = 1;
    pubMarkerPosition.publish(deleteMarker2);

    visualization_msgs::Marker deleteMarker_camera;
    deleteMarker_camera.action = visualization_msgs::Marker::DELETE;
    deleteMarker_camera.header.frame_id = "calibration_center";
    deleteMarker_camera.ns = NAMESPACECAMERA;
    deleteMarker_camera.header.stamp = ros::Time();
    deleteMarker_camera.id = 0;
    pubCameraPosition.publish(deleteMarker_camera);

    ROS_INFO("Marker publisher initialized.");
}
void MarkerPublisher::publishARMarkers(bool activeLeft, bool activeRight, const Eigen::Matrix4d topFrame)
{
    markerARMarker_Left.points.clear();
    markerARMarker_Left.header.stamp = ros::Time();
    markerARMarker_Right.points.clear();
    markerARMarker_Right.header.stamp = ros::Time();

    double edgeSize = calibrationObject->marker_edge_size;

    geometry_msgs::Point A, B, C, D;
    Eigen::Matrix4d markerBase;
    markerBase = topFrame*calibrationObject->frame_marker_left;
    A.x = markerBase(0,3) + markerBase(0,0)*edgeSize*0.5 + markerBase(0,1)*edgeSize*0.5;
    A.y = markerBase(1,3) + markerBase(1,0)*edgeSize*0.5 + markerBase(1,1)*edgeSize*0.5;
    A.z = markerBase(2,3) + markerBase(2,0)*edgeSize*0.5 + markerBase(2,1)*edgeSize*0.5;
    B.x = markerBase(0,3) - markerBase(0,0)*edgeSize*0.5 + markerBase(0,1)*edgeSize*0.5;
    B.y = markerBase(1,3) - markerBase(1,0)*edgeSize*0.5 + markerBase(1,1)*edgeSize*0.5;
    B.z = markerBase(2,3) - markerBase(2,0)*edgeSize*0.5 + markerBase(2,1)*edgeSize*0.5;
    C.x = markerBase(0,3) - markerBase(0,0)*edgeSize*0.5 - markerBase(0,1)*edgeSize*0.5;
    C.y = markerBase(1,3) - markerBase(1,0)*edgeSize*0.5 - markerBase(1,1)*edgeSize*0.5;
    C.z = markerBase(2,3) - markerBase(2,0)*edgeSize*0.5 - markerBase(2,1)*edgeSize*0.5;
    D.x = markerBase(0,3) + markerBase(0,0)*edgeSize*0.5 - markerBase(0,1)*edgeSize*0.5;
    D.y = markerBase(1,3) + markerBase(1,0)*edgeSize*0.5 - markerBase(1,1)*edgeSize*0.5;
    D.z = markerBase(2,3) + markerBase(2,0)*edgeSize*0.5 - markerBase(2,1)*edgeSize*0.5;

    markerARMarker_Left.points.push_back(A);
    markerARMarker_Left.points.push_back(B);
    markerARMarker_Left.points.push_back(B);
    markerARMarker_Left.points.push_back(C);
    markerARMarker_Left.points.push_back(C);
    markerARMarker_Left.points.push_back(D);
    markerARMarker_Left.points.push_back(D);
    markerARMarker_Left.points.push_back(A);

    markerBase = topFrame*calibrationObject->frame_marker_right;
    geometry_msgs::Point A_, B_, C_, D_;
    A_.x = markerBase(0,3) + markerBase(0,0)*edgeSize*0.5 + markerBase(0,1)*edgeSize*0.5;
    A_.y = markerBase(1,3) + markerBase(1,0)*edgeSize*0.5 + markerBase(1,1)*edgeSize*0.5;
    A_.z = markerBase(2,3) + markerBase(2,0)*edgeSize*0.5 + markerBase(2,1)*edgeSize*0.5;
    B_.x = markerBase(0,3) - markerBase(0,0)*edgeSize*0.5 + markerBase(0,1)*edgeSize*0.5;
    B_.y = markerBase(1,3) - markerBase(1,0)*edgeSize*0.5 + markerBase(1,1)*edgeSize*0.5;
    B_.z = markerBase(2,3) - markerBase(2,0)*edgeSize*0.5 + markerBase(2,1)*edgeSize*0.5;
    C_.x = markerBase(0,3) - markerBase(0,0)*edgeSize*0.5 - markerBase(0,1)*edgeSize*0.5;
    C_.y = markerBase(1,3) - markerBase(1,0)*edgeSize*0.5 - markerBase(1,1)*edgeSize*0.5;
    C_.z = markerBase(2,3) - markerBase(2,0)*edgeSize*0.5 - markerBase(2,1)*edgeSize*0.5;
    D_.x = markerBase(0,3) + markerBase(0,0)*edgeSize*0.5 - markerBase(0,1)*edgeSize*0.5;
    D_.y = markerBase(1,3) + markerBase(1,0)*edgeSize*0.5 - markerBase(1,1)*edgeSize*0.5;
    D_.z = markerBase(2,3) + markerBase(2,0)*edgeSize*0.5 - markerBase(2,1)*edgeSize*0.5;

    markerARMarker_Right.points.push_back(A_);
    markerARMarker_Right.points.push_back(B_);
    markerARMarker_Right.points.push_back(B_);
    markerARMarker_Right.points.push_back(C_);
    markerARMarker_Right.points.push_back(C_);
    markerARMarker_Right.points.push_back(D_);
    markerARMarker_Right.points.push_back(D_);
    markerARMarker_Right.points.push_back(A_);

    if (activeLeft)
    {
        markerARMarker_Left.color.r = 0.0f;
        markerARMarker_Left.color.g = 1.0f;
        markerARMarker_Left.color.b = 0.0f;
    }
    else
    {
        markerARMarker_Left.color.r = 1.0f;
        markerARMarker_Left.color.g = 0.0f;
        markerARMarker_Left.color.b = 0.0f;
    }

    if (activeRight)
    {
        markerARMarker_Right.color.r = 0.0f;
        markerARMarker_Right.color.g = 1.0f;
        markerARMarker_Right.color.b = 0.0f;
    }
    else
    {
        markerARMarker_Right.color.r = 1.0f;
        markerARMarker_Right.color.g = 0.0f;
        markerARMarker_Right.color.b = 0.0f;
    }

    if (!activeLeft && !activeRight)
    {
        visualization_msgs::Marker deleteMarker_camera;
        deleteMarker_camera.action = visualization_msgs::Marker::DELETE;
        deleteMarker_camera.header.frame_id = "calibration_center";
        deleteMarker_camera.ns = NAMESPACECAMERA;
        deleteMarker_camera.header.stamp = ros::Time();
        deleteMarker_camera.id = 0;
        pubCameraPosition.publish(deleteMarker_camera);
    }

    pubMarkerPosition.publish(markerARMarker_Left);
    pubMarkerPosition.publish(markerARMarker_Right);
}

void MarkerPublisher::publishColouredCameraFrames(std::vector<colouredCameraFrame, Eigen::aligned_allocator<colouredCameraFrame> > * camFrames)
{  
    ROS_INFO_STREAM("Got "<< camFrames->size() << " markers. ");
    for (unsigned int i = 0; i < camFrames->size(); i++)
    {
        colouredCameraFrame cCF = camFrames->at(i);
        Eigen::Matrix4d tempMatrix = cCF.pose;
        visualization_msgs::Marker tempMarker;
        tempMarker.header.frame_id = "calibration_center";
        tempMarker.ns = NAMESPACECAMERA;
        tempMarker.id = i+1;
        tempMarker.type = visualization_msgs::Marker::ARROW;
        tempMarker.action = visualization_msgs::Marker::ADD;

        tempMarker.color.r = cCF.color.r;
        tempMarker.color.g = cCF.color.g;
        tempMarker.color.b = cCF.color.b;
        tempMarker.color.a = cCF.color.a;

        tempMarker.scale.x = LINESCALE / 8.0;
        tempMarker.scale.y = LINESCALE / 4.0;
        tempMarker.pose.orientation.w = 1.0;
        tempMarker.header.stamp = ros::Time();

        Eigen::Vector3d tempVec(tempMatrix(0,2), tempMatrix(1,2), tempMatrix(2,2));
        tempVec.normalize();
        tempVec = tempVec * LINESCALE*10.0;

        geometry_msgs::Point p1, p2;

        p1.x = tempMatrix(0,3);
        p1.y = tempMatrix(1,3);
        p1.z = tempMatrix(2,3);
        p2.x = tempMatrix(0,3) + tempVec[0];
        p2.y = tempMatrix(1,3) + tempVec[1];
        p2.z = tempMatrix(2,3) + tempVec[2];
        tempMarker.points.push_back(p1);
        tempMarker.points.push_back(p2);

        pubCameraPosition.publish (tempMarker);
    }
}

void MarkerPublisher::publishCameraFramePointer(const Eigen::Vector3d &startPoint, const Eigen::Vector3d &endPoint, bool isValid)
{
    visualization_msgs::Marker arrowMarker;
    arrowMarker.header.frame_id = "calibration_center";
    arrowMarker.ns = NAMESPACECAMERA;
    arrowMarker.id = 0;
    arrowMarker.type = visualization_msgs::Marker::ARROW;
    arrowMarker.action = visualization_msgs::Marker::ADD;
    arrowMarker.color.a = 1.0f;
    if (isValid)
    {
        arrowMarker.color.r = 0.0f;
        arrowMarker.color.g = 1.0f;
    }
    else
    {
        arrowMarker.color.r = 1.0f;
        arrowMarker.color.g = 0.0f;
    }
    arrowMarker.color.b = 0.0f;
    arrowMarker.scale.x = LINESCALE;
    arrowMarker.scale.y = 2.0*LINESCALE;
    arrowMarker.scale.y = 5.0*LINESCALE;
    arrowMarker.pose.orientation.w = 1.0;

    geometry_msgs::Point p1, p2;
    p1.x = startPoint[0];
    p1.y = startPoint[1];
    p1.z = startPoint[2];
    p2.x = endPoint[0];
    p2.y = endPoint[1];
    p2.z = endPoint[2];
    arrowMarker.points.push_back(p1);
    arrowMarker.points.push_back(p2);
    pubCameraPosition.publish (arrowMarker);
}

void MarkerPublisher::publishTetrahedon(const Eigen::Vector2d &baseA, const Eigen::Vector2d &baseB, const Eigen::Vector2d &baseC, const Eigen::Vector3d &top)
{
    markerTop.points.clear();
    markerTop.header.stamp = ros::Time();
    markerBottom.points.clear();
    markerBottom.header.stamp = ros::Time();

    Eigen::Vector3d tempVector;
    geometry_msgs::Point A_,B_, C_, A, B, C, TOP;
    A_.x = baseA[0];
    A_.y = baseA[1];
    A_.z = 0.0;
    B_.x = baseB[0];
    B_.y = baseB[1];
    B_.z = 0.0;
    C_.x = baseC[0];
    C_.y = baseC[1];
    C_.z = 0.0;
    TOP.x = top[0];
    TOP.y = top[1];
    TOP.z = top[2];
    tempVector = Eigen::Vector3d(baseA[0] - top[0], baseA[1] - top[1], - top[2]);
    tempVector.normalize();
    tempVector = tempVector* calibrationObject->side_a + top;
    A.x = tempVector[0];
    A.y = tempVector[1];
    A.z = tempVector[2];
    tempVector = Eigen::Vector3d(baseB[0] - top[0], baseB[1] - top[1], - top[2]);
    tempVector.normalize();
    tempVector = tempVector* calibrationObject->side_b + top;
    B.x = tempVector[0];
    B.y = tempVector[1];
    B.z = tempVector[2];
    tempVector = Eigen::Vector3d(baseC[0] - top[0], baseC[1] - top[1], - top[2]);
    tempVector.normalize();
    tempVector = tempVector* calibrationObject->side_c + top;
    C.x = tempVector[0];
    C.y = tempVector[1];
    C.z = tempVector[2];

    markerTop.points.push_back(A_);
    markerTop.points.push_back(TOP);
    markerTop.points.push_back(B_);
    markerTop.points.push_back(TOP);
    markerTop.points.push_back(C_);
    markerTop.points.push_back(TOP);

    markerBottom.points.push_back(A);
    markerBottom.points.push_back(A_);
    markerBottom.points.push_back(B);
    markerBottom.points.push_back(B_);
    markerBottom.points.push_back(C);
    markerBottom.points.push_back(C_);
    markerBottom.points.push_back(A);
    markerBottom.points.push_back(B);
    markerBottom.points.push_back(B);
    markerBottom.points.push_back(C);

    pubObjectPosition.publish(markerTop);
    pubObjectPosition.publish(markerBottom);
}


void MarkerPublisher::publishTriangles(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > *triangles)
{
    if (triangles->size() > 1)
    {
        markerTriangles.points.clear();
        markerTriangles.header.stamp = ros::Time();
        geometry_msgs::Point A,B;
        A.z = 0.0;
        B.z = 0.0;
        for (unsigned int i = 0 ; i < triangles->size() - 1; i++)
        {
            if (i % 3 != 2)
            { // Dont connect the triangles
                A.x = (triangles->at(i))[0];
                A.y = (triangles->at(i))[1];
                B.x = (triangles->at(i+1))[0];
                B.y = (triangles->at(i+1))[1];

                markerTriangles.points.push_back(A);
                markerTriangles.points.push_back(B);
            }
        }
        pubObjectPosition.publish(markerTriangles);
    }
}


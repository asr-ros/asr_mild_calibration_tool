/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "CameraUtils/marker_manager.h"
#include <QMetaType>

Marker_Manager::Marker_Manager(double timeout, double earlyabort_timeout, QObject *parent) :
    QThread(parent)
{
    qRegisterMetaType<Eigen::Matrix4d>("Eigen::Matrix4d");
    qRegisterMetaType<std::string>("std::string");
    nh = ros::NodeHandle(ros::this_node::getName());

    client_GetRecognizer = nh.serviceClient<asr_aruco_marker_recognition::GetRecognizer>("/asr_aruco_marker_recognition/get_recognizer");
    client_ReleaseRecognizer = nh.serviceClient<asr_aruco_marker_recognition::ReleaseRecognizer>("/asr_aruco_marker_recognition/release_recognizer");

    this->timeout = timeout;
    this->earlyabort_timeout = earlyabort_timeout;
    ROS_INFO_STREAM("Marker Recognition: Timeout = " << timeout);
    ROS_INFO_STREAM("Marker Recognition: Early Abort = " << earlyabort_timeout);
    if (timeout-earlyabort_timeout <= 0.0)
    {
        ROS_ERROR("Marker Recognition: Early Abort was invalid");
        earlyabort_timeout = 0.0;
    }
}

void Marker_Manager::run()
{
    if (getRecognizer())
    {
        //Start recognition
        recognizedMarker = false;

        ros::Duration d = ros::Duration(earlyabort_timeout, 0);
        d.sleep();

        //Early abort
        if (!recognizedMarker)
        {
            ROS_INFO("No Marker found -> Early abort.");
            markerFound("", Eigen::Matrix4d::Identity());
            releaseRecognizer();

        }
        else
        {
            d = ros::Duration(timeout-earlyabort_timeout, 0);
            d.sleep();

            markerFound("", Eigen::Matrix4d::Identity());
            releaseRecognizer();
        }
    }
    else
    {
        ROS_ERROR("Could not start recognition.");
    }
}

void Marker_Manager::markerRecognitionCallback(const asr_msgs::AsrObject::ConstPtr object)
{
    asr_msgs::AsrObject o = *object;
    recognizedMarker = true;
    std::string ID = o.type;
    Eigen::Affine3d transform;
    if(o.sampledPoses.size() > 0)
    {
        tf::poseMsgToEigen(o.sampledPoses.at(0).pose, transform);
        markerFound(ID, transform.matrix());
    }
}

bool Marker_Manager::releaseRecognizer()
{
    a.shutdown();
    asr_aruco_marker_recognition::ReleaseRecognizer release;
    return client_ReleaseRecognizer.call(release);
}

bool Marker_Manager::getRecognizer()
{
    a = nh.subscribe("/stereo/objects", 1000, &Marker_Manager::markerRecognitionCallback, this);
    asr_aruco_marker_recognition::GetRecognizer get;
    return client_GetRecognizer.call(get);
}



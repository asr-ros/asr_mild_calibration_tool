/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <QMetaType>


#include <QThread>
#ifndef Q_MOC_RUN
#include <asr_msgs/AsrObject.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include <asr_aruco_marker_recognition/GetRecognizer.h>
#include <asr_aruco_marker_recognition/ReleaseRecognizer.h>
#endif



class Marker_Manager : public QThread
{
    Q_OBJECT
    public:
      Marker_Manager(double timeout, double earlyabort_timeout, QObject *parent=0);
      void run();
      void markerRecognitionCallback(const asr_msgs::AsrObject::ConstPtr object);

    private:
      ros::ServiceClient client_ReleaseRecognizer;
      ros::ServiceClient client_GetRecognizer;
      ros::NodeHandle nh;

      bool releaseRecognizer();
      bool getRecognizer();
      bool recognizedMarker;
      ros::Subscriber a;
      double timeout;
      double earlyabort_timeout;
    signals:
      void markerFound(std::string markerNumber, const Eigen::Matrix4d &transformation);
};


/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "VisualisationUtils/tfvisualizer.h"
#ifndef Q_MOC_RUN
#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>
#endif

TF_Visualizer::TF_Visualizer(const std::vector<std::string>& frame_ids, std::string camera_input_topic, std::string camera_output_topic)
: it_(nh_), frame_ids_(frame_ids), camera_input_topic_(camera_input_topic), camera_output_topic_(camera_output_topic)
{
}

void TF_Visualizer::imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }

    cam_model_.fromCameraInfo(info_msg);

    BOOST_FOREACH(const std::string& frame_id, frame_ids_) {
      tf::StampedTransform transform;
      try {
        ros::Time acquisition_time = info_msg->header.stamp;
        ros::Duration timeout(1.0 / 30);
        tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id,
                                      acquisition_time, timeout);
        tf_listener_.lookupTransform(cam_model_.tfFrame(), frame_id,
                                     acquisition_time, transform);
      }
      catch (tf::TransformException& ex) {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        return;
      }

      tf::Point pt = transform.getOrigin();
      cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
      cv::Point2d uv;
      uv = cam_model_.project3dToPixel(pt_cv);

      static const int RADIUS = 3;
      cv::circle(image, uv, RADIUS, CV_RGB(255,0,0), -1);
      CvSize text_size;
      int baseline;
      cvGetTextSize(frame_id.c_str(), &font_, &text_size, &baseline);
      CvPoint origin = cvPoint(uv.x - text_size.width / 2,
                               uv.y - RADIUS - baseline - 3);
      cv::putText(image, frame_id.c_str(), origin, cv::FONT_HERSHEY_SIMPLEX, 12, CV_RGB(255,0,0));
    }

    pub_.publish(input_bridge->toImageMsg());
}

void TF_Visualizer::run()
{
    std::string image_topic = nh_.resolveName(camera_input_topic_);
    sub_ = it_.subscribeCamera(image_topic, 1, &TF_Visualizer::imageCb, this);
    pub_ = it_.advertise(camera_output_topic_, 1);
    cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
}

void TF_Visualizer::stop()
{
    //sub_ = null;
    //pub_ = null;
}

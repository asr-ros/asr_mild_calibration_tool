/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "Laserscanner/laserscanner_lms400.h"
#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <stdint.h>      // UINT
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif


LaserScanner_LMS400::LaserScanner_LMS400(std::string hostname, int framerate) : Abstract_LaserScanner()
{
    this->framerate = framerate;
    this->hostname = hostname;
    const char * h = hostname.c_str();
    sickLMS400 = asr_sick_lms_400::asr_sick_lms_400((char *)h, 2111, 2);

    if (sickLMS400.Connect() != 0)
    {
        ROS_ERROR("Could not connect to LMS400!");
    }
    else
    {
        calibration_starting_angle = SCANNER_STARTING_ANGLE;
        calibration_angle_spread = SCANNER_ANGLE_SPREAD;

        sickLMS400.SetResolutionAndFrequency(360.0, SCANNER_TARGET_RESOLUTION, SCANNER_STARTING_ANGLE,SCANNER_ANGLE_SPREAD);
        //LMS400ConfigByAngle( Socket, 3, (char*)"B18244B6",SCANNER_TARGET_RESOLUTION, 2,SCANNER_STARTING_ANGLE, SCANNER_ANGLE_SPREAD, &LMSError, &real_scan_freq, &scanner_actual_resolution, &MQuality);
        //calibration_scan_values = (SCANNER_STARTING_ANGLE/scanner_actual_resolution);
        //calibration_scan_values = 70 / scanner_actual_resolution; //für 0.1
        calibration_scan_values = 560; //für 0.2
        //calibration_scan_values = 140; //für 0.5
        //calibration_scan_values = 70; //für 1.0
        //std::cout << "Value count: " << calibration_scan_values << std::endl;
        sickLMS400.SetMeanFilterParameters(0);
        ROS_INFO("LMS400: Initialization successful");
    }

}

void LaserScanner_LMS400::run()
{
    unsigned int microseconds = 20000;
    ROS_INFO("LMS400: Start Measurement...");
    if (sickLMS400.StartMeasurement(false) != 0)
    {
        ROS_ERROR("LMS400: Could not start Measurement!");
        return;
    }
    active = true;
    sensor_msgs::LaserScan msg;
    unsigned int i = 1;
    while (active)
    {
        msg = sickLMS400.ReadMeasurement();

        ROS_INFO_STREAM("Got " << msg.ranges.size() << " ranges");
        if (msg.ranges.size() == 0)
        {
            ROS_ERROR("Received data is invalid! Using old data instead.");
        }
        else
        {   msg_ptr = sensor_msgs::LaserScan::Ptr(new sensor_msgs::LaserScan());
            msg_ptr->header.stamp = ros::Time();
            msg_ptr->header.frame_id = "map";
            msg_ptr->header.seq = msg.header.seq;
            msg_ptr->header.stamp = ros::Time();
            msg_ptr->angle_min = msg.angle_min;
            msg_ptr->angle_max = msg.angle_max;
            msg_ptr->angle_increment = msg.angle_increment;
            msg_ptr->range_max= msg.range_max;
            msg_ptr->range_min= msg.range_min;
            msg_ptr->time_increment = msg.time_increment;
            msg_ptr->scan_time = msg.scan_time;
            for (unsigned int i = 0; i< msg.ranges.size(); i++)
            {
                msg_ptr->ranges.push_back(msg.ranges[i]);
                //ROS_INFO_STREAM("Range: " << msg.ranges[i]);
            }
        }
        ROS_INFO_STREAM("Range " << i);
        newData(msg_ptr);
        usleep(microseconds);
        ros::spinOnce();
        i++;
    }
}

void LaserScanner_LMS400::stop()
{
    ROS_INFO("LMS400: Stop Measurement...");
    active = false;
    if (sickLMS400.StopMeasurement() != 0)
    {
        ROS_ERROR("LMS400: Could not stop Measurement!");
        return;
    }
}

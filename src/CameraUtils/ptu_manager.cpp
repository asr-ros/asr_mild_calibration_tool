/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "CameraUtils/ptu_manager.h"

#ifndef Q_MOC_RUN
#include <sensor_msgs/JointState.h>
#include <ros/init.h>
#include <ptu_controller/PTUControllerClient.h>
#endif


PTU_Manager::PTU_Manager(double pan_min_angle, double pan_max_angle, double tilt_min_angle, double tilt_max_angle, int pan_step_count, int tilt_step_count)
{  
    ros::NodeHandle n_ = ros::NodeHandle();
    controller = new asr_flir_ptu_controller::PTUController(n_, "ptu_controller");
    //PTUMovementEndedCallback test( boost::bind( &PTU_Manager::ptu_callback, this, _1, _2) );
    //callbackPtr = PTUMovementEndedCallbackPtr(boost::make_shared<PTUMovementEndedCallback>(test));
    pan_step = 0;
    tilt_step = 0;
    pan_step_add = 1;
    this->pan_min_angle = pan_min_angle;
    this->pan_max_angle = pan_max_angle;
    this-> tilt_min_angle = tilt_min_angle;
    this->tilt_max_angle = tilt_max_angle;
    this->pan_stepsize = (pan_max_angle - pan_min_angle) / (double)pan_step_count;
    this->tilt_stepsize = (tilt_max_angle - tilt_min_angle) / (double)tilt_step_count;
}

void PTU_Manager::ptu_callback(double pan, double tilt)
{
    currentPan = pan;
    currentTilt = tilt;
    ptu_moved(pan, tilt);
}

bool PTU_Manager::setStartPose()
{
    pan_step = 0;
    tilt_step = 0;
    sendJoint(pan_min_angle, tilt_min_angle, true);
    return true ;
}

bool PTU_Manager::setNeutralPose()
{
    sendJoint(0,0,true);
    return true;
}

bool PTU_Manager::nextPose()
{
    pan_step += pan_step_add;
    if (pan_step*pan_stepsize + pan_min_angle > pan_max_angle || pan_step < 0)
    {
        pan_step_add *= -1;
        pan_step += pan_step_add;
        tilt_step ++;
    }
    if (tilt_step*tilt_stepsize + tilt_min_angle <= tilt_max_angle)
    {

        sendJoint(pan_min_angle + pan_step*pan_stepsize, tilt_min_angle + tilt_step*tilt_stepsize, false);
        return true;
    }
    else
    {
        return false;
    }
}

void PTU_Manager::sendJoint(double pan, double tilt, bool wait)
{
    ROS_DEBUG_STREAM("PTU: Sending joint to " << pan<< ", " << tilt);
    if (wait)
    {
        //controller->moveTo(pan, tilt);
    }
    else
    {
        //controller->asyncMoveTo(pan, tilt, callbackPtr);
    }
}

double PTU_Manager::getPan()
{
    return currentPan;
}

double PTU_Manager::getTilt()
{
   return currentTilt;
}


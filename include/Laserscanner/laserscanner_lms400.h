/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef LASERSCANNER_LMS400_H
#define LASERSCANNER_LMS400_H

#ifndef Q_MOC_RUN
#include <sick_lms400.h>
#endif
#include "abstract_laserscanner.h"


class LaserScanner_LMS400 : public Abstract_LaserScanner
{
    Q_OBJECT
public:
    LaserScanner_LMS400(std::string hostname, int framerate);
    void run();
    void stop();

signals:
    void newData(const sensor_msgs::LaserScan::ConstPtr& msg);

protected:
    sensor_msgs::LaserScan::Ptr msg_ptr;
    asr_sick_lms_400::asr_sick_lms_400 sickLMS400;
    int framerate;
    bool active;
    static constexpr float SCANNER_TARGET_RESOLUTION = 0.2;
    static constexpr float SCANNER_STARTING_ANGLE = 55;
    static constexpr float SCANNER_ANGLE_SPREAD = 70;

    float scanner_actual_resolution, scanner_max_resolution;
    std::string hostname;
    int	Socket;
    unsigned long LMSError;
    float real_scan_freq, real_angle_res;
    int MQuality;

};


#endif // LASERSCANNER_LMS400_H

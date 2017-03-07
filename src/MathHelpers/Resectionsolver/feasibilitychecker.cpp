/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include <MathHelpers/Resectionsolver/feasibilitychecker.h>
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

FeasibilityChecker::FeasibilityChecker(boost::shared_ptr<Calibration_Object> calibrationObject, const Eigen::Vector3d &expectedOrientationz, double maximumAngleDeviation)
{
    this->calibrationObject = calibrationObject;
    this->expectedOrientation_z = Eigen::Vector4d(expectedOrientationz[0], expectedOrientationz[1], expectedOrientationz[2], 0.0);
    this->maximumAngleDeviation = maximumAngleDeviation;
}

bool FeasibilityChecker::checkFeasibility_sideLengths(double sideA, double sideB, double sideC)
{
    return (sideA <= calibrationObject->side_a && sideB <= calibrationObject->side_b && sideC <= calibrationObject->side_c);
}

bool FeasibilityChecker::checkFeasibility_pose(const Eigen::Matrix4d& pose)
{
    Eigen::Vector4d zAxis(0.0,0.0,1.0,0.0);
    zAxis = pose*zAxis;
    double angleDeviation = 1.0 - std::fabs(zAxis.dot(this->expectedOrientation_z));
    return angleDeviation < maximumAngleDeviation;
}

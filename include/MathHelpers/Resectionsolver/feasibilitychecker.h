/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef FEASIBILITYCHECKER_H
#define FEASIBILITYCHECKER_H

#include "calibration_object.h"

#ifndef Q_MOC_RUN
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#endif

class FeasibilityChecker
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit FeasibilityChecker(boost::shared_ptr<Calibration_Object> calibrationObject, const Eigen::Vector3d &expectedOrientationz, double maximumAngleDeviation);
    bool checkFeasibility_sideLengths(double sideA, double sideB, double sideC);
    bool checkFeasibility_pose(const Eigen::Matrix4d& pose);

protected:
    Eigen::Vector4d expectedOrientation_z;
    double maximumAngleDeviation;
    boost::shared_ptr<Calibration_Object> calibrationObject;
};

#endif // FEASIBILITYCHECKER_H

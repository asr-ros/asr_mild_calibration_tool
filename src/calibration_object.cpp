/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "calibration_object.h"
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

Calibration_Object::Calibration_Object()
{
    calculateTransformationFrames();
}

void Calibration_Object::calculateTransformationFrames()
{
    //Initialize default values

    Eigen::Affine3d transformationLeft = getDHTransformation(marker_left_rotation_z*M_PI/180.0, marker_left_transformation_z, marker_left_rotation_x*M_PI/180.0, marker_left_transformation_x);
    frame_marker_left = transformationLeft.matrix();

    Eigen::Affine3d transformationRight = getDHTransformation(marker_right_rotation_z*M_PI/180.0, marker_right_transformation_z, marker_right_rotation_x*M_PI/180.0, marker_right_transformation_x);
    frame_marker_right = transformationRight.matrix();
    double degreesToRadiant =M_PI/180.0;
    Eigen::Translation<double, 3> translation(0.102046, 0.00162421, 0.101897);
    Eigen::Affine3d rotationA(Eigen::AngleAxisd(77.75461941777554*degreesToRadiant, Eigen::Vector3d(0, 0, 1.0)));
    Eigen::Affine3d rotationB(Eigen::AngleAxisd(-90.30507946507706*degreesToRadiant, Eigen::Vector3d(0.0, 1.0, 0.0)));
    Eigen::Affine3d rotationC(Eigen::AngleAxisd(-77.27385977970101*degreesToRadiant, Eigen::Vector3d(1.0, 0, 0.0)));

    /*Eigen::Translation<double, 3> translation(0.116425, -0.001466, 0.070503);
    Eigen::Affine3d rotationA(Eigen::AngleAxisd(-89.456491*tt, Eigen::Vector3d(0, 0, 1.0)));
    Eigen::Affine3d rotationB(Eigen::AngleAxisd(90.316866*tt, Eigen::Vector3d(1.0, 0, 0.0)));
    Eigen::Affine3d rotationC(Eigen::AngleAxisd(89.785602*tt, Eigen::Vector3d(0, 0, 1.0)));*/
    Eigen::Affine3d temp  = translation * rotationA *rotationB * rotationC;
    frame_marker_right = frame_marker_left * temp.matrix();

    /*Eigen::Translation<double, 3> translation(0, -0.0998, 0.1065);
    Eigen::Affine3d rotationA(Eigen::AngleAxisd(90.802397*tt, Eigen::Vector3d(0, 0, 1.0)));
    Eigen::Affine3d rotationB(Eigen::AngleAxisd(91.552294*tt, Eigen::Vector3d(1.0, 0, 0.0)));
    Eigen::Affine3d rotationC(Eigen::AngleAxisd(-91.280564 *tt, Eigen::Vector3d(0, 0, 1.0)));
    Eigen::Affine3d temp  = translation * rotationA *rotationB * rotationC;*/
}

Eigen::Affine3d Calibration_Object::getDHTransformation(double rotZ, double transZ, double rotX, double transX)
{
    Eigen::Affine3d rotationZ(Eigen::AngleAxisd(rotZ, Eigen::Vector3d(0, 0, 1.0)));
    Eigen::Translation<double, 3> translation(transX, 0.0, transZ);
    Eigen::Affine3d rotationX(Eigen::AngleAxisd(rotX, Eigen::Vector3d(1.0, 0, 0)));
    return rotationZ * translation * rotationX;
}

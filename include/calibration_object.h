/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef CALIBRATION_OBJECT_H
#define CALIBRATION_OBJECT_H
#ifndef Q_MOC_RUN
#include <Eigen/Dense>
#endif

class Calibration_Object
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Calibration_Object();
    Eigen::Matrix4d frame_marker_left;
    Eigen::Matrix4d frame_marker_right;
    double top_angle_ab;
    double top_angle_bc;
    double top_angle_ca;
    double side_a;
    double side_b;
    double side_c;
    double marker_edge_size;

    double marker_left_rotation_z;
    double marker_left_transformation_z;
    double marker_left_transformation_x;
    double marker_left_rotation_x;

    double marker_right_rotation_z;
    double marker_right_transformation_z;
    double marker_right_transformation_x;
    double marker_right_rotation_x;

    std::string marker_id_left;
    std::string marker_id_right;

    void calculateTransformationFrames();

protected:

    Eigen::Affine3d getDHTransformation(double rotZ, double transZ, double rotX, double transX);

    /*static const double CALIBRATION_OBJECT_FLAT_ANGLE_AB = 1.12;
    static const double CALIBRATION_OBJECT_FLAT_ANGLE_BC = 1.12;
    static const double CALIBRATION_OBJECT_FLAT_ANGLE_CA = 1.38;
    static const double CALIBRATION_OBJECT_FLAT_SIDE_A = 1.293;
    static const double CALIBRATION_OBJECT_FLAT_SIDE_B = 0.577;
    static const double CALIBRATION_OBJECT_FLAT_SIDE_C = 1.293;

    static const double CALIBRATION_OBJECT_HIGH_ANGLE_AB = 0.46469;
    static const double CALIBRATION_OBJECT_HIGH_ANGLE_BC = 0.46469;
    static const double CALIBRATION_OBJECT_HIGH_ANGLE_CA = 0.6435;
    static const double CALIBRATION_OBJECT_HIGH_SIDE_A = 1.2786;
    static const double CALIBRATION_OBJECT_HIGH_SIDE_B = 1.143;
    static const double CALIBRATION_OBJECT_HIGH_SIDE_C = 1.2786;

    static const double MARKER_LEFT_TRANSFORMATION_X =0.1;
    static const double MARKER_LEFT_TRANSFORMATION_Y =0.1;
    static const double MARKER_LEFT_ROTATION_Z =0;
    static const double MARKER_RIGHT_TRANSFORMATION_X =0.1;
    static const double MARKER_RIGHT_TRANSFORMATION_Y =0.1;
    static const double MARKER_RIGHT_ROTATION_Z =0;
    static const double MARKER_EDGE_SIZE =0.08;
    static const unsigned int MARKER_ID_LEFT =10;
    static const unsigned int MARKER_ID_RIGHT =9;*/
};

#endif // CALIBRATION_OBJECT_H

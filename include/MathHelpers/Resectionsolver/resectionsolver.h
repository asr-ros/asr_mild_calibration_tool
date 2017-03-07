/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef RESECTIONSOLVER_H
#define RESECTIONSOLVER_H
#include <vector>

#include <MathHelpers/Resectionsolver/feasibilitychecker.h>
#include <calibration_object.h>
#ifndef Q_MOC_RUN
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#endif

class ResectionSolver
{
public:
    ResectionSolver(boost::shared_ptr<Calibration_Object> calibrationObject, boost::shared_ptr<FeasibilityChecker> feasibilityChecker);
    unsigned int solve(const Eigen::Vector2d& base_A, const Eigen::Vector2d& base_B, const Eigen::Vector2d& base_C);
    Eigen::Matrix4d calculateTransformationMatrix(const Eigen::Vector2d& base_A, const Eigen::Vector2d& base_B, const Eigen::Vector2d& base_C, const Eigen::Vector3d& solution);
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > solutions;
    std::vector<double> solutions_a;
    std::vector<double> solutions_b;
    std::vector<double> solutions_c;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
    double q_0, q_1, q_2, q_3, q_4;
    boost::shared_ptr<Calibration_Object> calibrationObject;
    double K_1, K_2;

    boost::shared_ptr<FeasibilityChecker> feasibilityChecker;

    int solve_for_lengths(double side_A, double side_B, double side_C);
    unsigned int solve_for_lengths_(double side_A, double side_B, double side_C);
    Eigen::Vector3d solve_for_top(const Eigen::Vector2d& base_A, const Eigen::Vector2d& base_B, const Eigen::Vector2d& base_C, double length_AS, double length_BS, double length_CS);

};

#endif // RESECTIONSOLVER_H

/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "MathHelpers/Resectionsolver/resectionsolver.h"
#include <math.h>
#include "MathHelpers/Resectionsolver/poly34.h"
#include <vector>
#include <iostream>
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

ResectionSolver::ResectionSolver(boost::shared_ptr<Calibration_Object> calibrationObject, boost::shared_ptr<FeasibilityChecker> feasibilityChecker)
{
    this->calibrationObject = calibrationObject;
    this->feasibilityChecker = feasibilityChecker;
}

unsigned int ResectionSolver::solve(const Eigen::Vector2d& base_A, const Eigen::Vector2d& base_B, const Eigen::Vector2d& base_C)
{
    solutions_a.clear();
    solutions_b.clear();
    solutions_c.clear();
    solve_for_lengths((base_A-base_B).norm(), (base_B-base_C).norm(), (base_C-base_A).norm());
    solutions.clear();

    for (int i = (int)solutions_a.size()-1; i >= 0; i--)
    {
        if (feasibilityChecker->checkFeasibility_sideLengths(solutions_a[i],solutions_c[i],solutions_b[i]))
        {
            Eigen::Vector3d top;
            top = solve_for_top(base_A, base_B, base_C, solutions_b[i], solutions_c[i],solutions_a[i]);
            solutions.push_back(top);
        }
        else
        {
            ROS_INFO_STREAM(solutions_a[i] << ", " << solutions_b[i] << ", " << solutions_c[i] << " erased");
            solutions_a.erase(solutions_a.begin()+i);
            solutions_b.erase(solutions_b.begin()+i);
            solutions_c.erase(solutions_c.begin()+i);
        }

    }
    return solutions.size() ;
}

//Formel aus "New Non-iterative Solution of the Perspective 3-Point Problem" / Huaien ZENG
Eigen::Vector3d ResectionSolver::solve_for_top(const Eigen::Vector2d& base_A, const Eigen::Vector2d& base_B, const Eigen::Vector2d& base_C, double length_AS, double length_BS, double length_CS)
{
    Eigen::Vector2d base_B_new, base_C_new;
    base_B_new = base_B - base_A;
    base_C_new = base_C - base_A;
    double alpha, beta;
    alpha = (pow(base_B_new[0],2.0) + pow(base_B_new[1],2.0) + pow(length_AS,2.0) - pow(length_BS,2.0)) / 2.0;
    // Hinweis: für die Formel von beta steht an dieser Stelle x*x - y*y, sollte aber x*x - z*z heißen
    beta = (pow(base_C_new[0],2.0) + pow(base_C_new[1],2.0) + pow(length_AS,2.0) - pow(length_CS,2.0)) / 2.0;
    double X, Y, Z;
    Y = (alpha*base_C_new[0]-beta*base_B_new[0])/(base_B_new[1]*base_C_new[0]-base_C_new[1]*base_B_new[0]);
    if (base_B_new[0] != 0.0)
    {
        X = (alpha - Y*base_B_new[1])/base_B_new[0];
    }
    else
    {
        X = (beta - Y*base_C_new[1])/base_C_new[0];
    }
    // da die Höhe Z als positiver Wert definiert ist kann hier von einer eindeutigen Lösung für Z ausgegangen werden
    Z = sqrt(pow(length_CS, 2.0) - pow(X - base_C_new[0], 2.0) - pow(Y - base_C_new[1], 2.0));
    return Eigen::Vector3d(X + base_A[0], Y + base_A[1], Z);
}


//Code from https://github.com/Itseez/opencv/blob/master/modules/calib3d/src/p3p.cpp
//http://iplimage.com/blog/p3p-perspective-point-overview/
int ResectionSolver::solve_for_lengths(double side_A, double side_B, double side_C)
{
    ROS_DEBUG_STREAM("Sides: " << side_A << ", " << side_B << ", " << side_C);

    double p = cos(calibrationObject->top_angle_ab) * 2;
    double q = cos(calibrationObject->top_angle_bc) * 2;
    double r = cos(calibrationObject->top_angle_ca) * 2;

    double inv_d22 = 1. / (side_C * side_C);
    double a = inv_d22 * (side_A * side_A);
    double b = inv_d22 * (side_B * side_B);

    double a2 = a * a, b2 = b * b, p2 = p * p, q2 = q * q, r2 = r * r;
    double pr = p * r, pqr = q * pr;

    // Check reality condition (the four points should not be coplanar)
    if (p2 + q2 + r2 - pqr - 1 == 0)
        return 0;

    double ab = a * b, a_2 = 2*a;

    double A = -2 * b + b2 + a2 + 1 + ab*(2 - r2) - a_2;

    // Check reality condition
    if (A == 0) return 0;

    double a_4 = 4*a;

    double B = q*(-2*(ab + a2 + 1 - b) + r2*ab + a_4) + pr*(b - b2 + ab);
    double C = q2 + b2*(r2 + p2 - 2) - b*(p2 + pqr) - ab*(r2 + pqr) + (a2 - a_2)*(2 + q2) + 2;
    double D = pr*(ab-b2+b) + q*((p2-2)*b + 2 * (ab - a2) + a_4 - 2);
    double E = 1 + 2*(b - a - ab) + b2 - b*p2 + a2;

    double temp = (p2*(a-1+b) + r2*(a-1-b) + pqr - a*pqr);
    double b0 = b * temp * temp;
    // Check reality condition
    if (b0 == 0)
        return 0;

    double real_roots[4];
    int n = SolveP4(real_roots, B/A, C/A,D/A, E/A);

    if (n == 0)
        return 0;

    int nb_solutions = 0;
    double r3 = r2*r, pr2 = p*r2, r3q = r3 * q;
    double inv_b0 = 1. / b0;

    // For each solution of x
    for(int i = 0; i < n; i++) {
        double x = real_roots[i];

        // Check reality condition
        if (x <= 0)
            continue;

        double x2 = x*x;

        double b1 =
            ((1-a-b)*x2 + (q*a-q)*x + 1 - a + b) *
            (((r3*(a2 + ab*(2 - r2) - a_2 + b2 - 2*b + 1)) * x +

            (r3q*(2*(b-a2) + a_4 + ab*(r2 - 2) - 2) + pr2*(1 + a2 + 2*(ab-a-b) + r2*(b - b2) + b2))) * x2 +

            (r3*(q2*(1-2*a+a2) + r2*(b2-ab) - a_4 + 2*(a2 - b2) + 2) + r*p2*(b2 + 2*(ab - b - a) + 1 + a2) + pr2*q*(a_4 + 2*(b - ab - a2) - 2 - r2*b)) * x +

            2*r3q*(a_2 - b - a2 + ab - 1) + pr2*(q2 - a_4 + 2*(a2 - b2) + r2*b + q2*(a2 - a_2) + 2) +
            p2*(p*(2*(ab - a - b) + a2 + b2 + 1) + 2*q*r*(b + a_2 - a2 - ab - 1)));

        // Check reality condition
        if (b1 <= 0)
            continue;

        double y = inv_b0 * b1;
        double v = x2 + y*y - x*y*r;

        if (v <= 0)
            continue;

        double Z = side_C / sqrt(v);
        double X = x * Z;
        double Y = y * Z;

        solutions_a.push_back(X);
        solutions_b.push_back(Y);
        solutions_c.push_back(Z);

        nb_solutions++;
    }

    return nb_solutions;
}


Eigen::Matrix4d ResectionSolver::calculateTransformationMatrix(const Eigen::Vector2d& base_A, const Eigen::Vector2d& base_B, const Eigen::Vector2d& base_C, const Eigen::Vector3d& solution)
{
    Eigen::Matrix4d *transformation = new Eigen::Matrix4d();
    //Transformationsmatrix berechnen
    Eigen::Vector3d x_axis, y_axis, z_axis;
    //Z-Achse entspricht dem Vektor vom Schnitt-Punkt 2 bis zu Spitze des Tetraeders
    z_axis = Eigen::Vector3d(solution[0] - base_B[0], solution[1] - base_B[1], solution[2]);
    z_axis.normalize();

    Eigen::Vector3d basePoint = solution - z_axis * calibrationObject->side_b;
    //X- und Y-Achse entsprechen den beiden vorderen Seiten an der Basis des Tetraeders
    //Die Basispunkte können durch die Vektoren zur Spitze hin und den Kantenlängen des Tetraeders bestimmt werden
    x_axis = Eigen::Vector3d(base_A[0] - solution[0], base_A[1] - solution[1], -solution[2]);
    y_axis = Eigen::Vector3d(base_C[0] - solution[0], base_C[1] - solution[1], -solution[2]);
    x_axis.normalize();
    y_axis.normalize();
    x_axis = solution + x_axis * calibrationObject->side_a - basePoint;
    y_axis = solution + y_axis * calibrationObject->side_c - basePoint;
    //Orthonormalisierunsgverfahren um sicherzustellen, dass die Vektoren tatsächlich eine orthonormale Basis bilden
    x_axis = x_axis - x_axis.dot(z_axis)*z_axis;
    x_axis.normalize();
    y_axis = y_axis - y_axis.dot(z_axis)*z_axis - y_axis.dot(x_axis)*x_axis;
    y_axis.normalize();

    (*transformation)(0,0) = x_axis[0];
    (*transformation)(1,0) = x_axis[1];
    (*transformation)(2,0) = x_axis[2];
    (*transformation)(3,0) = 0.0;
    (*transformation)(0,1) = y_axis[0];
    (*transformation)(1,1) = y_axis[1];
    (*transformation)(2,1) = y_axis[2];
    (*transformation)(3,1) = 0.0;
    (*transformation)(0,2) = z_axis[0];
    (*transformation)(1,2) = z_axis[1];
    (*transformation)(2,2) = z_axis[2];
    (*transformation)(3,2) = 0.0;
    (*transformation)(0,3) = solution[0];
    (*transformation)(1,3) = solution[1];
    (*transformation)(2,3) = solution[2];
    (*transformation)(3,3) = 1.0;

    return *transformation;
}




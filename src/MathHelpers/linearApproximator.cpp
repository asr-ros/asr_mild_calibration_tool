/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <MathHelpers/linearApproximator.h>
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

    ApproximationResult LinearApproximator::calculateApproximation(Eigen::Vector2d * functionPoints, int startIndex, int endIndex, Eigen::Vector2d centerOffset)
    {
        double X = 0, Y = 0, Z = 0;
        double alpha;
        for (int i = startIndex; i < endIndex; i++)
        {
            Eigen::Vector2d it = functionPoints[i];
            X += (it[0] - centerOffset[0]) * (it[0] - centerOffset[0]);
            Y += (it[1] - centerOffset[1]) * (it[1] - centerOffset[1]);
            Z += (it[0] - centerOffset[0]) * (it[1] - centerOffset[1]);
        }
        if (X == Y)
        {
            if (Z > 0)
            {
                alpha = M_PI / 4.0;
            }
            else
            {
                alpha = M_PI * 3.0 / 4.0;
            }
        }
        else
        {
            if (Y - X < 0)
            {
                alpha = - atan(2.0 * Z / (Y - X)) * 0.5;
                //ROS_INFO_STREAM("Y - X < 0");
            }
            else
            {
                //ROS_INFO_STREAM("Y - X > 0");
                //alpha = - atan(2.0 * Z / (Y - X)) * 0.5;
                alpha = M_PI / 2.0 - atan(2.0 * Z / (Y - X)) * 0.5;
            }
            if (alpha < M_PI/2.0)
            {
                alpha += M_PI;
            }
        }

        double variance = 0;
        double maxLength = 0;
        for (int i = startIndex; i < endIndex; i++)
        {
            Eigen::Vector2d it = functionPoints[i];
            variance += pow(sin(alpha) * (it[0] - centerOffset[0]) - cos(alpha) * (it[1] - centerOffset[1]), 2.0);
            maxLength = std::max(maxLength, std::abs(cos(alpha) * (it[0] - centerOffset[0]) + sin(alpha) * (it[1] - centerOffset[1])));
        }
        variance /= (double)functionPoints->size();
        ApproximationResult result;
        result.approximatedVector = Eigen::Vector2d(cos(alpha) * maxLength, sin(alpha) * maxLength);
        result.angle = alpha;
        result.variance = variance;
        return result;
    }
/*
    double LinearApproximator::calculateApproximation(std::vector<Eigen::Vector2d> * functionPoints)
    {
        long X = 0, Y = 0, Z = 0;
        double alpha;
        for (std::vector<Eigen::Vector2d>::iterator it = functionPoints->begin(); it < functionPoints->end(); it++)
        {
            X += it[0] * it[0];
            Y += it[1] * it[1];
            Z += it[0] * it[1];
        }

        if (X == Y)
        {
            if (Z > 0)
            {
                alpha = M_PI / 4.0;
            }
            else
            {
                alpha = M_PI * 3.0 / 4.0;
            }
        }
        else
        {
            if (Y - X < 0)
            {
                alpha = - atan(2.0 * (double)Z / (double)(Y - X)) * 0.5;
                if (alpha < 0.0)
                {
                    alpha += M_PI;
                }
            }
            else
            {
                alpha = M_PI / 2.0 - atan(2.0 * (double)Z / (double)(Y - X)) * 0.5;
            }
        }

        variance = 0;
        double maxLength = 0;
        for (std::vector<Eigen::Vector2d>::iterator it = functionPoints->begin(); it < functionPoints->end(); it++)
        {
            variance += pow(sin(alpha) * it[0] + cos(alpha) * it[1], 2.0);
            maxLength = max(maxLength, cos(alpha) * it[0] - sin(alpha) * it[1]);
        }
        approximatedVector = new Eigen::Vector2d(cos(alpha) * maxLength, sin(alpha) * maxLength);
        variance /= (double)functionPoints->size();

        angle = alpha;
        return variance;
    }*/

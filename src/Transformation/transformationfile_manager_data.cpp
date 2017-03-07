/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "Transformation/transformationfile_manager_data.h"
#include <iostream>
#include <fstream>
#include <string>
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#endif

using namespace std;

TransformationFile_Manager_Data::TransformationFile_Manager_Data (string filePath) : Abstract_TransformationFile_Manager(filePath) {}


bool TransformationFile_Manager_Data::writeToFile(Transformation_Data data)
{
    ofstream myfile;
    myfile.open (this->filePath, std::ofstream::out | std::ofstream::app);

    Eigen::Affine3d tr(data.PTU_Frame);
    Eigen::Matrix3d rotation = tr.rotation();

    //Eigen::Vector3d angles = rotation.eulerAngles(2, 0, 2);
    Eigen::Vector3d translation(Eigen::Vector3d(data.PTU_Frame(0, 3), data.PTU_Frame(1, 3), data.PTU_Frame(2, 3)));
    //myfile << angles[0] << "; " << angles[1] << "; " << angles[2] << "; " << translation[0] << "; " << translation[1] << "; " << translation[2] << "; ";

    Eigen::Quaterniond quaternion(rotation);

    myfile << quaternion.x() << "; " << quaternion.y() << "; " <<  quaternion.z() << "; " << quaternion.w() << "; " << translation[0] << "; " << translation[1] << "; " << translation[2]<< "; ";


    /*transform = data.PTU_Frame;
    roll  = atan2f(transform(2,0), transform(0,0));
    pitch = asinf(transform(1,0));
    yaw   = atan2f(-transform(1,2), transform(1,1));
    translation = Eigen::Vector3d(transform(3, 0), transform(3, 1), transform(3, 2));
    myfile << roll << " " << pitch << " " << yaw << " " << translation[0] << " " << translation[1] << " " << translation[2] << " ";
*/
    myfile << data.pan << "; " << data.tilt << std::endl;

    myfile.close();

    return true;
}

std::vector<Transformation_Data> TransformationFile_Manager_Data::readFromFile(string filePath)
{
    std::vector<Transformation_Data> output;
    std::ifstream input(filePath);
    std::string line;
    unsigned int lineNumber = 1;
    while(std::getline( input, line ))
    {
        std::vector<std::string> strs;
        boost::split(strs, line, boost::is_any_of(";"));
        if (strs.size() == 9)
        {
            std::vector<double> numericValues;
            bool success = true;
            for (unsigned int i = 0; i < strs.size(); i++)
            {
                try
                {
                    std::string temp = strs.at(i);
                    boost::algorithm::trim(temp);
                    numericValues.push_back(boost::lexical_cast<double>(temp));
                }
                catch (boost::bad_lexical_cast const&)
                {
                    ROS_ERROR_STREAM("Line " << lineNumber << ": " <<strs.at(i) << " is not a number.");
                    success = false;
                    break;
                }
            }
            if (success)
            {
                Eigen::Affine3d rotation(Eigen::Quaterniond(numericValues.at(3),numericValues.at(0),numericValues.at(1),numericValues.at(2)));
                /*Eigen::Affine3d rotation1(Eigen::AngleAxisd(numericValues.at(0), Eigen::Vector3d(0.0,0.0,1.0)));
                Eigen::Affine3d rotation2(Eigen::AngleAxisd(numericValues.at(1), Eigen::Vector3d(1.0,0.0,0.0)));
                Eigen::Affine3d rotation3(Eigen::AngleAxisd(numericValues.at(2), Eigen::Vector3d(0.0,0.0,1.0)));*/
                Eigen::Affine3d translation(Eigen::Translation3d(Eigen::Vector3d(numericValues.at(4),numericValues.at(5),numericValues.at(6))));
                Eigen::Affine3d transformation = translation * rotation;
                Transformation_Data data;
                data.PTU_Frame = transformation.matrix();
                data.pan = numericValues.at(7);
                data.tilt = numericValues.at(8);
                output.push_back(data);
            }
        }
        else
        {
            ROS_ERROR_STREAM("Line " << lineNumber << ": Expected 8 items, got " << strs.size());
        }
        lineNumber ++;
    }

    return output;
}

std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> TransformationFile_Manager_Data::readTransformationFromFile(string filePath)
{
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> output;
    std::ifstream input(filePath);
    std::string line;
    unsigned int lineNumber = 1;
    while(std::getline( input, line ))
    {
        std::vector<std::string> strs;
        boost::split(strs, line, boost::is_any_of(";"));
        if (strs.size() == 7)
        {
            std::vector<double> numericValues;
            bool success = true;
            for (unsigned int i = 0; i < strs.size(); i++)
            {
                try
                {
                    std::string temp = strs.at(i);
                    boost::algorithm::trim(temp);
                    numericValues.push_back(boost::lexical_cast<double>(temp));
                }
                catch (boost::bad_lexical_cast const&)
                {
                    ROS_ERROR_STREAM("Line " << lineNumber << ": " <<strs.at(i) << " is not a number.");
                    success = false;
                    break;
                }
            }
            if (success)
            {
                Eigen::Affine3d rotation(Eigen::Quaterniond(numericValues.at(3),numericValues.at(0),numericValues.at(1),numericValues.at(2)));
                /*Eigen::Affine3d rotation1(Eigen::AngleAxisd(numericValues.at(0), Eigen::Vector3d(0.0,0.0,1.0)));
                Eigen::Affine3d rotation2(Eigen::AngleAxisd(numericValues.at(1), Eigen::Vector3d(1.0,0.0,0.0)));
                Eigen::Affine3d rotation3(Eigen::AngleAxisd(numericValues.at(2), Eigen::Vector3d(0.0,0.0,1.0)));*/
                Eigen::Affine3d translation(Eigen::Translation3d(Eigen::Vector3d(numericValues.at(4),numericValues.at(5),numericValues.at(6))));
                Eigen::Affine3d transformation = rotation * translation;
                Eigen::Matrix4d data;
                data = transformation.matrix();
                output.push_back(data);
            }
        }
        else
        {
            ROS_ERROR_STREAM("Line " << lineNumber << ": Expected 7 items, got " << strs.size());
        }
        lineNumber ++;
    }

    return output;
}

bool TransformationFile_Manager_Data::writeTransformationToFile(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> transformation)
{
    ofstream myfile;
    myfile.open (this->filePath, std::ofstream::out | std::ofstream::app);
    for (unsigned int i = 0; i < transformation.size(); i++)
    {

        Eigen::Matrix4d temp = transformation.at(i);
        Eigen::Affine3d tr(temp);
        Eigen::Matrix3d rotation = tr.rotation();
        Eigen::Quaterniond quaternion(rotation);
        Eigen::Vector3d translation(Eigen::Vector3d(temp(0, 3), temp(1, 3), temp(2, 3)));

        myfile << quaternion.x() << "; " << quaternion.y() << "; " <<  quaternion.z() << "; " << quaternion.w() << "; " << translation[0] << "; " << translation[1] << "; " << translation[2] << std::endl;

        /*!!!Angles!!!
        Eigen::Vector3d angles = rotation.eulerAngles(2, 0, 2);

        myfile << angles[0] << "; " << angles[1] << "; " << angles[2] << "; " << translation[0] << "; " << translation[1] << "; " << translation[2] << std::endl;
        */

    }
    myfile.close();


    return true;
}

bool TransformationFile_Manager_Data::writeCameraOffsetToFile(const PanTiltOffsetTupleList &offsetData)
{
    ofstream myfile;
    myfile.open (this->filePath, std::ofstream::out | std::ofstream::app);
    for (unsigned int i = 0; i < offsetData.size(); i++)
    {
        PanTiltOffsetTuple currentTuple = offsetData.at(i);
        double pan = std::get<0>(currentTuple);
        double tilt = std::get<1>(currentTuple);
        Eigen::Matrix4d temp = std::get<2>(currentTuple);
        Eigen::Matrix4d tempOld = std::get<3>(currentTuple);

        myfile << pan << "; " << tilt << "; " << temp(0,3) << "; " << temp(1,3) << "; " << temp(2,3) << "; "<< tempOld(0,3) << "; " << tempOld(1,3) << "; " << tempOld(2,3) << "; " << std::endl;
        //myfile << quaternion.x() << "; " << quaternion.y() << "; " <<  quaternion.z() << "; " << quaternion.w() << "; " << translation[0] << "; " << translation[1] << "; " << translation[2] << std::endl;

        /*!!!Angles!!!
        Eigen::Vector3d angles = rotation.eulerAngles(2, 0, 2);

        myfile << angles[0] << "; " << angles[1] << "; " << angles[2] << "; " << translation[0] << "; " << translation[1] << "; " << translation[2] << std::endl;
        */

    }
    myfile.close();

    return true;
}

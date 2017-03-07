/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/init.h>
#endif
#include <Optimization/abstract_optimizer.h>
#include <Transformation/transformationfile_manager_XML.h>

/*
 * optimization_main.cpp
 *
 *  Created on: 16.09.2015
 *      Author: Florian Aumann
 */
std::vector<Transformation_Data> readTransformationFile(string fileName)
{
    std::vector<Transformation_Data> data;
    ROS_INFO_STREAM("Reading file " << fileName << "...");
    std::ifstream infile(fileName);
    if (infile.good())
    {
        TransformationFile_Manager_XML fileManager(fileName);
        try
        {
            data = fileManager.readFromFile();
        }
        catch (...)
        {
            ROS_ERROR("Could not read file.");
        }
    }
    else
    {
        ROS_ERROR("File does not exist.");
    }
    return data;
}


int main(int argc, char *argv[])
{
    char *arg[0];
    int x = 0;
    ROS_INFO("Init ROS");
    ros::init(x, arg, "optimizer");
    if (argc > 0)
    {
        **++argv;
        std::string fileName = *argv;
        std::vector<Transformation_Data> transformationData = readTransformationFile(fileName);
        ROS_INFO_STREAM("Got " << transformationData.size() << " datasets.");
    }
    else
    {
        ROS_ERROR("No data file specified.");
    }
    return 0;
}






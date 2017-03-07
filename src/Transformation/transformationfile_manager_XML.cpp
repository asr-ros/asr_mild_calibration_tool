/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "Transformation/transformationfile_manager_XML.h"
#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#ifndef Q_MOC_RUN
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#endif

using namespace std;

TransformationFile_Manager_XML::TransformationFile_Manager_XML (string filePath) : Abstract_TransformationFile_Manager(filePath) {}

bool TransformationFile_Manager_XML::writeToFile(Transformation_Data data)
{    
    boost::property_tree::ptree pt;
    ifstream infile(this->filePath);
    if (infile.good()) boost::property_tree::read_xml(this->filePath, pt); //If file exists, append new data

    boost::property_tree::ptree dataNode;
    boost::property_tree::ptree transformLaserScan_Node;
    boost::property_tree::ptree transformPTU_Node;

    dataNode.put<double>("<xmlattr>.Pan", data.pan);
    dataNode.put<double>("<xmlattr>.Tilt", data.tilt);

    for (unsigned int i = 0; i < 4; i++)
    {
        for (unsigned int j = 0; j < 4; j++)
        {
            std::string name = "<xmlattr>.x_" + std::to_string(i) + std::to_string(j);
            transformLaserScan_Node.put<double>(name, data.LaserScan_Frame(i,j));
            transformPTU_Node.put<double>(name, data.PTU_Frame(i,j));
        }
    }
    dataNode.add_child("FrameLaserscan", transformLaserScan_Node);
    dataNode.add_child("FramePTU", transformPTU_Node);
    pt.add_child("TransformationData.DataSet", dataNode);
    //boost::property_tree::xml_writer_settings<char> settings('\t', 1);
    boost::property_tree::write_xml(this->filePath, pt,std::locale());
    return true;
}

std::vector<Transformation_Data> TransformationFile_Manager_XML::readFromFile(string filePath)
{
    std::vector<Transformation_Data> output;
    boost::property_tree::ptree pt;
    //open the archive
    read_xml(filePath, pt);

    BOOST_FOREACH(boost::property_tree::ptree::value_type &node, pt.get_child("TransformationData"))
    {
        if (node.first == "DataSet")
        {
            Transformation_Data data;
            data.pan = node.second.get<double>("<xmlattr>.Pan");
            data.tilt = node.second.get<double>("<xmlattr>.Tilt");
            boost::property_tree::ptree transformLaserScan_Node = node.second.get_child("FrameLaserscan");
            boost::property_tree::ptree transformPTU_Node = node.second.get_child("FramePTU");
            for (unsigned int i = 0; i < 4; i++)
            {
                for (unsigned int j = 0; j < 4; j++)
                {
                    std::string name = "<xmlattr>.x_" + std::to_string(i) + std::to_string(j);
                    data.LaserScan_Frame(i, j) = transformLaserScan_Node.get<double>(name);
                    data.PTU_Frame(i, j) = transformPTU_Node.get<double>(name);
                }
            }
            output.push_back(data);
        }
    }
    return output;
}

std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> TransformationFile_Manager_XML::readTransformationFromFile(string filePath)
{
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> re;
    return re;
}

bool TransformationFile_Manager_XML::writeTransformationToFile(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> transformation)
{
    return false;
}

bool TransformationFile_Manager_XML::writeCameraOffsetToFile(const PanTiltOffsetTupleList &offsetData)
{
    return true;
}

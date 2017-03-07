/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef TRANSFORMATIONWRITERXML_H
#define TRANSFORMATIONWRITERXML_H
#include <Transformation/abstract_transformationfile_manager.h>

//This class is used to export a pair of Frames (PTU->Object, Laserscanner->Object)
//along with the current configurations of the PTU (Pan, Tilt) to an xml file

class TransformationFile_Manager_XML : public Abstract_TransformationFile_Manager
{
public:
    TransformationFile_Manager_XML(string filePath);
    bool writeToFile(Transformation_Data data);
    std::vector<Transformation_Data> readFromFile(string filePath);
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> readTransformationFromFile(string filePath);
    bool writeTransformationToFile(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> transformation);
    bool writeCameraOffsetToFile(const PanTiltOffsetTupleList &offsetData);

    using Abstract_TransformationFile_Manager::writeToFile;
    using Abstract_TransformationFile_Manager::readFromFile;
};

#endif // TRANSFORMATIONWRITERXML_H

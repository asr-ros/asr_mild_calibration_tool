/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "Transformation/abstract_transformationfile_manager.h"

Abstract_TransformationFile_Manager::Abstract_TransformationFile_Manager(string filePath)
{
    this->filePath = filePath;
}

bool Abstract_TransformationFile_Manager::writeToFile(std::vector<Transformation_Data> &dataSets)
{
    bool success = true;
    for (std::vector<Transformation_Data>::iterator dataSet = dataSets.begin(); dataSet < dataSets.end(); dataSet++)
    {
        success &= this->writeToFile(*dataSet);
    }
    return success;
}

bool Abstract_TransformationFile_Manager::writeToFile(const Eigen::Matrix4d PTU_Frame, const Eigen::Matrix4d LaserScan_Frame, double pan, double tilt)
{
    Transformation_Data data(PTU_Frame, LaserScan_Frame, pan, tilt);
    return writeToFile(data);
}

std::vector<Transformation_Data> Abstract_TransformationFile_Manager::readFromFile()
{
    return this->readFromFile(this->filePath);
}

/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef ABSTRACTTRANSFORMATIONWRITER_H
#define ABSTRACTTRANSFORMATIONWRITER_H
#include <iostream>
#ifndef Q_MOC_RUN
#include <Eigen/Dense>
#endif
#include "transformation_data.h"

using namespace std;

typedef std::tuple<double, double, Eigen::Matrix4d, Eigen::Matrix4d> PanTiltOffsetTuple;
typedef std::vector<PanTiltOffsetTuple,Eigen::aligned_allocator<Eigen::Vector4f> > PanTiltOffsetTupleList;

class Abstract_TransformationFile_Manager
{
public:
    explicit Abstract_TransformationFile_Manager(string filePath);
    virtual std::vector<Transformation_Data> readFromFile(string filePath) = 0;
    virtual bool writeToFile(Transformation_Data data) = 0;
    virtual std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> readTransformationFromFile(string filePath) = 0;
    virtual bool writeTransformationToFile(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> transformation) = 0;
    virtual bool writeCameraOffsetToFile(const PanTiltOffsetTupleList &offsetData) = 0;

    bool writeToFile(std::vector<Transformation_Data> &dataSets);
    bool writeToFile(const Eigen::Matrix4d PTU_Frame, const Eigen::Matrix4d LaserScan_Frame, double pan, double tilt);
    std::vector<Transformation_Data> readFromFile();
protected:
    string filePath;
};

#endif // ABSTRACTTRANSFORMATIONWRITER_H

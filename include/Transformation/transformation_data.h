/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef TRANSFORMATION_DATA_H
#define TRANSFORMATION_DATA_H

#include <fstream>
#ifndef Q_MOC_RUN
#include <Eigen/Core>
#endif

//Add the Eigen Matrix class to boost, so it knows how to serialize it
    /*#ifndef EIGEN_BOOST_SERIALIZATION
    #define EIGEN_BOOST_SERIALIZATION
    #include <Eigen/Dense>
    #include <boost/serialization/split_free.hpp>
    #include <boost/serialization/vector.hpp>

    namespace boost{namespace serialization{
        template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
            void save(Archive & ar, const Eigen::Matrix<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int version) {
                int rows=m.rows(),cols=m.cols();
                ar & rows;
                ar & cols;
                ar & boost::serialization::make_array(m.data(), rows*cols);
            }
        template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
            void load(Archive & ar, Eigen::Matrix<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int version) {
                int rows,cols;
                ar & rows;
                ar & cols;
                m.resize(rows,cols);
                ar & boost::serialization::make_array(m.data(), rows*cols);
            }

        template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
            void serialize(Archive & ar, Eigen::Matrix<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int version) {
                split_free(ar,m,version);
            }

    }}
    #endif*/

//---------------------------------------------
// Begin of the actual class
//---------------------------------------------
class Transformation_Data
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Transformation_Data();
    Transformation_Data(Eigen::Matrix4d PTU_Frame, Eigen::Matrix4d LaserScan_Frame, double pan,double tilt);
    Eigen::Matrix4d PTU_Frame;
    Eigen::Matrix4d LaserScan_Frame;
    double pan;
    double tilt;
private:
    /*friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & pan;
        ar & tilt;
        ar & PTU_Frame;
        ar & LaserScan_Frame;
    }*/
};

#endif // TRANSFORMATION_DATA_H

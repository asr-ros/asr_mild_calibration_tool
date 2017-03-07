/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <iostream>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <vector>
#include <MathHelpers/Resectionsolver/resectionsolver.h>
#include <iostream>     //inp
#include <unistd.h>
#include <QObject>
#include <math.h>
#include <Transformation/transformationfile_manager_XML.h>
#include <Transformation/transformationfile_manager_data.h>
#include <Transformation/transformation_data.h>
#ifndef Q_MOC_RUN
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/init.h>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#endif
#include <CameraUtils/marker_manager.h>

/*class MarkerDetectionHelper : public QObject
{

   public slots:
     void markerFound(int markerNumber, Eigen::Matrix4d transformation)
     {

     }
};*/


using namespace std;
string workspacePath;

bool test_averageframe(string filename)
{
    std::ifstream input(filename);
    std::string line;
    unsigned int lineNumber = 1;
    std::vector<Eigen::Quaterniond> rotationVector;
    std::vector<Eigen::Vector3d> translationVector;
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
                Eigen::Quaterniond rotation(numericValues.at(3),numericValues.at(0),numericValues.at(1),numericValues.at(2));
                Eigen::Vector3d translation(numericValues.at(4),numericValues.at(5),numericValues.at(6));
                rotationVector.push_back(rotation);
                translationVector.push_back(translation);
            }
        }
        else
        {
            ROS_ERROR_STREAM("Line " << lineNumber << ": Expected 7 items, got " << strs.size());
        }
        lineNumber ++;
    }
    if (rotationVector.size() > 0)
    {
        Eigen::Vector3d averageTranslation = translationVector.at(0);
        Eigen::Quaterniond averageRotation = rotationVector.at(0);

        for (unsigned int i = 1; i < rotationVector.size(); i++)
        {
            Eigen::Quaterniond tempRotation  = rotationVector.at(i);
            Eigen::Vector3d tempTranslation = translationVector.at(i);
            averageTranslation = averageTranslation + tempTranslation;
            averageRotation = averageRotation.slerp((double)i/(i+1.0), tempRotation);
        }
        averageTranslation = averageTranslation * 1.0/(double)rotationVector.size();
        ROS_INFO_STREAM("Position X=" << averageTranslation(0) << " Y=" << averageTranslation(1) << " Z=" << averageTranslation(2));
        ROS_INFO_STREAM("Rotation w=" << averageRotation.w() << " x=" << averageRotation.x() << " y=" << averageRotation.y() << " z=" << averageRotation.z());
    }
    return true;
}

bool test_markerDetection()
{
    bool success = false;
    Marker_Manager markerManager(4.0, 3.0);
    //MarkerDetectionHelper helper;
    //connect(markerManager, SIGNAL(markerFound(int,Eigen::Matrix4d )),helper, SLOT(markerFound(int, Eigen::Matrix4d )));

    return success;
}

bool test_serialisation(Abstract_TransformationFile_Manager * manager)
{
    double pan, tilt;
    bool success = false;
    unsigned int testCount = 3;
    Eigen::Matrix4d m1_array[testCount];
    double pan_array[testCount];
    double tilt_array[testCount];
    for (unsigned int k = 0; k < testCount; k++)
    {
        Eigen::Matrix4d m1, m2;
        pan = rand() % 1000;
        tilt = rand() & 1000;
        pan_array[k] = pan;
        tilt_array[k] = tilt;
        double roll = fmod((double)rand(),2.0*M_PI);
        double pitch = fmod((double)rand(),2.0*M_PI);
        double yaw = fmod((double)rand(),2.0*M_PI);
        ROS_INFO_STREAM("Roll: " << roll << " pitch: " << pitch << " yaw: " << yaw);
        Eigen::Affine3d tr1 = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(rand() % 1000, rand()% 1000, rand()% 1000);
        roll = fmod((double)rand(),2.0*M_PI);
        pitch = fmod((double)rand(),2.0*M_PI);
        yaw = fmod((double)rand(),2.0*M_PI);
        ROS_INFO_STREAM("Roll: " << roll << " pitch: " << pitch << " yaw: " << yaw);
        Eigen::Affine3d tr2 = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(rand()% 1000, rand()% 1000, rand()% 1000);
        m1 = tr1.matrix();
        m2 = tr2.matrix();
        m1_array[k] = m1;
        manager->writeToFile(m1, m2, pan, tilt);
    }

    std::vector<Transformation_Data> dataCollection = manager->readFromFile();
    if (dataCollection.size() >= testCount)
    {
        success = true;
        for (unsigned int k = 0; k < testCount; k++)
        {
            Eigen::Matrix4d m1;
            m1 = m1_array[k];
            pan = pan_array[k];
            tilt = tilt_array[k];
            Transformation_Data data = dataCollection[k];
            success &= (fabs(pan - data.pan) < 0.00001);
            if (success) {cout << "Pan correct" <<  endl;}
            else {ROS_ERROR_STREAM("Pan incorrect. Expected " << pan << ", got " << data.pan << " difference " << fabs(pan - data.pan));return success;}
            success &= (fabs(tilt - data.tilt) < 0.00001);
            if (success) {cout << "Tilt correct" <<  endl;}
            else {ROS_ERROR_STREAM("Tilt incorrect. Expected " << tilt << ", got " << data.tilt << " difference " << fabs(tilt - data.tilt));return success;}
            for (unsigned int i = 0; i < 4; i++)
            {
                for (unsigned int j = 0; j < 4; j++)
                {
                    success &= (fabs(m1(i,j) - data.PTU_Frame(i,j)) < 0.001);
                    if (!success) {ROS_ERROR_STREAM("PTU frame incorrect. Expected " << m1(i,j) << ", got " << data.PTU_Frame(i,j) << " difference " << fabs(m1(i,j) - data.PTU_Frame(i,j)));return success;}
                }
            }
            if (success) {cout << "PTU frame correct" <<  endl;}
        }
    }
    else
    {
        ROS_ERROR_STREAM("Not enough dataNodes found. Expected " << testCount << ", got " << dataCollection.size());
    }
    return success;
}

bool test_resectionsolver()
{
     Eigen::Vector3d top, temp;
     Eigen::Vector2d base1, base2, base3;
     double angle12, angle23, angle31;
     double side1, side2, side3, sideBase;

     bool success;
     /* initialize random seed: */

         top[0] = (rand() % 1000 + 1) /100.0;
         top[1] = (rand() % 1000 + 1) /100.0;
         top[2] = (rand() % 1000 + 1) /500.0;

         base1[0]  = (rand() % 1000 + 1) /100.0;
         base1[1]  = (rand() % 1000 + 1) /100.0;
         base2[0]  = (rand() % 1000 + 1) /100.0;
         base2[1]  = (rand() % 1000 + 1) /100.0;
         base3[0]  = (rand() % 1000 + 1) /100.0;
         // Nur rechten Winkel zulassen
         base3[1]  = (base1[0]*base3[0]-base2[0]*base3[0]-base1[0]*base2[0]+base2[0]*base2[0]-base1[1]*base2[1]+base2[1]*base2[1])/(base2[1]-base1[1]);
         // Alle Winkel zulassen
         //base3[1]  = (rand() % 1000 + 1) /100.0;

         temp = Eigen::Vector3d(base1.x(), base1.y(), 0.0f);
         side1 = (top - temp).norm();
         temp = Eigen::Vector3d(base2.x(), base2.y(), 0.0f);
         side2 = (top - temp).norm();
         temp = Eigen::Vector3d(base3.x(), base3.y(), 0.0f);
         side3 = (top - temp).norm();
         sideBase = (base1 - base2).norm();
         angle12 = acos((-sideBase*sideBase + side1*side1 + side2*side2)/(2*side1*side2));

         sideBase = (base2 - base3).norm();
         angle23 = acos((-sideBase*sideBase + side3*side3 + side2*side2)/(2*side3*side2));

         sideBase = (base3 - base1).norm();
         angle31 = acos((-sideBase*sideBase + side1*side1 + side3*side3)/(2*side1*side3));

         cout << "Testing vectors [" << base1.x() << ", " << base1.y() << "] ";
         cout << "[" << base2.x() << ", " << base2.y() << "] ";
         cout << "[" << base3.x() << ", " << base3.y() << "]" << endl;
         cout << "with top-point [" << top.x() << ", " << top.y() << ", " << top.z() << "]" << endl;
         cout << "Distances " << side1 << ", " << side2 << ", " << side3 << "" << endl;
         cout << "Angles " << angle12 << ", " << angle23 << ", " << angle31 << "" << endl;

         //Call Testfunction
         boost::shared_ptr<Calibration_Object> calibrationObject(new Calibration_Object());
         calibrationObject->top_angle_ab = angle12;
         calibrationObject->top_angle_bc = angle23;
         calibrationObject->top_angle_ca = angle31;
         calibrationObject->side_a = (base1 - base2).norm();
         calibrationObject->side_b = (base2 - base3).norm();
         calibrationObject->side_c = (base1 - base3).norm();

         boost::shared_ptr<FeasibilityChecker> feasibilityChecker(new FeasibilityChecker(calibrationObject, Eigen::Vector3d(0.0,0.0,1.0), 0.15));
         ResectionSolver solver(calibrationObject, feasibilityChecker);
         solver.solve(base1, base2, base3);
         cout << "Results:" << endl;
         success = false;
         if (solver.solutions_a.size() == 0) {cout << "No solutions found." << endl;}
         for (unsigned int i = 0; i < solver.solutions_a.size(); i++)
         {
             if (abs(solver.solutions_b[i] - side1) <= 0.0001f && abs(solver.solutions_c[i] - side2) <= 0.0001f && abs(solver.solutions_a[i] - side3) <= 0.0001f)
             {
                 //system("Color 02");
                 cout << "[" << solver.solutions_b[i] << ", " << solver.solutions_c[i] << ", " << solver.solutions_a[i]<< "]" << endl;
                 success = true;
             }
             else
             {
                 cout << "[" << solver.solutions_b[i] << ", " << solver.solutions_c[i] << ", " << solver.solutions_a[i] << "]" << endl;
                 //system("Color 04");
             }
         }
         if (success) {
             cout << "Distance test successful" << endl;
             success = false;
             for (unsigned int i = 0; i < solver.solutions.size(); i++)
             {
                 if (abs(solver.solutions[i].x() - top.x()) <= 0.0001f && abs(solver.solutions[i].y() - top.y()) <= 0.0001f && abs(solver.solutions[i].z() - top.z()) <= 0.0001f)
                 {
                     //system("Color 02");
                     cout << "[" << solver.solutions[i][0] << ", " <<solver.solutions[i][1] << ", " << solver.solutions[i][2]<< "]" << endl;
                     success = true;
                 }
                 else
                 {
                     cout << "[" << solver.solutions[i][0] << ", " <<solver.solutions[i][1] << ", " << solver.solutions[i][2]<< "]" << endl;
                     //system("Color 04");
                 }
             }
             if (success) {cout << "Top-point test successful!" << endl;}
        }

    return success;
}

bool is_numeric(string string_)
{
    int sizeOfString = string_.size();
    int iteration = 0;
    bool isNumeric = true;

    while(iteration < sizeOfString)
    {
        if(!isdigit(string_[iteration]))
        {
            isNumeric = false;
            break;
        }

        iteration++;
    }
    return isNumeric;
}

bool start_test(string args)
{
    bool testFound = false;
    if (args.substr(0,9) == "resection")
    {
        string numerArg = args.substr(9);
        numerArg.erase(std::remove(numerArg.begin(), numerArg.end(), ' '), numerArg.end());
        srand (time(NULL));
        if (is_numeric(numerArg) && numerArg.size() > 0) {
            unsigned int numbers = atoi(numerArg.c_str());
            unsigned int success = 0;
            for (unsigned int i = 0;  i < numbers; i++)
            {
                if (test_resectionsolver()) {success++;}
                clock_t time_end;
                time_end = clock() + 20 * CLOCKS_PER_SEC/1000;
                while (clock() < time_end)
                {
                }
                cout <<  endl;
            }
            cout << success << "/" << numbers << " were successful." <<  endl;
        }
        else
        {
            int inp;
            do
            {
                test_resectionsolver();
                inp = cin.get();
            }
            while (inp != 32);
        }
        testFound = true;
    }
    else if (args.substr(0,13) == "serialization")
    {
        string numerArg = args.substr(13);
        numerArg.erase(std::remove(numerArg.begin(), numerArg.end(), ' '), numerArg.end());
        srand (time(NULL));
        if (numerArg.size() > 0) {
            std::string writer = numerArg.c_str();
            Abstract_TransformationFile_Manager * manager;
            if (writer == "XML")
            {
                manager = new TransformationFile_Manager_XML(workspacePath + "/test.xml");
            }
            else
            {
                manager = new TransformationFile_Manager_Data(workspacePath + "/test.data");
            }
            if (test_serialisation(manager))
            {
                ROS_INFO("Test was successful.");
            }
            else
            {
                ROS_INFO("Test failed.");
            }
        }
        testFound = true;
    }
    else if (args.substr(0,15) == "markerdetection")
    {
        test_markerDetection();
        testFound = true;
    }
    else if (args.substr(0,12) == "averageframe")
    {
        string filename = args.substr(13);
        filename.erase(std::remove(filename.begin(), filename.end(), ' '), filename.end());
        if (filename.size() > 0) test_averageframe(filename);
        testFound = true;
    }
    else
    {
        cout << "Testcase '" << args << "'not declared!" <<  endl;

    }
    return testFound;
}

int main(int argc, char *argv[])
{
    string inp = "";
    char *arg[0];
    int x = 0;
    ros::init(x, arg, "calibrationTool_tests");
    ros::NodeHandle nh(ros::this_node::getName());
    string myWorkspace;
    nh.getParam("test_workspace", myWorkspace);
    ROS_INFO_STREAM("Using workspace " << myWorkspace);
    workspacePath = myWorkspace;
    std::cout << "Please choose a test to run: " << std::endl;

    std::getline(std::cin, inp);
    string args_ = inp;
    while (args_ != "exit")
    {
        if (!start_test(args_))
        {
            std::cout << "Available tests are: " << std::endl;
            std::cout << " resection <count>" << std::endl;
            std::cout << " serialization XML|DATA" << std::endl;
            std::cout << " markerdetection" << std::endl;
            std::cout << " averageframe <filename>" << std::endl;
        }
        std::cout << "Please choose a test to run: " << std::endl;
        std::getline(std::cin, inp);
        args_ = inp;
    }

    return 0;
}

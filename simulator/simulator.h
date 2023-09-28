#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <string>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Eigen>
#include <time.h>

#include "robot.h"

class Simulator{
    private:
        std::string logFileName;
        std::ofstream logFile;
        Robot robot;
        const double fUpdateRobot;
        Eigen::Vector3d robotSpdBuffer[1000];
        
        std::string createName4Log();

    public:
        Simulator(const double fUpdateRobot);

        ~Simulator();

        /// use this function to run the simulation
        /// for a certain duration and write the results
        /// in a log file
        /// @param duration in seconds
        /// @param logFileName name of the file you want to create it
        void simulateFor(double duration, std::string logFileName);

        /// use this function to run the simulation
        /// for a certain duration. The function will
        /// write a log file whose name will be related
        /// to the current date
        /// @param duration in seconds
        void simulateFor(double duration);




};

#endif
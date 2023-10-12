#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <string>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Eigen>
#include <time.h>
#include <ctime>
#include <vector>

#include "robot.h"
#include "spd_types.h"
#include "../consts/some_consts.h"
#include "cmd_signal.h"


class Simulator{
    private:
        static const int bufferSize = 1000;
        Robot *robot;
        std::vector<Eigen::Vector3d> robotTrueSpd_Buffer;
        std::vector<Eigen::Vector3d> robotReadSpd_Buffer;
        std::vector<Eigen::Vector4d> wheelsTrueSpd_Buffer;
        std::vector<Eigen::Vector4d> wheelsReadSpd_Buffer;
        std::vector<double> timeBuffer;
        std::string timeUnity = "ms";
        double seconds2timeUnity = 1e-3;
        const Eigen::IOFormat tableFormatTAB;

        std::string logFileName;
        static const vecEachSpdType<std::string> suffixes4logFile;
        vecEachSpdType<std::string> fileNames;
        static const vecEachSpdType<std::string> logFileHeaders;


        std::string createName4Log();

        void bufferize(int bufferPos);

        /// @brief flush part of the buffers into each file (for all the spdType's).
        /// @attention this function won't clean the buffers, since it supposes
        /// you are using rotating indexes.
        /// @param printUntil last position in buffer to print: inside [0, bufferSize[
        void flushAllBuffers(int printUntil);

        /// @brief flush the entire buffers into each file (for all the spdType's).
        /// @attention this function won't clean the buffers, since it supposes
        /// you are using rotating indexes.
        void flushAllBuffers();

        /// @brief print a specified buffer into a specified file
        /// @attention this function is an auxiliar function. It is used
        /// to help 'flushAllBuffers'.
        /// @attention this funcion will open and close the file.
        /// @overload used for vector3d
        /// @param file the file you wanna print the buffer.
        /// @param buffer specifies which buffer we are talking about.
        /// @param printUntil last position in buffer to print: inside [0, bufferSize[
        void printBufferOnFile(const std::string &fileName, const std::vector<Eigen::Vector3d> &buffer, int printUntil);
        
        /// @brief print a specified buffer into a specified file
        /// @attention this function is an auxiliar function. It is used
        /// to help 'flushAllBuffers'.
        /// @attention this funcion will open and close the file.
        /// @overload used for vector4d
        /// @param file the file you wanna print the buffer.
        /// @param buffer specifies which buffer we are talking about.
        /// @param printUntil last position in buffer to print: inside [0, bufferSize[
        void printBufferOnFile(const std::string &fileName, const std::vector<Eigen::Vector4d> &buffer, int printUntil);

        /// @brief creates the log files and prints the headers on them 
        void startLogFiles();
    public:
        Simulator();

        ~Simulator();

        /// @brief use this function to run the simulation
        /// for a certain duration and write the results
        /// in a log file
        /// @param duration in seconds
        /// @param logFileName name of the file you want to create it (without extension)
        /// @param cmd4wheels sugou
        void simulateFor(double duration, const std::string logFileName, const CmdSignal &cmdSignal);

        /// @brief use this function to run the simulation
        /// for a certain duration. The function will
        /// write a log file whose name will be related
        /// to the current date
        /// @param duration in seconds
        /// @param cmd4wheels sugou
        void simulateFor(double duration, const CmdSignal &cmdSignal);


        /// @brief sets the unity for the time printed on the log files
        /// @param timeUnity choose one: "s", "ms", "us", "ns", "ps".
        void setTimeUnity(const std::string timeUnity);
};


#endif
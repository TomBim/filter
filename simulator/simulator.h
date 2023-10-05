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

enum class commandSignal{ Constant, Line, Triangular };

class Simulator{
    private:
        static const int bufferSize = 1000;
        std::string logFileName;
        std::ofstream logFile;
        Robot *robot;
        std::vector<Eigen::Vector3d> robotTrueSpd_Buffer;
        std::vector<Eigen::Vector3d> robotReadSpd_Buffer;
        std::vector<Eigen::Vector4d> wheelsTrueSpd_Buffer;
        std::vector<Eigen::Vector4d> wheelsReadSpd_Buffer;
        const Eigen::IOFormat tableFormatTAB;

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
        void simulateFor(double duration, std::string logFileName);

        /// @brief use this function to run the simulation
        /// for a certain duration. The function will
        /// write a log file whose name will be related
        /// to the current date
        /// @param duration in seconds
        void simulateFor(double duration);
};

// @todo make this happen
/*
class CommandType{
    private:
        commandSignal cmdSignal;
        std::vector<double> timeParams;
        std::vector<double> valueParams;
    
    public:
        CommandType(commandSignal cmdSignal, std::vector<double> timeParams, std::vector<double> valueParams);

        double getCommand(double timeNow);
};*/

#endif
#ifndef ROBOT_H
#define ROBOT_H

#include <eigen3/Eigen/Dense>
#include "controller.h"
#include "some_consts.h"

class Robot{
    public:
        static constexpr double P = 0.6;
        static constexpr double I = 0.1;
        static constexpr double angSpdCmd2Voltage_oneWheel = 0.5;
        static constexpr double fs = 1e3;
        static constexpr double fUpdateRobot = 1e6;
        static constexpr int nSensorsPerWheel = 40000;


        static const Eigen::Matrix4d angSpdCmd2Voltage;
        static const int nStepsBetweenUsingController;
        static const double motorDivSize;
    
    private:
        Eigen::Vector3d robotSpd;
        Eigen::Vector3d robotSpdCmd;
        Eigen::Vector4d wheelsAngSpd;       // rad/s
        Eigen::Vector4d wheelsAngSpdCmd;    // rad/s
        double noise;
        Controller* controller;
        Eigen::Vector4d motorPosition;      // rad
        Eigen::Vector4d lastVoltageCmd;     
        int stepsTilControl;

    public:
        Robot(double stdDevNoise);

        ~Robot();

        Eigen::Vector4d readSensors();

        void applyController();

        void updateRobotStatus();
};

#endif
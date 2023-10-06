#ifndef ROBOT_H
#define ROBOT_H

#include <cmath>
#include <eigen3/Eigen/Dense>
#include "controller.h"
#include "../consts/some_consts.h"
#include "../main.h"

class Robot{

    private:
        Eigen::Vector3d robotSpd;
        Eigen::Vector3d robotSpdCmd;
        Eigen::Vector4d wheelsAngSpd;       // rad/s
        Eigen::Vector4d wheelsAngSpdCmd;    // rad/s
        Eigen::Vector4d lastEncodersRead;
        double noise;
        Controller* controller;
        Eigen::Vector4d motorPosition;      // rad
        Eigen::Vector4d lastVoltageCmd;

        Eigen::Vector4d registerEncodersRead();

    public:
        Robot();

        Robot(double stdDevNoise);

        ~Robot();

        void resetRobot();

        Eigen::Vector4d getLastEncodersRead() const;

        Eigen::Vector4d getWheelsTrueSpd() const;

        Eigen::Vector3d estimateRobotsSpd() const;

        Eigen::Vector3d getRobotsTrueSpd() const;

        void applyController();

        void updateRobotStatus(const Eigen::Vector4d &wheelsAngSpdCmd);

        void updateRobotStatus(const Eigen::Vector3d &robotSpdCmd);
};

#endif
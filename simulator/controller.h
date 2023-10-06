#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <eigen3/Eigen/Eigen>
#include "robot.h"

class Controller{
    private:
        Eigen::Vector4d errorSum;
        Eigen::Vector4d lastError;
        bool hasP, hasI, hasD;

    public:
        Controller();

        ~Controller();

        Eigen::Vector4d applyController(const Eigen::Vector4d &wheelsAngSpeed, const Eigen::Vector4d &wheelsAngSpeedCommand);

        void resetController();
};

#endif
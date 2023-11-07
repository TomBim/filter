#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <eigen3/Eigen/Core>

#include <iostream>

#include "../consts/some_consts.h"
#include "some_functions.h"

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
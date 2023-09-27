#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <eigen3/Eigen/Eigen>
#include "robot.h"

class Controller{
    private:
        double P = 1;
        double I = 1;
        Eigen::Vector4d errorSum;

    public:
        Controller(double P, double I);

        ~Controller();

        Eigen::Vector4d applyController(Eigen::Vector4d wheelsAngSpeed, Eigen::Vector4d wheelsAngSpeedCommand);

        void resetController();        
};

#endif
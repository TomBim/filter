#include "robot.h"

Robot::Robot(double stdDevNoise) {
    robotSpeed.setZero();
    robotSpeedCommand.setZero();
    wheelsSpeed.setZero();
    wheelsSpeedCommand.setZero();
    noise = stdDevNoise;
}

Robot::~Robot() {};

Eigen::Vector4d Robot::readSensors() const{
    return ;
}
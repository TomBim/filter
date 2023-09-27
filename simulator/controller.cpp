#include "controller.h"

Controller::Controller(double P, double I) : P(P), I(I) {
        errorSum.setZero();
}

Controller::~Controller() {}

Eigen::Vector4d Controller::applyController(Eigen::Vector4d wheelsAngSpeed, Eigen::Vector4d wheelsAngSpeedCommand) {    
    Eigen::Vector4d error = wheelsAngSpeedCommand - wheelsAngSpeed;
    Eigen::Vector4d outVoltage;
    errorSum += error;
    outVoltage = (Robot::angSpdCmd2Voltage * wheelsAngSpeedCommand) + (P * error) + (I * errorSum);
    return outVoltage;
}

void Controller::resetController() {
    errorSum.setZero();
}
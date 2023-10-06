#include "controller.h"

Controller::Controller() {
    errorSum.setZero();
    lastError.setZero();
    hasP = (CONTROL_P != 0);
    hasI = (CONTROL_I != 0);
    hasD = (CONTROL_D != 0);
}

Controller::~Controller() {}

Eigen::Vector4d Controller::applyController(const Eigen::Vector4d &wheelsAngSpd, const Eigen::Vector4d &wheelsAngSpdCmd) {
    Eigen::Vector4d error = wheelsAngSpdCmd - wheelsAngSpd;
    Eigen::Vector4d outVoltage;

    outVoltage = Binv* wheelsAngSpdCmd;

    if(hasP)
        outVoltage += CONTROL_P * error;
    if(hasI) {
        errorSum += error;
        outVoltage += CONTROL_I * errorSum;
    }
    if(hasD) {
        outVoltage += CONTROL_D * (error - lastError);
        lastError = error;
    }

    return outVoltage;
}

void Controller::resetController() {
    errorSum.setZero();
    lastError.setZero();
}
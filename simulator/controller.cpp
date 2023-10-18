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

    #ifdef DBG_MODE
    std::cout << "cmd = " << wheelsAngSpdCmd.transpose() << "\n\n";
    std::cout << "read = " << wheelsAngSpd.transpose() << "\n\n";
    std::cout << "error =  " << error.transpose() << "\n\n";
    #endif

    outVoltage.setZero();
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

    double voltage_i;
    for(int i = 0; i < outVoltage.size(); i++) {
        voltage_i = outVoltage[i];
        if(abs(voltage_i) > MAX_VOLTAGE)
            if(voltage_i > 0)
                 outVoltage[i] =  MAX_VOLTAGE;
            else outVoltage[i] = -MAX_VOLTAGE;
    }

    return outVoltage;
}

void Controller::resetController() {
    errorSum.setZero();
    lastError.setZero();
}
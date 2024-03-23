#include "perf_measurers.h"

PerformanceMeasurers::PerformanceMeasurers() {
    reset();
}

double PerformanceMeasurers::getTotalSpentTime() const {
    return totalSpentTime;
}

void PerformanceMeasurers::setTotalSpentTime(double newValue) {
    totalSpentTime = newValue;
}

Eigen::Vector4d PerformanceMeasurers::getMSE_wheels() const {
    return MSEwheels_sum / MSEwheels_n;
}

Eigen::Vector3d PerformanceMeasurers::getMSE_robot() const {
    return MSErobot_sum / MSErobot_n;
}

void PerformanceMeasurers::putWheelsEstim(const Eigen::Vector4d &estimation, 
        const Eigen::Vector4d &trueValue) {
    Eigen::Vector4d error = estimation - trueValue;
    MSEwheels_sum += error.cwiseProduct(error); // error squared
    MSEwheels_n++;
}

void PerformanceMeasurers::putRobotEstim(const Eigen::Vector3d &estimation,
        const Eigen::Vector3d &trueValue) {
    Eigen::Vector3d error = estimation - trueValue;
    MSErobot_sum += error.cwiseProduct(error); // error squared
    MSErobot_n++;
}

void PerformanceMeasurers::reset() {
    totalSpentTime = 0;
    MSEwheels_sum.setZero();
    MSEwheels_n = 0;
    MSErobot_sum.setZero();
    MSErobot_n = 0;
}
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
    MSEwheels_sum += (estimation - trueValue).cwisePow(2);
    MSEwheels_n++;
}

void PerformanceMeasurers::putRobotEstim(const Eigen::Vector3d &estimation,
        const Eigen::Vector3d &trueValue) {
    MSErobot_sum += (estimation - trueValue).cwisePow(2);
    MSErobot_n++;
}

void PerformanceMeasurers::reset() {
    totalSpentTime = 0;
    MSEwheels_sum.setZero();
    MSEwheels_n = 0;
    MSErobot_sum.setZero();
    MSErobot_n = 0;
}
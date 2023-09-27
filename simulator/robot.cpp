#include "robot.h"

Robot::Robot(double stdDevNoise) {
    robotSpd.setZero();
    robotSpdCmd.setZero();
    wheelsAngSpd.setZero();
    wheelsAngSpdCmd.setZero();
    noise = stdDevNoise;
    controller = new Controller(P, I);
    motorPosition.setZero();
    lastVoltageCmd.setZero();
    stepsTilControl = 0;
}

Robot::~Robot() {};

Eigen::Vector4d Robot::readSensors() {
    Eigen::Vector4i counter;
    counter = motorPosition / motorDivSize;
    Eigen::Vector4d deltaTheta;
    deltaTheta = ((Eigen::Vector4d) counter) * motorDivSize;
    motorPosition -= deltaTheta;    // don't need to think if it's negative or positive
    return deltaTheta * fs;
}

void Robot::applyController() {
    lastVoltageCmd = controller->applyController(readSensors(), wheelsAngSpdCmd);
}


void Robot::updateRobotStatus() {
    /// TODO: need to use the constraints here
    /// TODO: this 'decrease' is really necessary?
    if(stepsTilControl == 0) {
        applyController();
        stepsTilControl = nStepsBetweenUsingController; // we will decrease one after
    }
    Eigen::Vector4d wheelsAngAcceleration = angSpdCmd2Voltage.inverse() * lastVoltageCmd;
    motorPosition = motorPosition + wheelsAngSpd / fUpdateRobot + 
        wheelsAngAcceleration / fUpdateRobot / fUpdateRobot / 2;
    wheelsAngSpd += wheelsAngAcceleration / fUpdateRobot;

    stepsTilControl--;       
}

const Eigen::Matrix4d Robot::angSpdCmd2Voltage = Eigen::Matrix4d::Identity() * Robot::angSpdCmd2Voltage_oneWheel;

const int Robot::nStepsBetweenUsingController = int(fUpdateRobot / fs);

const double Robot::motorDivSize = 2 * PI / Robot::nSensorsPerWheel;
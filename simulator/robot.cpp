#include "robot.h"

Robot::Robot(double stdDevNoise) {
    controller = new Controller();
    noise = stdDevNoise;
    resetRobot();
}

Robot::Robot() {
    controller = new Controller();
    noise = 0;
    resetRobot();
}

Robot::~Robot() {}

void Robot::resetRobot() {
    // set velocities = 0
    robotSpd.setZero();
    robotSpdCmd.setZero();
    wheelsAngSpd.setZero();
    wheelsAngSpdCmd.setZero();

    // reset motor's position
    motorPosition.setZero();

    // reset controller
    controller->resetController();
    
    // reset voltage cmd
    lastVoltageCmd.setZero();   // @todo is this rly necessary?
}

Eigen::Vector4d Robot::getLastEncodersRead() const {
    return lastEncodersRead;
};

Eigen::Vector4d Robot::getWheelsTrueSpd() const {
    return wheelsAngSpd;
}

Eigen::Vector3d Robot::estimateRobotsSpd() const {
    return rMplus * lastEncodersRead;
}

Eigen::Vector3d Robot::getRobotsTrueSpd() const {
    return rMplus * wheelsAngSpd;
}

Eigen::Vector4d Robot::registerEncodersRead() {
    Eigen::Vector4i counts;
    counts = motorPosition * rad2counts;
    Eigen::Vector4d deltaThetaCounted;
    deltaThetaCounted = ((Eigen::Vector4d) counts) * counts2rad;
    motorPosition -= deltaThetaCounted;    // don't need to think if it's negative or positive
    lastEncodersRead =  deltaThetaCounted * Fs;
    return lastEncodersRead;
}

void Robot::applyController() {
    lastVoltageCmd = controller->applyController(getLastEncodersRead(), wheelsAngSpdCmd);
}


void Robot::updateRobotStatus(const Eigen::Vector4d &wheelsAngSpdCmd) {
    // first, let's apply the controller
    this->wheelsAngSpdCmd = wheelsAngSpdCmd;
    applyController();

    /************************************************************************************
    * OBS:                                                                              *
    *   vw(t) = exp[F.(t-t0)].vw0 + lambda ->                                           *
    *       -> thetaw(t) = inv(F).{exp[F.(t-t0)] - I}.vw0 + lambda.(t-t0) + thetaw(t0)  *
    * THEREFORE:                                                                        *
    *   thetaw(n) = inv(F).{exp[F.T] - I}.vw(n-1) + lambda.T + thetaw(n-1)              *
    * NOTE:                                                                             *
    *   thetaw(n) = inv(F).(A - I).vw(n-1) + B.T.u + thetaw(n-1)                        *
    ************************************************************************************/
    // we cant consider noise for now because the equations would be very different
    // double noiseObs = STD_DEV_NOISE * gaussian_distr(gen);
    // wheelsAngSpd = A * wheelsAngSpd + B * lastVoltageCmd + noiseObs;

    // with the command voltage from controller, let's update wheels' velocities
    motorPosition += F.inverse() * (A - Eigen::Matrix4d::Identity()) * wheelsAngSpd +
        B * Ts * lastVoltageCmd;
    wheelsAngSpd += A * wheelsAngSpd + B * lastVoltageCmd;

    // with the wheels' velocities, we can calculate the robot's velocities
    robotSpd = rMplus * wheelsAngSpd;

    // to finish, we need to register the encoders' reads and update 'lastEncodersRead'
    registerEncodersRead();
}

void Robot::updateRobotStatus(const Eigen::Vector3d &robotSpdCmd) {
    this->robotSpdCmd = robotSpdCmd;
    Eigen::Vector4d wheelsCmd = M_over_r * robotSpdCmd;
    updateRobotStatus(wheelsCmd);
}
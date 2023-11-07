#include "robot.h"

Robot::Robot(const std::string &filterType, double stdDevNoise) {
    filter = nullptr;
    hasFilter = false;
    if(filterType == "kalman") {
        filter = (GaussianFilter*) new KalmanFilter();
        hasFilter = true;
    }
    else if(filterType == "info") {
        filter = (GaussianFilter*) new InfoFilter();
        hasFilter = true;
    }
    controller = new Controller();
    noise = stdDevNoise;
    resetRobot();
}

Robot::Robot(const std::string &filterType) {
    this->filter = nullptr;
    hasFilter = false;
    if(filterType == "kalman") {
        this->filter = (GaussianFilter*) new KalmanFilter();
        hasFilter = true;
    }
    else if(filterType == "info") {
        this->filter = (GaussianFilter*) new InfoFilter();
        hasFilter = true;
    }
    controller = new Controller();
    noise = 0;
    resetRobot();
}

Robot::Robot() {
    controller = new Controller();
    filter = nullptr;
    hasFilter = false,
    noise = 0;
    resetRobot();
}

Robot::Robot(double stdDevNoise) {
    filter = nullptr;
    hasFilter = false;
    controller = new Controller();
    noise = stdDevNoise;
    resetRobot();
}

Robot::~Robot() {
    delete filter;
    delete controller;
}

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

    // reset filter
    lastEncodersRead.setZero();
    if(hasFilter) {
        filter->resetFilter();
        estimatedWheelsAngSpd.setZero();
    }
    
    // reset voltage cmd
    lastVoltageCmd.setZero();   // @todo is this rly necessary?
}

Eigen::Vector4d Robot::getEstimatedWheelsSpd() const {
    return estimatedWheelsAngSpd;
};

Eigen::Vector4d Robot::getWheelsTrueSpd() const {
    return wheelsAngSpd;
}

Eigen::Vector3d Robot::estimateRobotsSpd() const {
    return rMplus * estimatedWheelsAngSpd;
}

Eigen::Vector3d Robot::getRobotsTrueSpd() const {
    return rMplus * wheelsAngSpd;
}

Eigen::Vector4d Robot::registerEncodersRead() {
    Eigen::Vector4i  counts;
    counts = (motorPosition * rad2counts).cast<int>();
    Eigen::Vector4d deltaThetaCounted;
    deltaThetaCounted = counts.cast<double>() * counts2rad;
    #ifdef DBG_MODE
    debugBuffer << "motor pos b4 = " << motorPosition.transpose() << "\n\n";
    debugBuffer << "counts = " << counts.transpose() << "\n\n";
    debugBuffer << "dThetaCounted = " << deltaThetaCounted.transpose() << "\n\n";
    #endif
    motorPosition -= deltaThetaCounted;    // don't need to think if it's negative or positive
    #ifdef DBG_MODE
    debugBuffer << "motor pos after = " << motorPosition.transpose() << "\n\n";
    #endif
    lastEncodersRead =  deltaThetaCounted * Fs;
    return lastEncodersRead;
}

void Robot::applyController() {
    lastVoltageCmd = controller->applyController(estimatedWheelsAngSpd,
                                                 wheelsAngSpdCmd);
}

void Robot::applyFilter() {
    if(hasFilter)
        estimatedWheelsAngSpd = filter->applyFilter(lastVoltageCmd, lastEncodersRead);
    else
        estimatedWheelsAngSpd = lastEncodersRead / N;
    
    #ifdef DBG_MODE
    if( !hasFilter )
        debugBuffer << "NO FILTER\n\n";
    #endif
}

void Robot::updateRobotStatus(const Eigen::Vector4d &wheelsAngSpdCmd) {
    // first, let's apply the controller
    this->wheelsAngSpdCmd = wheelsAngSpdCmd;
    #ifdef DBG_MODE
    debugBuffer << "## CONTROLLER \n\n";
    #endif
    applyController();

    /************************************************************************************
    * OBS:                                                                              *
    *   vw(t) = exp[F.(t-t0)].vw0 + lambda ->                                           *
    *       -> thetaw(t) = inv(F).{exp[F.(t-t0)] - I}.vw0 + lambda.(t-t0) + thetaw(t0)  *
    * THEREFORE:                                                                        *
    *   thetaw(n) = inv(F).{exp[F.T] - I}.vw(n-1) + lambda.T + thetaw(n-1)              *
    * BUT: theta motor                                                                  *
    *   thetam(n) = thetaw(n) * REDUCTION_FACTOR                                        *
    * NOTE:                                                                             *
    *   thetam(n) = REDUCTION_FACTOR.(inv(F).(A - I).vw(n-1) + B.T.u) + thetam(n-1)     *
    ************************************************************************************/
    // we cant consider noise for now because the equations would be very different
    // double noiseObs = STD_DEV_NOISE * gaussian_distr(gen);
    // wheelsAngSpd = A * wheelsAngSpd + B * lastVoltageCmd + noiseObs;

    #ifdef DBG_MODE
    debugBuffer << "## CINEMATIC UPDATE \n\n";
    debugBuffer << "motor position b4 = " << motorPosition.transpose() << "\n\n";
    #endif

    // with the command voltage from controller, let's update wheels' velocities
    motorPosition += N * (F.inverse() * (A - Eigen::Matrix4d::Identity()) * wheelsAngSpd +
        B * Ts * lastVoltageCmd);
    wheelsAngSpd = A * wheelsAngSpd + B * lastVoltageCmd;
    #ifdef DBG_MODE
    debugBuffer << "motor position after = " << motorPosition.transpose() << "\n\n";
    debugBuffer << "wheels omega = " << wheelsAngSpd.transpose() << "\n\n";
    debugBuffer << "last voltage cmd = " << lastVoltageCmd.transpose() << "\n\n";
    #endif

    // with the wheels' velocities, we can calculate the robot's velocities
    robotSpd = rMplus * wheelsAngSpd;

    // to finish, we need to register the encoders' reads, update 'lastEncodersRead'
    // and apply the filter
    #ifdef DBG_MODE
    debugBuffer << "## ENCODER \n\n";
    #endif
    registerEncodersRead();
    #ifdef DBG_MODE
    debugBuffer << "## FILTER \n\n";
    #endif
    applyFilter();
}

void Robot::updateRobotStatus(const Eigen::Vector3d &robotSpdCmd) {
    this->robotSpdCmd = robotSpdCmd;
    Eigen::Vector4d wheelsCmd = M_over_r * robotSpdCmd;
    updateRobotStatus(wheelsCmd);
}
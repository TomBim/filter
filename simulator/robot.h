#ifndef ROBOT_H
#define ROBOT_H

#include <cmath>
#include <eigen3/Eigen/Core>
#include <random>
#include <ctime>
#include <stdio.h>
#include <iostream>

#include "../consts/some_consts.h"
#include "some_functions.h"
#include "controller.h"
#include "../filter/filter.h"

extern std::mt19937_64 gen;
extern std::normal_distribution<double> gaussian_distr;

class Robot{
    private:
        Eigen::Vector3d robotSpd;
        Eigen::Vector3d robotSpdCmd;
        Eigen::Vector4d wheelsAngSpd;       // rad/s
        Eigen::Vector4d wheelsAngSpdCmd;    // rad/s
        double noise = 0;

        GaussianFilter *filter = nullptr;
        bool hasFilter = false;
        Controller* controller = nullptr;

        Eigen::Vector4d lastEncodersRead;
        Eigen::Vector4d estimatedWheelsAngSpd;  // if has filter, its the filtered speed. 
                                                // If doesn't have, the encoders read speed
        Eigen::Vector4d motorPosition;      // rad
        Eigen::Vector4d lastVoltageCmd;


        Eigen::Vector4d registerEncodersRead();
    public:
        /// @brief default for a robot. It creates a robot without a filter.
        Robot();

        /// @brief constructor for a robot without filter, but with noise
        /// on prediction
        /// @attention the Robot class is not using noise on prediction, so
        /// passing this parameter is kind of useless.
        /// @param stdDevNoise std deviation for the noise on prediction
        Robot(double stdDevNoise);

        /// @brief constructor for a robot with filter
        /// @note the filter types available are only Kalman ("Kalman")
        /// and Information Filter ("Info").
        /// @param filterType filter types: "Kalman" or "Info"
        Robot(const std::string &filterType);


        /// @brief constructor for a robot with filter and noise on prediction
        /// @note the filter types available are only Kalman ("kalman")
        /// and Information Filter ("info").
        /// @attention the Robot class is not using noise on prediction, so
        /// passing this parameter is kind of useless.
        /// @param filterType filter types: "kalman" or "info"
        /// @param stdDevNoise std deviation for the noise on prediction
        Robot(const std::string &filterType, double stdDevNoise);

        ~Robot();

        void resetRobot();

        Eigen::Vector4d getEncodersRead() const;

        Eigen::Vector4d getEstimatedWheelsSpd() const;

        Eigen::Vector4d getWheelsTrueSpd() const;

        Eigen::Vector3d estimateRobotsSpd() const;

        Eigen::Vector3d getRobotsTrueSpd() const;

        void applyController();

        void applyFilter();

        void updateRobotStatus(const Eigen::Vector4d &wheelsAngSpdCmd);

        void updateRobotStatus(const Eigen::Vector3d &robotSpdCmd);

        /// @return the total spent time for filtering (in seconds)
        double getTotalSpentTimeFiltering() const;
};

#endif
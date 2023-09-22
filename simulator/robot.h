#ifndef ROBOT_H
#define ROBOT_H

#include <eigen3/Eigen/Eigen>

class Robot{
    static const double SENSOR_DISCRETIZATION = 1e-3;
    private:
        Eigen::Vector2d robotSpeed;
        Eigen::Vector2d robotSpeedCommand;
        Eigen::Vector4d wheelsSpeed;
        Eigen::Vector4d wheelsSpeedCommand;
        double noise;

    public:
        Robot(double stdDevNoise);

        ~Robot();

        Eigen::Vector4d readSensors() const;

        void applyController();
};

#endif
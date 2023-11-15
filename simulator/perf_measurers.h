#include <eigen3/Eigen/Core>

class PerformanceMeasurers{
    private:
        double totalSpentTime; // in seconds
        Eigen::Vector4d MSEwheels_sum;
        Eigen::Vector3d MSErobot_sum;
        int MSEwheels_n, MSErobot_n;

    public:
        PerformanceMeasurers();

        double getTotalSpentTime() const;

        void setTotalSpentTime(double newValue);

        Eigen::Vector4d getMSE_wheels() const;

        Eigen::Vector3d getMSE_robot() const;

        void putWheelsEstim(const Eigen::Vector4d &estimation, const Eigen::Vector4d &trueValue);

        void putRobotEstim(const Eigen::Vector3d &estimation, const Eigen::Vector3d &trueValue);

        void reset();
};
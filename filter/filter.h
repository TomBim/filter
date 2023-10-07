#ifndef FILTER_H
#define FILTER_H

#define FILTER_V_1_0

#include "../consts/some_consts.h"

#ifdef FILTER_V_1_0

#include "eigen/Eigen/Eigen"

class GaussianFilter {
    protected:
        virtual void estimate(const Eigen::Vector4d &cmd) = 0;

        virtual void filtrate(const Eigen::Vector4d &obs) = 0;
    public:
        GaussianFilter();

        ~GaussianFilter();

        virtual Eigen::Vector4d applyFilter(const Eigen::Vector4d &voltageCmd, 
            const Eigen::Vector4d &encodersRead) = 0;

        virtual Eigen::Vector4d getLastFilteredSpd() const = 0;

        virtual void resetFilter() = 0;
};

class KalmanFilter : GaussianFilter {
    private:
        Eigen::Vector4d mu;
        Eigen::Matrix4d Sigma;
        Eigen::Matrix4d KalmanGain;

        void estimate(const Eigen::Vector4d &cmd);

        void filtrate(const Eigen::Vector4d &obs);
    public:
        KalmanFilter();

        ~KalmanFilter();

        Eigen::Vector4d applyFilter(const Eigen::Vector4d &voltageCmd, 
            const Eigen::Vector4d &encodersRead);

        Eigen::Vector4d getLastFilteredSpd() const;

        void resetFilter();
};

class InfoFilter : GaussianFilter {
    private:
        Eigen::Vector4d infoVec;
        Eigen::Matrix4d infoMatrix;
        Eigen::Matrix4d infoMatrix_inv;
        Eigen::Vector4d mu;

        void estimate(const Eigen::Vector4d &cmd);

        void filtrate(const Eigen::Vector4d &obs);
    public:
        InfoFilter();

        ~InfoFilter();

        Eigen::Vector4d applyFilter(const Eigen::Vector4d &voltageCmd,
            const Eigen::Vector4d &encodersRead);

        Eigen::Vector4d getLastFilteredSpd() const;

        void resetFilter();
};


#endif

#endif
#ifndef FILTER_H
#define FILTER_H

#define FILTER_V_2_0

#include "eigen3/Eigen/Core"
#include <array>
#include <time.h>
#include <iostream>

#include "../consts/some_consts.h"
#include "../simulator/some_functions.h"

#ifdef FILTER_V_1_0


class GaussianFilter {
    protected:
        const std::string type;

        virtual void predict(const Eigen::Vector4d &cmd) = 0;

        virtual void filtrate(const Eigen::Vector4d &obs) = 0;
    public:
        GaussianFilter(std::string type) : type(type) {}

        virtual ~GaussianFilter() = 0;

        virtual Eigen::Vector4d applyFilter(const Eigen::Vector4d &voltageCmd, 
            const Eigen::Vector4d &encodersRead) = 0;

        virtual Eigen::Vector4d getLastFilteredSpd() const = 0;

        virtual void resetFilter() = 0;

        std::string getFilterType() { return type; }
};

class KalmanFilter : GaussianFilter {
    private:
        Eigen::Vector4d mu;
        Eigen::Matrix4d Sigma;
        Eigen::Matrix4d KalmanGain;

        void predict(const Eigen::Vector4d &cmd);

        void filtrate(const Eigen::Vector4d &obs);
    public:
        KalmanFilter();

        ~KalmanFilter() {}

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

        void predict(const Eigen::Vector4d &cmd);

        void filtrate(const Eigen::Vector4d &obs);
    public:
        InfoFilter();

        ~InfoFilter() {}

        Eigen::Vector4d applyFilter(const Eigen::Vector4d &voltageCmd,
            const Eigen::Vector4d &encodersRead);

        Eigen::Vector4d getLastFilteredSpd() const;

        void resetFilter();
};


#endif


#ifdef FILTER_V_2_0

// using myVector4d = std::array<double, 4>;
// using myMatrix4d = std::array<std::array<double, 4>, 4>;

class GaussianFilter {
    protected:
        static constexpr double A11 = myArray::A[0][0];
        static constexpr double A12 = myArray::A[0][1];
        static constexpr double A13 = myArray::A[0][2];
        static constexpr double A14 = myArray::A[0][3];
        static constexpr double A22 = myArray::A[1][1];
        static constexpr double A23 = myArray::A[1][2];

        static constexpr double Bii = myArray::B[0][0];
        static constexpr double Bij = 0;

        static constexpr double Cii = myArray::C[0][0];
        static constexpr double Cij = 0;

        static constexpr double Rii = myArray::R[0][0];
        static constexpr double Qii = myArray::Q[0][0];
        static constexpr double Qinv_ii = 1 / Qii;
        static constexpr double CQinv_ii = Cii * Qinv_ii;
        static constexpr double CQinvC_ii = Cii * CQinv_ii;

        const std::string type;

        myVector4d mu;
        clock_t totalSpentTime, dt; // in clocks
        

        /// @param mtx must be bisymmetric
        static void AtimesMTX(myMatrix4d &mtx, myMatrix4d &res, bool only2rows);

        static inline void A_MTX_At_plus_R_aux(myMatrix4d &mtx, myMatrix4d &res, 
                                        myMatrix4d &A_MTX, bool exportAM);

        /// @note if you want to put the result directly into mtx,
        /// you can pass res as mtx (res = mtx)
        /// @param mtx must be bisymmetric
        static void A_MTX_At_plus_R(myMatrix4d &mtx, myMatrix4d &res);

        static void A_MTX_At_plus_R(myMatrix4d &mtx, myMatrix4d &res, myMatrix4d A_MTX);

        static void invertBisymmetricMatrix(myMatrix4d &mtx, myMatrix4d &res);

        static void multBisymmetricMatrixes(myMatrix4d &mtxA, const myMatrix4d &mtxB, myMatrix4d &res);

        virtual inline void predict(const myVector4d &cmd) = 0;

        virtual inline void filtrate(const myVector4d &obs) = 0;
    public:
        /// @brief 
        /// @param type "KF" or "IF"
        GaussianFilter(std::string type);

        virtual ~GaussianFilter() = 0;

        virtual Eigen::Vector4d applyFilter(const Eigen::Vector4d &voltageCmd, 
            const Eigen::Vector4d &encodersRead);

        Eigen::Vector4d getLastFilteredSpd() const;

        virtual void resetFilter() = 0;

        std::string getFilterType() { return type; }

        /// @return total spent time in seconds
        double getTotalSpentTime() const;
};

class KalmanFilter : GaussianFilter {
    private:
        myMatrix4d Sigma, KalmanGain, auxMatrix;

        void updateKalmanGain();

        inline void predict(const myVector4d &cmd) override;

        inline void filtrate(const myVector4d &obs) override;
    public:
        KalmanFilter();

        ~KalmanFilter() = default;

        void resetFilter() override;
};

class InfoFilter : GaussianFilter {
    private:
        myVector4d infoVec;
        myMatrix4d infoMatrix;
        myMatrix4d posterioriInfoMatrix_inv;
        myMatrix4d prioriInfoMatrix_inv;
        myMatrix4d auxMatrix;
        bool isInfoMatrixZero;

        inline void predict(const myVector4d &cmd) override;

        inline void filtrate(const myVector4d &obs) override;
    public:
        InfoFilter();

        ~InfoFilter() = default;

        void resetFilter() override;
};




#endif




#endif
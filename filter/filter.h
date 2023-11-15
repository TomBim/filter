#ifndef FILTER_H
#define FILTER_H

#ifndef FILTER_V_MAJOR
    #define FILTER_V_1_0
#else
    #if FILTER_V_MAJOR == 1 && FILTER_V_MINOR == 0
        #define FILTER_V_1_0
    #elif FILTER_V_MAJOR == 2 && FILTER_V_MINOR == 0
        #define FILTER_V_2_0
    #endif
#endif


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
        clock_t dt, totalSpentTime; // in clocks
        Eigen::Vector4d mu;

        virtual void predict(const Eigen::Vector4d &cmd) = 0;

        virtual void filtrate(const Eigen::Vector4d &obs) = 0;
    public:
        GaussianFilter(std::string type);

        virtual ~GaussianFilter() = 0;

        virtual Eigen::Vector4d applyFilter(const Eigen::Vector4d &voltageCmd, 
            const Eigen::Vector4d &encodersRead);

        virtual Eigen::Vector4d getLastFilteredSpd() const = 0;

        virtual void resetFilter() = 0;

        /// @return filter type (if nothing changed: "KF" or "IF")
        std::string getFilterType() const { return type; }
        
        /// @return total spent time (in seconds)
        double getTotalSpentTime() const;
};

class KalmanFilter : GaussianFilter {
    private:
        Eigen::Matrix4d Sigma;
        Eigen::Matrix4d KalmanGain;

        void predict(const Eigen::Vector4d &cmd) override;

        void filtrate(const Eigen::Vector4d &obs) override;
    public:
        KalmanFilter();

        ~KalmanFilter() {}

        Eigen::Vector4d getLastFilteredSpd() const override;

        void resetFilter() override;
};

class InfoFilter : GaussianFilter {
    private:
        Eigen::Vector4d infoVec;
        Eigen::Matrix4d infoMatrix;
        Eigen::Matrix4d infoMatrix_inv;

        void predict(const Eigen::Vector4d &cmd) override;

        void filtrate(const Eigen::Vector4d &obs) override;
    public:
        InfoFilter();

        ~InfoFilter() {}

        Eigen::Vector4d getLastFilteredSpd() const override;

        void resetFilter() override;
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
        
        static constexpr double B11 = myArray::B[0][0];
        static constexpr double B12 = myArray::B[0][1];
        static constexpr double B13 = myArray::B[0][2];
        static constexpr double B14 = myArray::B[0][3];
        static constexpr double B22 = myArray::B[1][1];
        static constexpr double B23 = myArray::B[1][2];

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

        static void A_MTX_At_plus_R(myMatrix4d &mtx, myMatrix4d &res, myMatrix4d &A_MTX);

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
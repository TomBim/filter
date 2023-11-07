#ifndef SOME_CONSTS_H
#define SOME_CONSTS_H

#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <array>
#include <string>
#include <iostream>

#define DBG_MODE

const double PI = 3.1415926536;
const double deg2rad = PI / 180.0;
const double MY_INFINITE = 1.0e9;

#ifdef DBG_MODE
const std::string LOG_MD_PATH = "/mnt/c/Users/tombim/Onedrive/Bureau/log.md";
#endif


using myVector4d = std::array<double, 4>;
using myMatrix4d = std::array<std::array<double, 4>, 4>;

/* @todo fix the values */


/***********************
*   SIMULATION: no     *
*   ROBOT:      ok     *
*   WHEELS:     wheel_radius     *
*   MOTOR:      +-     *
*   REDUCTION:  ok     *
*   CONTROLLER: ok     *
*   MATRIXES:   no     *
***********************/

/***** SIMULATION'S PROPERTIES *****/
const double Fs = 1000;      // sample rate (Hz)
const double Ts = 1.0 / Fs;    // sample period (s)


/***** ROBOT'S PROPERTIES *****/
inline const double ROBOT_RADIUS = 0.0855;      // robot's radius (m)
inline const double ALPHA_FRONT = 60.0 * deg2rad;  // angle between front wheels and the robot's front (WÔF) (rad)
inline const double ALPHA_REAR = 132.5 * deg2rad;  // angle between rear wheels and the robot's front (WÔF) (rad)
inline const double ROBOT_MASS = 2.4;        // robot's mass (kg)
inline const double Icom = 0.03;      // moment of inertia related to the center of mass (SI)
const double STD_DEV_NOISE_MOV = 0.1;    // standard deviation of the noise on robot's movement


/***** WHEELS' PROPERTIES *****/
inline const double WHEEL_RADIUS = 0.02528;       // @attention NOT SURE wheels' radius (m)
inline const double Jw = 1.6396e-5;         // @atention I CALCULATED THIS wheels' inertia (SI)
const double Bw = 4.6753e-8;         // wheels' friction coeff (SI)

/***** MOTOR'S PROPERTIES *****/
inline const double K = 25.5e-3;          // torque constant (SI)
inline const double RES_MOTOR = 1.2;         // motor's resistence (ohm)
inline const double Jm = 92.5e-3 * 1e-4;         // motor's inertia (SI)
inline const double Bm = 8.1224e-6;         // motor's friction coeff (SI)
inline const int SERVO_PPR = 360;          // pulses per revolution
inline const int SERVO_CPR = SERVO_PPR * 4;       // counts per revolution
inline const double counts2rad = 2.0 * PI / SERVO_CPR;    // rad per encoder's count
inline const double rad2counts = 1.0 / counts2rad;        // encoder's counts per rad
const double STD_DEV_NOISE_SENSOR = 1.0 * counts2rad / Ts;   // standard deviation of the noise on the sensor

/***** REDUCTION'S PROPERTIES *****/
const double N = 3.0;         // reduction factor
const double eta = 0.94;      // reduction's efficiency

/***** CONTROLLER'S PROPERTIES *****/
const double CONTROL_P = 0.4360;
const double CONTROL_I = 11.8990;
const double CONTROL_D = 0;
const double NOMINAL_VOLTAGE = 12;      // (V)
const double MAX_VOLTAGE = NOMINAL_VOLTAGE;


/***** CALCULATING MATRIXES *****/
const double Beq = Bm * N*N * eta + Bw;
const double Jeq = Jm * N*N * eta + Jw;
extern const Eigen::Matrix<double, 4, 3> M;         // robot velocities -> wheels velocities * wheels' radius
extern const Eigen::Matrix<double, 4, 3> M_over_r;   // robot velocities -> wheels velocities
extern const Eigen::Matrix<double, 3, 4> rMplus;    // wheels velocities -> robot velocities
extern const Eigen::Matrix4d Cv;
extern const Eigen::Matrix4d H;
extern const Eigen::Matrix4d F, Finv;
extern const Eigen::Matrix4d A, B, Binv, C;
extern const Eigen::Matrix4d R, Rinv, Q, Qinv;



namespace myArray {
    constexpr myMatrix4d A = {{
        {9.824336133474945e-01,  9.611513532313617e-03, -1.081038257466952e-02,  1.301876985722949e-02},
        {9.611513532313619e-03,  9.852338222768789e-01,  9.216746521759373e-03, -1.081038257466952e-02},
        {-1.081038257466952e-02, 9.216746521759371e-03,  9.852338222768789e-01,  9.611513532313617e-03},
        {1.301876985722949e-02, -1.081038257466952e-02,  9.611513532313621e-03,  9.824336133474945e-01}
    }};

    static const double Cvii = Beq + K*K + N*N * eta / RES_MOTOR;
    static const double Bii = (N * eta * K / RES_MOTOR) * Cvii;
    constexpr myMatrix4d B = {{
        {Bii, 0, 0, 0},
        {0, Bii, 0, 0},
        {0, 0, Bii, 0},
        {0, 0, 0, Bii}
    }};

    static const double Cii = 1;
    constexpr myMatrix4d C = {{
        {Cii, 0, 0, 0},
        {0, Cii, 0, 0},
        {0, 0, Cii, 0},
        {0, 0, 0, Cii}
    }};

    static const double Rii = STD_DEV_NOISE_MOV;
    constexpr myMatrix4d R = {{
        {Rii, 0, 0, 0},
        {0, Rii, 0, 0},
        {0, 0, Rii, 0},
        {0, 0, 0, Rii}
    }};

    static const double Qii = STD_DEV_NOISE_SENSOR;
    constexpr myMatrix4d Q = {{
        {Qii, 0, 0, 0},
        {0, Qii, 0, 0},
        {0, 0, Qii, 0},
        {0, 0, 0, Qii}
    }};
};


#endif
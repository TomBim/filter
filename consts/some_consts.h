#ifndef SOME_CONSTS_H
#define SOME_CONSTS_H

#include <eigen3/Eigen/Core>

const double PI = 3.1415926536;
const double deg2rad = PI / 180;

/* @todo fix the values */


/***** SIMULATION'S PROPERTIES *****/
const double Fs = 1000;      // sample rate (Hz)
const double Ts = 1 / Fs;    // sample period (s)
const double STD_DEV_NOISE = 0.1;    // standard deviation of the noise on observation


/***** ROBOT'S PROPERTIES *****/
const double ROBOT_RADIUS = 0.0825;      // robot's radius (m)
const double ALPHA_FRONT = 45 * deg2rad;  // angle between front wheels and the robot's front (WÔF) (rad)
const double ALPHA_REAR = 135 * deg2rad;  // angle between rear wheels and the robot's front (WÔF) (rad)
const double ROBOT_MASS = 2.23185;        // robot's mass (kg)
const double Icom = 15;      // moment of inertia related to the center of mass (SI)


/***** WHEELS' PROPERTIES *****/
const double WHEEL_RADIUS = 0.02275;       // wheels' radius (m)
const double Jw = 1;         // wheels' inertia (SI)
const double Bw = 1;         // wheels' friction coeff (SI)

/***** MOTOR'S PROPERTIES *****/
const double K = 1;          // torque constant (SI)
const double RES_MOTOR = 50;         // motor's resistence (ohm)
const double Jm = 1;         // motor's inertia (SI)
const double Bm = 1;         // motor's friction coeff (SI)
const int N_POLES = 8;       // encoder's number of poles
const int N_EDGES = 2;       // encoder's number of edges
const double counts2rad = 2 * PI / (N_POLES * N_EDGES * 3);    // rad per encoder's count
const double rad2counts = 1 / counts2rad;        // encoder's counts per rad

/***** REDUCTION'S PROPERTIES *****/
const double N = 3.0;         // reduction factor
const double eta = 0.9;      // reduction's efficiency

/***** CONTROLLER'S PROPERTIES *****/
const double CONTROL_P = 2;
const double CONTROL_D = 0.2;
const double CONTROL_I = 0;

/***** CALCULATING MATRIXES *****/
const double Beq = Bm * N*N * eta + Bw;
const double Jeq = Jm * N*N * eta + Jw;
extern const Eigen::Matrix<double, 4, 3> M;         // robot velocities -> wheels velocities * wheels' radius
const Eigen::Matrix<double, 4, 3> M_over_r = M / WHEEL_RADIUS;   // robot velocities -> wheels velocities
extern const Eigen::Matrix<double, 3, 4> rMplus;    // wheels velocities -> robot velocities
extern const Eigen::Matrix4d Cv;
extern const Eigen::Matrix4d H;
extern const Eigen::Matrix4d F, Finv;
extern const Eigen::Matrix4d A, B, Binv, C;
extern const Eigen::Matrix4d R, Rinv, Q, Qinv;

#endif
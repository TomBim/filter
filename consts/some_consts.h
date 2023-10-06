#ifndef SOME_CONSTS_H
#define SOME_CONSTS_H

#include <eigen3/Eigen/Eigen>

static const double PI = 3.1415926536;
static const double deg2rad = PI / 180;

/* @todo fix the values */


/***** SIMULATION'S PROPERTIES *****/
static const double Fs = 1000;      // sample rate (Hz)
static const double Ts = 1 / Fs;    // sample period (s)
static const double STD_DEV_NOISE = 0.1;    // standard deviation of the noise on observation


/***** ROBOT'S PROPERTIES *****/
static const double ROBOT_RADIUS = 0.0825;      // robot's radius (m)
static const double ALPHA_FRONT = 45 * deg2rad;  // angle between front wheels and the robot's front (WÔF) (rad)
static const double ALPHA_REAR = 135 * deg2rad;  // angle between rear wheels and the robot's front (WÔF) (rad)
static const double ROBOT_MASS = 2.23185;        // robot's mass (kg)
static const double Icom = 15;      // moment of inertia related to the center of mass (SI)


/***** WHEELS' PROPERTIES *****/
static const double WHEEL_RADIUS = 0.02275;       // wheels' radius (m)
static const double Jw = 1;         // wheels' inertia (SI)
static const double Bw = 1;         // wheels' friction coeff (SI)

/***** MOTOR'S PROPERTIES *****/
static const double K = 1;          // torque constant (SI)
static const double R = 50;         // motor's resistence (ohm)
static const double Jm = 1;         // motor's inertia (SI)
static const double Bm = 1;         // motor's friction coeff (SI)
static const int N_POLES = 8;       // encoder's number of poles
static const int N_EDGES = 2;       // encoder's number of edges
static const double counts2rad = 2 * PI / (N_POLES * N_EDGES * 3);    // rad per encoder's count
static const double rad2counts = 1 / counts2rad;        // encoder's counts per rad

/***** REDUCTION'S PROPERTIES *****/
static const double N = 3.0;         // reduction factor
static const double eta = 0.9;      // reduction's efficiency

/***** CONTROLLER'S PROPERTIES *****/
static const double CONTROL_P = 2;
static const double CONTROL_D = 0.2;
static const double CONTROL_I = 0;

/***** CALCULATING MATRIXES *****/
static const double Beq = Bm * N*N * eta + Bw;
static const double Jeq = Jm * N*N * eta + Jw;
static const Eigen::Matrix<double, 4, 3> M;         // robot velocities -> wheels velocities * wheels' radius
static const Eigen::Matrix<double, 4, 3> M_over_r = M / WHEEL_RADIUS;   // robot velocities -> wheels velocities
static const Eigen::Matrix<double, 3, 4> rMplus;    // wheels velocities -> robot velocities
static const Eigen::Matrix4d Cv;
static const Eigen::Matrix4d H;
static const Eigen::Matrix4d F, Finv;
static const Eigen::Matrix4d A, B, Binv, C;

#endif
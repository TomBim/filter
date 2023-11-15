#include "some_consts.h"

/*************************************************
**********         MATRIX M        ***************
*************************************************/
const Eigen::Matrix<double, 4, 3> M = (Eigen::Matrix<double, 4, 3>() <<
            -sin(ALPHA_FRONT),  cos(ALPHA_FRONT),   ROBOT_RADIUS,
            -sin(ALPHA_REAR),   cos(ALPHA_REAR),    ROBOT_RADIUS,
            sin(ALPHA_REAR),    cos(ALPHA_REAR),    ROBOT_RADIUS,
            sin(ALPHA_FRONT),   cos(ALPHA_FRONT),   ROBOT_RADIUS).finished();

const Eigen::Matrix<double, 4, 3> M_over_r = M / WHEEL_RADIUS;

static const double Mplus11 = -0.25 / sin(ALPHA_FRONT);
static const double Mplus12 = -0.25 / sin(ALPHA_REAR);
static const double Mplus21 = 0.5 / (cos(ALPHA_FRONT) - cos(ALPHA_REAR));
static const double Mplus31 = 0.5 * cos(ALPHA_REAR) \
                                / ROBOT_RADIUS / (cos(ALPHA_REAR) - cos(ALPHA_FRONT));
static const double Mplus32 = 0.5 * cos(ALPHA_FRONT) \
                                / ROBOT_RADIUS / (cos(ALPHA_FRONT) - cos(ALPHA_REAR));

const Eigen::Matrix<double, 3, 4> rMplus = WHEEL_RADIUS * (Eigen::Matrix<double, 3, 4>() <<
            Mplus11,     Mplus12,   -Mplus12,   -Mplus11,
            Mplus21,    -Mplus21,   -Mplus21,    Mplus21,
            Mplus31,     Mplus32,    Mplus32,    Mplus31).finished();

/**************************************************
***********        MATRIX H        ****************
**************************************************/
static const Eigen::DiagonalMatrix<double, 4> I_v_diag(Mplus11, Mplus12, -Mplus12, -Mplus11);
static const Eigen::DiagonalMatrix<double, 4> I_vn_diag(Mplus21, -Mplus21, -Mplus21, Mplus21);
static const Eigen::DiagonalMatrix<double, 4> I_omega_diag(Mplus31, Mplus32, Mplus32, Mplus31);
static const Eigen::Matrix4d I_v = ROBOT_MASS * pow(WHEEL_RADIUS, 2) * I_v_diag * Eigen::Matrix4d::Ones() * I_v_diag;
static const Eigen::Matrix4d I_vn = ROBOT_MASS * pow(WHEEL_RADIUS, 2) * I_vn_diag * Eigen::Matrix4d::Ones() * I_vn_diag;
static const Eigen::Matrix4d I_omega = Icom * pow(WHEEL_RADIUS, 2) * I_omega_diag * Eigen::Matrix4d::Ones() * I_omega_diag;

static const Eigen::Matrix4d Htau = I_v + I_vn + I_omega;
const Eigen::Matrix4d H = Htau + Jeq * Eigen::Matrix4d::Identity();

/*******************************************************
***********        OTHER MATRIXES      *****************
*******************************************************/
const Eigen::Matrix4d Cv = (Beq + K*K * N*N * eta / RES_MOTOR) * Eigen::Matrix4d::Identity();

const Eigen::Matrix4d F = - H.inverse() * Cv;
const Eigen::Matrix4d Finv = F.inverse();

const Eigen::Matrix4d A = (-Ts * H.inverse() * Cv).exp();
const Eigen::Matrix4d B = (Eigen::Matrix4d::Identity() - A) * (N * eta * K / RES_MOTOR) * Cv.inverse();
const Eigen::Matrix4d Binv = B.inverse();
const Eigen::Matrix4d C = Eigen::Matrix4d::Identity();

const Eigen::Matrix4d R = STD_DEV_NOISE_MOV * Eigen::Matrix4d::Identity();
const Eigen::Matrix4d Rinv = R.inverse();
const Eigen::Matrix4d Q = STD_DEV_NOISE_SENSOR * Eigen::Matrix4d::Identity();
const Eigen::Matrix4d Qinv = Q.inverse();



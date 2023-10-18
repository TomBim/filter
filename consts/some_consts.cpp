#include "some_consts.h"

/*************************************************/
/**********         MATRIX M        **************/
/*************************************************/
const Eigen::Matrix<double, 4, 3> M = (Eigen::Matrix<double, 4, 3>() <<
            -sin(ALPHA_FRONT),  cos(ALPHA_FRONT),   ROBOT_RADIUS,
            -sin(ALPHA_REAR),   cos(ALPHA_REAR),    ROBOT_RADIUS,
            sin(ALPHA_REAR),    cos(ALPHA_REAR),    ROBOT_RADIUS,
            sin(ALPHA_FRONT),   cos(ALPHA_FRONT),   ROBOT_RADIUS).finished();

const Eigen::Matrix<double, 4, 3> M_over_r = M / WHEEL_RADIUS;

const double Mplus11 = -0.25 / sin(ALPHA_FRONT);
const double Mplus12 = -0.25 / sin(ALPHA_REAR);
const double Mplus21 = 0.5 / (cos(ALPHA_FRONT) - cos(ALPHA_REAR));
const double Mplus31 = 0.5 * cos(ALPHA_FRONT) \
                                / ROBOT_RADIUS / (cos(ALPHA_FRONT) - cos(ALPHA_REAR));
const double Mplus32 = 0.5 * cos(ALPHA_REAR) \
                                / ROBOT_RADIUS / (cos(ALPHA_REAR) - cos(ALPHA_FRONT));

const Eigen::Matrix<double, 3, 4> rMplus = WHEEL_RADIUS * (Eigen::Matrix<double, 3, 4>() <<
            Mplus11,     Mplus12,   -Mplus12,   -Mplus11,
            Mplus21,    -Mplus21,   -Mplus21,    Mplus21,
            Mplus31,     Mplus32,    Mplus32,    Mplus31).finished();

/**************************************************/
/***********        MATRIX H        ***************/
/**************************************************/

const double Id_a = ROBOT_MASS * std::pow(WHEEL_RADIUS, 2) / 4;
const double Id_b = Icom * pow(WHEEL_RADIUS, 2) / 16 / pow(ROBOT_RADIUS, 2);

const double Id = Id_a + Id_b;
const double Iadj = Id_b;
const double Iop = Id_b - Id_a;

const Eigen::Matrix4d H = (Eigen::Matrix4d() << 
            Id+Jeq,     Iadj,       Iop,        Iadj,
            Iadj,       Id+Jeq,     Iadj,       Iop,
            Iop,        Iadj,       Id+Jeq,     Iadj,
            Iadj,       Iop,        Iadj,       Id+Jeq).finished();


/*****************************************************/
/*********        OTHER MATRIXES      ****************/
/*****************************************************/
const Eigen::Matrix4d Cv = (Beq + K*K * N*N * eta / RES_MOTOR) * Eigen::Matrix4d::Identity();

const Eigen::Matrix4d F = - H.inverse() * Cv;
const Eigen::Matrix4d Finv = F.inverse();

const Eigen::Matrix4d A = (-Ts * H.inverse() * Cv).exp();
const Eigen::Matrix4d B = (N * eta * K / RES_MOTOR) * Cv;
const Eigen::Matrix4d Binv = B.inverse();
const Eigen::Matrix4d C = Eigen::Matrix4d::Identity();

const Eigen::Matrix4d R = STD_DEV_NOISE_MOV * Eigen::Matrix4d::Identity();
const Eigen::Matrix4d Rinv = R.inverse();
const Eigen::Matrix4d Q = STD_DEV_NOISE_SENSOR * Eigen::Matrix4d::Identity();
const Eigen::Matrix4d Qinv = Q.inverse();
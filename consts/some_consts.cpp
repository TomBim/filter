#include "some_consts.h"

const Eigen::Matrix4d Cv = (Beq + K*K * N*N * eta / RES_MOTOR) * Eigen::Matrix4d::Identity();

static const double Id_a = ROBOT_MASS * pow(WHEEL_RADIUS, 2) / 4;
static const double Id_b = Icom * pow(WHEEL_RADIUS, 2) / 16 / pow(ROBOT_RADIUS, 2);

static const double Id = Id_a + Id_b;
static const double Iadj = Id_b;
static const double Iop = Id_b - Id_a;

const Eigen::Matrix4d H = (Eigen::Matrix4d() << 
            Id+Jeq,     Iadj,       Iop,        Iadj,
            Iadj,       Id+Jeq,     Iadj,       Iop,
            Iop,        Iadj,       Id+Jeq,     Iadj,
            Iadj,       Iop,        Iadj,       Id+Jeq).finished();

const Eigen::Matrix4d F = - H.inverse() * Cv;
const Eigen::Matrix4d Finv = F.inverse();

const Eigen::Matrix4d A = (- Ts * H.inverse() * Cv).exp();
const Eigen::Matrix4d B = (N * eta * K / RES_MOTOR) * Cv;
const Eigen::Matrix4d Binv = B.inverse();
const Eigen::Matrix4d C = Eigen::Matrix4d::Identity();
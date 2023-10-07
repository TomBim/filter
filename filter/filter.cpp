#include "filter.h"


#ifdef FILTER_V_1_0


// @todo there is something weird
// infoMatrix = 0 in the start, cool. But, then I have
// to invert it?? It doesn't make sense.

/*****************    K A L M A N    *****************/

KalmanFilter::KalmanFilter() : GaussianFilter() {
    resetFilter();
}

void KalmanFilter::estimate(const Eigen::Vector4d &cmd) {
    mu = A * mu + B * cmd;
    Sigma = A * Sigma * A.transpose() + R;
}

void KalmanFilter::filtrate(const Eigen::Vector4d &obs) {
    KalmanGain = Sigma * C * (C * Sigma * C.transpose() + Q).inverse();
    mu = mu + KalmanGain * (obs - C * mu);
    Sigma = (Eigen::Matrix4d::Identity() - KalmanGain * C) * Sigma;
}

Eigen::Vector4d KalmanFilter::applyFilter(const Eigen::Vector4d &voltageCmd,
        const Eigen::Vector4d &encodersRead) {
    estimate(voltageCmd);
    filtrate(encodersRead);
    return mu;
}

Eigen::Vector4d KalmanFilter::getLastFilteredSpd() const {
    return mu;
}

void KalmanFilter::resetFilter() {
    mu.setZero();
    Sigma.setConstant(INFINITY);
}

/*************    I N F O R M A T I O N    ***************/

InfoFilter::InfoFilter() : GaussianFilter() {
    resetFilter();
}

void InfoFilter::estimate(const Eigen::Vector4d &cmd) {
    Eigen::Matrix4d infoMatrixPrev_inv = infoMatrix_inv;    // store infoMatrix_inv
    
    infoMatrix_inv = A * infoMatrixPrev_inv * A.transpose() + R;    // update infoMatrix_inv
    infoMatrix = infoMatrix_inv.inverse();      // update infoMatrix

    infoVec = infoMatrix * (A * infoMatrixPrev_inv * infoVec + B * cmd);    // update infoVec
}

void InfoFilter::filtrate(const Eigen::Vector4d &obs) {
    infoMatrix = infoMatrix + C.transpose() * Qinv * C; // update infoMatrix
    infoMatrix_inv = infoMatrix.inverse();  // update infoMatrix_inv

    infoVec = infoVec + C.transpose() * Qinv * obs; // update infoVec

    mu = infoMatrix_inv * infoVec;  // update mean state
}

Eigen::Vector4d InfoFilter::applyFilter(const Eigen::Vector4d &voltageCmd,
        const Eigen::Vector4d &encodersRead) {
    estimate(voltageCmd);
    filtrate(encodersRead);
    return mu;
}

Eigen::Vector4d InfoFilter::getLastFilteredSpd() const {
    return mu;
}

void InfoFilter::resetFilter() {
    infoVec.setZero();
    infoMatrix.setZero();
    infoMatrix_inv.setConstant(INFINITY);
    mu.setZero();
}

#endif
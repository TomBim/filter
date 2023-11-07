#include "filter.h"


#ifdef FILTER_V_1_0


// @todo there is something weird
// infoMatrix = 0 in the start, cool. But, then I have
// to invert it?? It doesn't make sense.

GaussianFilter::~GaussianFilter() {}

/*****************    K A L M A N    *****************/

KalmanFilter::KalmanFilter() : GaussianFilter("KF") {
    resetFilter();
}

void KalmanFilter::predict(const Eigen::Vector4d &cmd) {
    #ifdef DBG_MODE
    debugBuffer << "#### PREDICTING" << std::endl
        << "   mu = " << mu.transpose() << std::endl
        << "   Sigma(0,:) = " << Sigma.row(0) << std::endl;
    #endif
    
    mu = A * mu  +  B * cmd;
    Sigma = A * Sigma * A.transpose()  +  R;
    
    #ifdef DBG_MODE
    debugBuffer << "   mu* = " << mu.transpose() << std::endl
        << "   Sigma* = " << Sigma.row(0) << std::endl;
    #endif
}

void KalmanFilter::filtrate(const Eigen::Vector4d &obs) {
    KalmanGain = Sigma * C * (C * Sigma * C.transpose()  +  Q).inverse();
    mu = mu  +  KalmanGain * (obs  -  C * mu);
    Sigma = (Eigen::Matrix4d::Identity()  -  KalmanGain * C) * Sigma;

    #ifdef DBG_MODE
    debugBuffer << "#### FILTRATING" << std::endl
        << "K[0,:] = " << KalmanGain.row(0) << std::endl
        << "mu = " << mu.transpose() << std::endl
        << "Sigma[0,:] = " << Sigma.row(0) << std::endl;
    #endif
}

Eigen::Vector4d KalmanFilter::applyFilter(const Eigen::Vector4d &voltageCmd,
        const Eigen::Vector4d &encodersRead) {
    #ifdef DBG_MODE
    debugBuffer << "### Type: " << type << "\n\n"
        << "voltageCmd = " << voltageCmd.transpose() << "\n\n"
        << "encodersRead = " << encodersRead.transpose() << "\n\n";
    #endif

    predict(voltageCmd);
    filtrate(encodersRead / N);

    return mu;
}

Eigen::Vector4d KalmanFilter::getLastFilteredSpd() const {
    return mu;
}

void KalmanFilter::resetFilter() {
    mu.setZero();
    Sigma = MY_INFINITE * Eigen::Matrix4d::Identity();
}

/*************    I N F O R M A T I O N    ***************/

InfoFilter::InfoFilter() : GaussianFilter("IF") {
    resetFilter();
}

void InfoFilter::predict(const Eigen::Vector4d &cmd) {
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
    predict(voltageCmd);
    filtrate(encodersRead / N);
    return mu;
}

Eigen::Vector4d InfoFilter::getLastFilteredSpd() const {
    return mu;
}

void InfoFilter::resetFilter() {
    infoVec.setZero();
    infoMatrix.setZero();
    infoMatrix_inv = MY_INFINITE * Eigen::Matrix4d::Identity();
    mu.setZero();
}

#endif



#ifdef FILTER_V_2_0

/****************   G A U S S I A N  *****************/
GaussianFilter::GaussianFilter(std::string type) : type(type) {
    mu.fill(0);
    totalSpentTime = 0;
    dt = 0;
}

GaussianFilter::~GaussianFilter() {}

void GaussianFilter::AtimesMTX(myMatrix4d &mtx, myMatrix4d &res, bool only2rows) {
    // we will consider as follow:
    //            [a   b   c   d]
    //      mtx = [b   e   f   c]
    //            [c   f   e   b]
    //            [d   c   b   a]
    double a = mtx[0][0];
    double b = mtx[0][1];
    double c = mtx[0][2];
    double d = mtx[0][3];
    double e = mtx[1][1];
    double f = mtx[1][2];


    if(only2rows) {
        // @todo: maybe we can work a little more
        res[0][0] = A11 * a + A12 * b + A13 * c + A14 * d;
        res[0][1] = A11 * b + A12 * e + A13 * f + A14 * c;
        res[0][2] = A11 * c + A12 * f + A13 * e + A14 * b;
        res[0][3] = A11 * d + A12 * c + A13 * b + A14 * a;

        res[1][0] = A12 * a + A22 * b + A23 * c + A13 * d;
        res[1][1] = A12 * b + A22 * e + A23 * f + A13 * c;
        res[1][2] = A12 * c + A22 * f + A23 * e + A13 * b;
        res[1][3] = A12 * d + A22 * c + A23 * b + A13 * a;
    }
    else {
        double aux;
        
        aux = A11 * a + A12 * b + A13 * c + A14 * d;
        res[0][0] = aux;
        res[3][3] = aux;
        aux = A11 * b + A12 * e + A13 * f + A14 * c;
        res[0][1] = aux;
        res[3][2] = aux;
        aux = A11 * c + A12 * f + A13 * e + A14 * b;
        res[0][2] = aux;
        res[3][1] = aux;
        aux = A11 * d + A12 * c + A13 * b + A14 * a;
        res[0][3] = aux;
        res[3][0] = aux;

        aux = A12 * a + A22 * b + A23 * c + A13 * d;
        res[1][0] = aux;
        res[2][3] = aux;
        aux = A12 * b + A22 * e + A23 * f + A13 * c;
        res[1][1] = aux;
        res[2][2] = aux;
        aux = A12 * c + A22 * f + A23 * e + A13 * b;
        res[1][2] = aux;
        res[2][1] = aux;
        aux = A12 * d + A22 * c + A23 * b + A13 * a;
        res[1][3] = aux;
        res[2][0] = aux;
    }
}


inline void GaussianFilter::A_MTX_At_plus_R_aux(myMatrix4d &mtx, myMatrix4d &res,
                                         myMatrix4d &A_MTX, bool exportAM) {
    /// @note we copy mtx. Therefore, there is no problem in
    //  in putting  res=mtx if you want something like:
    //          M  =  A.M.A + R
    // We will consider R as a diagonal matrix:
    //     R = Rii * I4 ,
    // where I4 is the identity of size 4.
    // Also, we will consider M as a bisymmetric matrix
    // we will consider as follow:
    //            [a   b   c   d]
    //      mtx = [b   e   f   c]
    //            [c   f   e   b]
    //            [d   c   b   a]


    // we will consider as follow:
    //            [a   b   c   d]
    //      mtx = [b   e   f   c]
    //            [c   f   e   b]
    //            [d   c   b   a]
    double a = mtx[0][0];
    double b = mtx[0][1];
    double c = mtx[0][2];
    double d = mtx[0][3];
    double e = mtx[1][1];
    double f = mtx[1][2];

    double am1, am2, am3, am4, ama_aux;
    
    // for the first line
    am1 = A11 * a + A12 * b + A13 * c + A14 * d;
    am2 = A11 * b + A12 * e + A13 * f + A14 * c;
    am3 = A11 * c + A12 * f + A13 * e + A14 * b;
    am4 = A11 * d + A12 * c + A13 * b + A14 * a;
        // A.MTX:
        if (exportAM) {
            A_MTX[0][0] = am1;
            A_MTX[3][3] = am1;
            A_MTX[0][1] = am2;
            A_MTX[3][2] = am2;
            A_MTX[0][2] = am3;
            A_MTX[3][1] = am3;
            A_MTX[0][3] = am4;
            A_MTX[3][0] = am4;
        }

        // A.MTX.At + R
        ama_aux = am1 * A11 + am2 * A12 + am3 * A13 + am4 * A14 + Rii;  // 1.1
        res[0][0] = ama_aux;    // res1.1
        res[3][3] = ama_aux;    // res4.4
        ama_aux = am1 * A12 + am2 * A22 + am3 * A23 + am4 * A13;    // 1.2
        res[1][0] = ama_aux;    // res2.1
        res[0][1] = ama_aux;    // res1.2
        res[2][3] = ama_aux;    // res3.4
        res[3][2] = ama_aux;    // res4.3
        ama_aux = am1 * A13 + am2 * A23 + am3 * A22 + am4 * A12;    // 1.3
        res[2][0] = ama_aux;    // res3.1
        res[0][2] = ama_aux;    // res1.3
        res[1][3] = ama_aux;    // res2.4
        res[3][1] = ama_aux;    // res4.2
        ama_aux = am1 * A14 + am2 * A13 + am3 * A12 + am4 * A11;    // 1.4
        res[3][0] = ama_aux;    // res4.1
        res[0][3] = ama_aux;    // res1.4


    // for the second line
    am1 = A12 * a + A22 * b + A23 * c + A13 * d;
    am2 = A12 * b + A22 * e + A23 * f + A13 * c;
    am3 = A12 * c + A22 * f + A23 * e + A13 * b;
    am4 = A12 * d + A22 * c + A23 * b + A13 * a;
        // A.MTX
        if(exportAM) {
            A_MTX[1][0] = am1;
            A_MTX[2][3] = am1;
            A_MTX[1][1] = am2;
            A_MTX[2][2] = am2;
            A_MTX[1][2] = am3;
            A_MTX[2][1] = am3;
            A_MTX[1][3] = am4;
            A_MTX[2][0] = am4;
        }

        // A.MTX.At + R
        ama_aux = am1 * A12 + am2 * A22 + am3 * A23 + am4 * A13 + Rii;    // 2.2
        res[1][1] = ama_aux;    // res2.2
        res[2][2] = ama_aux;    // res3.3
        ama_aux = am1 * A13 + am2 * A23 + am3 * A22 + am4 * A12;    // 2.3
        res[1][2] = ama_aux;    // res2.3
        res[2][1] = ama_aux;    // res3.2
}

void GaussianFilter::A_MTX_At_plus_R(myMatrix4d &mtx, myMatrix4d &res) {
    /// @todo this seems wrong. Don't think its good practice...
    A_MTX_At_plus_R_aux(mtx, res, mtx, false); 
}

void GaussianFilter::A_MTX_At_plus_R(myMatrix4d &mtx, myMatrix4d &res, myMatrix4d A_MTX) {
    A_MTX_At_plus_R_aux(mtx, res, A_MTX, true);
}

void GaussianFilter::invertBisymmetricMatrix(myMatrix4d &mtx, myMatrix4d &res) {
    /// @note we will use only the the elements from the first row (1.1, 1.2, 1.3, 1.4)
    /// and the two elements in the middle of the second row (2.2, 2.3).
    /// Therefore, this funcion doesn't care for the other elements
    
    /// @note we copy mtx. Therefore, there is no problem in
    //  in putting  res=mtx if you want something like:
    //          M = inv(M)

    // Cayley-Hamilton Method for 4x4 matrixes:
    //      Ainv = 1 / det(A) * (k1.I - k2.A + k3.A² - A³)
    // with:
    //      k1 = 1/6 * { [tr(A)]³ - 3.tr(A).tr(A²) + 2.tr(A³) }
    //      k2 = 1/2 * { [tr(A)]² - tr(A²) }
    //      k3 = tr(A)

    // we will consider as follow:
    //            [a   b   c   d]
    //      mtx = [b   e   f   c]
    //            [c   f   e   b]
    //            [d   c   b   a]
    double a = mtx[0][0];
    double b = mtx[0][1];
    double c = mtx[0][2];
    double d = mtx[0][3];
    double e = mtx[1][1];
    double f = mtx[1][2];

    // since we will use the squares a lot:
    double a2 = a * a;
    double b2 = b * b;
    double c2 = c * c;
    double d2 = d * d;
    double e2 = e * e;
    double f2 = f * f;

    // *** determinant ***
    /// @note det(mtx) = (a² - d²)(e² - f²) + (c² - b²)² +
    ///             + 4.b.c(a.f + d.e) - 2.(a.e + d.f).(c² + b²)
    double determinant_inv = 1.0 / ((a2 - d2)*(e2 - f2) + pow((c2 - b2), 2) \
                                    + 2.0 * (2.0*b*c*(a*f + d*e) \
                                            - (a*e + d*f)*(c2 + b2)
                                            )
                                    );

    // *** traces ***
    // note:
    //      tr(mtx) = a + e + e + a = 2.(a + e)
    //      tr(mtx²) = 2.(a² + 2.b² + 2.c² + d² + e² + f²)
    //      tr(mtx³) = 2 . [a.(a² + 3.b² + 3.c² + 3.d²) + 
    //                       + e.(3.b² + 3.c² + e² + 3.f²)]
    double tr_mtx = 2.0 * (a + e);

    double tr_mtx2 = 2.0 * (a2 + d2 + e2 + f2 + 2.0*(b2 + c2));

    double tr_mtx3 = 2.0 * (a * (a2 + 3.0*(b2 + c2 + d2)) \
                            + e * (e2 + 3.0*(b2 + c2 + f2)));
    
    
    // *** consts ***
    // note:
    //      k1 = 1/6 * { [tr(A)]³ - 3.tr(A).tr(A²) + 2.tr(A³) }
    //      k2 = 1/2 * { [tr(A)]² - tr(A²) }
    //      k3 = tr(A)
    // but, we can say:
    //      k1 = 1/3 * { tr(A).[k2 - tr(A²)] + tr(A³) }
    double k3_over_det = tr_mtx * determinant_inv;
    double k2_over_det = 0.5 * (tr_mtx*tr_mtx - tr_mtx2);
    double k1_over_det = (tr_mtx*(k2_over_det - tr_mtx2) + tr_mtx3) * determinant_inv / 3.0;
    k2_over_det *= determinant_inv;

    // *** the inverse ***
    // note:
    //      Ainv = 1 / det(A) * (k1.I - k2.A + k3.A² - A³)
    double mtxSqr1, mtxSqr2, mtxSqr3, mtxSqr4;
    double mtxCube;
    double inv_aux;

    // first line
    mtxSqr1 = a2 + b2 + c2 + d2;
    mtxSqr2 = b*(a + e) + c*(d + f);
    mtxSqr3 = c*(a + e) + b*(d + f);
    mtxSqr4 = 2.0*(a*d + b*c);
        // 1.1
        mtxCube = a * mtxSqr1 + b * mtxSqr2 + c * mtxSqr3 + d * mtxSqr4;
        inv_aux = k1_over_det - k2_over_det * a + k3_over_det * mtxSqr1 - determinant_inv * mtxCube;
        res[0][0] = inv_aux;
        res[3][3] = inv_aux;

        // 1.2
        mtxCube = b * mtxSqr1 + e * mtxSqr2 + f * mtxSqr3 + c * mtxSqr4;
        inv_aux = - k2_over_det * b + k3_over_det * mtxSqr2 - determinant_inv * mtxCube;
        res[0][1] = inv_aux;
        res[1][0] = inv_aux;
        res[2][3] = inv_aux;
        res[3][2] = inv_aux;

        // 1.3
        mtxCube = c * mtxSqr1 + f * mtxSqr2 + e * mtxSqr3 + b * mtxSqr4;
        inv_aux = - k2_over_det * c + k3_over_det * mtxSqr3 - determinant_inv * mtxCube;
        res[0][2] = inv_aux;
        res[2][0] = inv_aux;
        res[1][3] = inv_aux;
        res[3][1] = inv_aux;

        // 1.4
        mtxCube = d * mtxSqr1 + c * mtxSqr2 + b * mtxSqr3 + a * mtxSqr4;
        inv_aux = - k2_over_det * d + k3_over_det * mtxSqr4 - determinant_inv * mtxCube;
        res[0][3] = inv_aux;
        res[3][0] = inv_aux;



    // second line
    mtxSqr1 = mtxSqr2;
    mtxSqr4 = mtxSqr3;
    mtxSqr2 = b2 + c2 + e2 + f2;
    mtxSqr3 = 2.0*(b*c + e*f);
        // 2.2
        mtxCube = b * mtxSqr1 + e * mtxSqr2 + f * mtxSqr3 + c * mtxSqr4;
        inv_aux = k1_over_det - k2_over_det * e + k3_over_det * mtxSqr2 - determinant_inv * mtxCube;
        res[1][1] = inv_aux;
        res[2][2] = inv_aux;

        // 2.3
        mtxCube = c * mtxSqr1 + f * mtxSqr2 + e * mtxSqr3 + b * mtxSqr4;
        inv_aux = - k2_over_det * f + k3_over_det * mtxSqr3 - determinant_inv * mtxCube;
        res[1][2] = inv_aux;
        res[2][1] = inv_aux;
}

void GaussianFilter::multBisymmetricMatrixes(myMatrix4d &mtxA, const myMatrix4d &mtxB, myMatrix4d &res) {
    /// @note we copy mtxA. Therefore, there is no problem in
    //  in putting  res=mtx if you want something like:
    //          A = A*B
    /// @attention we do not copy the entire mtxB before assigning
    //  values to res. Therefore, do not put mtxB in res. If you
    //  want something like:
    //          B = A*B
    //  then, idk yet how to optimize it, BUT DO NOT USE res=mtxB

    // we will consider mtxA as follow:
    //             [a   b   c   d]
    //      mtxA = [b   e   f   c]
    //             [c   f   e   b]
    //             [d   c   b   a]
    // Reminder...
    //      A, B centrosymmetrics -> A*B centrosymmetric
    //      A, B symmetrics doesn't imply A*B symmetric
    //      A, B bisymmetric -> A*B centrosymmetric

    double a = mtxA[0][0];
    double b = mtxA[0][1];
    double c = mtxA[0][2];
    double d = mtxA[0][3];
    double e = mtxA[1][1];
    double f = mtxA[1][2];

    double mtxB1, mtxB2, mtxB3, mtxB4;
    double res_aux;
    
    // first column
    mtxB1 = mtxB[0][0];
    mtxB2 = mtxB[1][0];
    mtxB3 = mtxB[2][0];
    mtxB4 = mtxB[3][0];
        
        // 1.1
        res_aux = a * mtxB1 + b * mtxB2 + c * mtxB3 + d * mtxB4;
        res[0][0] = res_aux;
        res[3][3] = res_aux;

        // 2.1
        res_aux = b * mtxB1 + e * mtxB2 + f * mtxB3 + c * mtxB4;
        res[1][0] = res_aux;
        res[2][3] = res_aux;

        // 3.1
        res_aux = c * mtxB1 + f * mtxB2 + e * mtxB3 + b * mtxB4;
        res[2][0] = res_aux;
        res[1][3] = res_aux;

        // 4.1
        res_aux = d * mtxB1 + c * mtxB2 + b * mtxB3 + a * mtxB4;
        res[3][0] = res_aux;
        res[0][3] = res_aux;

    // second column
    mtxB1 = mtxB2;
    mtxB4 = mtxB3;
    mtxB2 = mtxB[1][1];
    mtxB3 = mtxB[2][1];

        // 1.2
        res_aux = a * mtxB1 + b * mtxB2 + c * mtxB3 + d * mtxB4;
        res[0][1] = res_aux;
        res[3][2] = res_aux;

        // 2.2
        res_aux = b * mtxB1 + e * mtxB2 + f * mtxB3 + c * mtxB4;
        res[1][1] = res_aux;
        res[2][2] = res_aux;

        // 3.2
        res_aux = c * mtxB1 + f * mtxB2 + e * mtxB3 + b * mtxB4;
        res[2][1] = res_aux;
        res[1][2] = res_aux;

        // 4.2
        res_aux = d * mtxB1 + c * mtxB2 + b * mtxB3 + a * mtxB4;
        res[3][1] = res_aux;
        res[0][2] = res_aux;
}

Eigen::Vector4d GaussianFilter::applyFilter(const Eigen::Vector4d &voltageCmd,
    const Eigen::Vector4d &encodersRead) {
    
    #ifdef DBG_MODE
    debugBuffer << "\n### Type: " << type << "\n"
        << "voltageCmd = " << voltageCmd.transpose() << "\n"
        << "encodersRead = " << encodersRead.transpose() << "\n";
    #endif

    // eigen vector to myVector
    myVector4d cmd, obs;
    for(int i = 0; i < 4; i++) {
        cmd[i] = voltageCmd[i];
        obs[i] = encodersRead[i] / N;
    }

    // execute filter
    dt = clock();
    predict(cmd);
    filtrate(obs);
    dt = clock() - dt;
    totalSpentTime += dt;

    #ifdef DBG_MODE
    debugBuffer << "timeSpent = " << dt << "\n";
    #endif

    // return filtered speed
    return getLastFilteredSpd();
}

Eigen::Vector4d GaussianFilter::getLastFilteredSpd() const {
    Eigen::Vector4d eigenFilteredSpd;
    for(int i = 0; i < 4; i++)
        eigenFilteredSpd[i] = mu.at(i);
    return eigenFilteredSpd;
}

double GaussianFilter::getTotalSpentTime() const {
    return totalSpentTime * 1.0 / CLOCKS_PER_SEC;
}

/*****************    K A L M A N    *****************/

KalmanFilter::KalmanFilter() : GaussianFilter("KF") {
    resetFilter();
    #ifdef DBG_MODE
    debugBuffer << "Initializing Kalman Filter" << std::endl << std::endl;
    #endif
}

void KalmanFilter::resetFilter() {
    totalSpentTime = 0;
    mu.fill(0);
    for(int i = 0; i < 4; i++) {
        Sigma[i].fill(0);
        Sigma[i][i] = MY_INFINITE;
    }
}

inline void KalmanFilter::predict(const myVector4d &cmd) {
    #ifdef DBG_MODE
    debugBuffer << "\n#### Predicting\n";
    debugBuffer << "mu = " << formatMyVector(mu) << "\n"
        << "Sigma[0,:] = " << formatMyMatrixOneRow(Sigma, 0) << "\n";
    #endif

    // mu = A*mu + B*u
    double mu0 = mu.at(0);
    double mu1 = mu.at(1);
    double mu2 = mu.at(2);
    double mu3 = mu.at(3);
    mu[0] = A11 * mu0 + A12 * mu1 + A13 * mu2 + A14 * mu3 \
            + Bii * cmd.at(0);
    mu[1] = A12 * mu0 + A22 * mu1 + A23 * mu2 + A13 * mu3 \
            + Bii * cmd.at(1);
    mu[2] = A13 * mu0 + A23 * mu1 + A22 * mu2 + A12 * mu3 \
            + Bii * cmd.at(2);
    mu[3] = A14 * mu0 + A13 * mu1 + A12 * mu2 + A11 * mu3 \
            + Bii * cmd.at(3);

    // Sigma = A*Sigma*At + R
    // Sigma is bisymmetric
    A_MTX_At_plus_R(Sigma, Sigma);

    #ifdef DBG_MODE
    debugBuffer << "mu* = " << formatMyVector(mu) << "\n"
        << "Sigma*[0,:] = " << formatMyMatrixOneRow(Sigma, 0) << "\n";
    #endif
}

inline void KalmanFilter::filtrate(const myVector4d &obs) {
    #ifdef DBG_MODE
    debugBuffer << "#### Filtrating\n\n";
    #endif

    // Kalman gain
    updateKalmanGain();

    // mu
    double mu0 = mu.at(0);
    double mu1 = mu.at(1);
    double mu2 = mu.at(2);
    double mu3 = mu.at(3);

    //          [s11  s12  s13  s14]
    // Sigma =  [s12  s22  s23  s13]
    //          [s13  s23  s22  s12]
    //          [s14  s13  s12  s11]
    double s11 = Sigma[0][0];
    double s12 = Sigma[0][1];
    double s13 = Sigma[0][2];
    double s14 = Sigma[0][3];
    double s22 = Sigma[1][1];
    double s23 = Sigma[1][2];

    // auxiliar vector:
    //      aux = z - C.mu
    double aux0 = obs.at(0) - Cii * mu0;
    double aux1 = obs.at(1) - Cii * mu1;
    double aux2 = obs.at(2) - Cii * mu2;
    double aux3 = obs.at(3) - Cii * mu3;

    // first column of K
    double k1 = KalmanGain[0][0];
    double k2 = KalmanGain[0][1];
    double k3 = KalmanGain[0][2];
    double k4 = KalmanGain[0][3];
        // mu: 0 and 3
        //      mu = mu + K.(z - C.mu)
        //                  ^---vv---^
        //                      aux        
        mu0 += k1 * aux0 + k2 * aux1 + k3 * aux2 + k4 * aux3;
        mu3 += k4 * aux0 + k3 * aux1 + k2 * aux2 + k1 * aux3;
        mu[0] = mu0;
        mu[3] = mu3;

        // Sigma: 1.1; 1.2; 1.3; 1.4; etc.
        //      Sigma = Sigma.Ct.(I - K.C)
        //                    ^----vv----^
        //                         K'
        // reminder: Sigma is bisymmetric
        k1 = Cii * (1.0 - k1 * Cii);
        k2 *= -Cii*Cii;
        k3 *= -Cii*Cii;
        k4 *= -Cii*Cii;
        double sigma_aux;
        sigma_aux = k1 * s11 + k2 * s12 + k3 * s13 + k4 * s14;
        Sigma[0][0] = sigma_aux;
        Sigma[3][3] = sigma_aux;
        sigma_aux = k1 * s12 + k2 * s22 + k3 * s23 + k4 * s13;
        Sigma[0][1] = sigma_aux;
        Sigma[1][0] = sigma_aux;
        Sigma[3][2] = sigma_aux;
        Sigma[2][3] = sigma_aux;
        sigma_aux = k1 * s13 + k2 * s23 + k3 * s22 + k4 * s12;
        Sigma[0][2] = sigma_aux;
        Sigma[2][0] = sigma_aux;
        Sigma[3][1] = sigma_aux;
        Sigma[1][3] = sigma_aux;
        sigma_aux = k1 * s14 + k2 * s13 + k3 * s12 + k4 * s11;
        Sigma[0][3] = sigma_aux;
        Sigma[3][0] = sigma_aux;       


    // second column of K
    k1 = KalmanGain[1][0];
    k2 = KalmanGain[1][1];
    k3 = KalmanGain[1][2];
    k4 = KalmanGain[1][3];
        // mu: 1 and 2
        //      mu = mu + K.(z - C.mu)
        //                  ^---vv---^
        //                      aux        
        mu1 += k1 * aux0 + k2 * aux1 + k3 * aux2 + k4 * aux3;
        mu2 += k4 * aux0 + k3 * aux1 + k2 * aux2 + k1 * aux3;
        mu[1] = mu1;
        mu[2] = mu2;

        // Sigma: 2.2; 2.3; etc.
        //      Sigma = Sigma.Ct.(I - K.C)
        //                    ^----vv----^
        //                         K'
        // reminder: Sigma is bisymmetric
        k1 *= -Cii*Cii;
        k2 = Cii * (1.0 - k2 * Cii);
        k3 *= -Cii*Cii;
        k4 *= -Cii*Cii;
        sigma_aux = k1 * s12 + k2 * s22 + k3 * s23 + k4 * s13;
        Sigma[1][1] = sigma_aux;
        Sigma[2][2] = sigma_aux;
        sigma_aux = k1 * s13 + k2 * s23 + k3 * s22 + k4 * s12;
        Sigma[1][2] = sigma_aux;
        Sigma[2][1] = sigma_aux;
    
    #ifdef DBG_MODE
    debugBuffer << "mu = " << formatMyVector(mu) << "\n"
        << "Sigma[0,:] = " << formatMyMatrixOneRow(Sigma, 0) << "\n";
    #endif
}

void KalmanFilter::updateKalmanGain() {
    // we will consider Sigma as follows:
    //              [a  b   c   d]
    //      Sigma = [b  e   f   c]
    //              [c  f   e   b]
    //              [d  c   b   a]

    double a = Sigma[0][0];
    double b = Sigma[0][1];
    double c = Sigma[0][2];
    double d = Sigma[0][3];
    double e = Sigma[1][1];
    double f = Sigma[1][2];

    // C.Sigma.Ct + Q
    auxMatrix[0][0] = a * Cii*Cii + Qii;
    auxMatrix[0][1] = b * Cii*Cii;
    auxMatrix[0][2] = c * Cii*Cii;
    auxMatrix[0][3] = d * Cii*Cii;
    auxMatrix[1][1] = e * Cii*Cii + Qii;
    auxMatrix[1][2] = f * Cii*Cii;

    // invert
    invertBisymmetricMatrix(auxMatrix, auxMatrix);

    // Sigma.Ct.[(...)^-1]
    double aux1, aux2, aux3, aux4, kalmanAux;
        // first column
        aux1 = auxMatrix[0][0];
        aux2 = auxMatrix[0][1];
        aux3 = auxMatrix[0][2];
        aux4 = auxMatrix[0][3];
            // 1.1
            kalmanAux = Cii * (a * aux1 + b * aux2 + c * aux3 + d * aux4);
            KalmanGain[0][0] = kalmanAux;
            KalmanGain[3][3] = kalmanAux;

            // 2.1
            kalmanAux = Cii * (b * aux1 + e * aux2 + f * aux3 + c * aux4);
            KalmanGain[1][0] = kalmanAux;
            KalmanGain[2][3] = kalmanAux;

            // 3.1
            kalmanAux = Cii * (c * aux1 + f * aux2 + e * aux3 + b * aux4);
            KalmanGain[2][0] = kalmanAux;
            KalmanGain[1][3] = kalmanAux;

            // 4.1
            kalmanAux = Cii * (d * aux1 + c * aux2 + b * aux3 + a * aux4);
            KalmanGain[3][0] = kalmanAux;
            KalmanGain[0][3] = kalmanAux;

        // second column
        aux1 = aux2;
        aux4 = aux3;
        aux2 = auxMatrix[1][1];
        aux3 = auxMatrix[1][2];
            // 1.2
            kalmanAux = Cii * (a * aux1 + b * aux2 + c * aux3 + d * aux4);
            KalmanGain[0][1] = kalmanAux;
            KalmanGain[3][2] = kalmanAux;

            // 2.2
            kalmanAux = Cii * (b * aux1 + e * aux2 + f * aux3 + c * aux4);
            KalmanGain[1][1] = kalmanAux;
            KalmanGain[2][2] = kalmanAux;

            // 3.2
            kalmanAux = Cii * (c * aux1 + f * aux2 + e * aux3 + b * aux4);
            KalmanGain[2][1] = kalmanAux;
            KalmanGain[1][2] = kalmanAux;

            // 4.2
            kalmanAux = Cii * (d * aux1 + c * aux2 + b * aux3 + a * aux4);
            KalmanGain[3][1] = kalmanAux;
            KalmanGain[0][2] = kalmanAux;

    #ifdef DBG_MODE
    debugBuffer << "KalmanGain[0,:]" << formatMyMatrixOneRow(KalmanGain, 0) << std::endl;
    #endif
}


/*************    I N F O R M A T I O N    ***************/

InfoFilter::InfoFilter() : GaussianFilter("IF") {
    resetFilter();
    #ifdef DBG_MODE
    debugBuffer << "Initializing Information Filter" << std::endl << std::endl;
    #endif
}

void InfoFilter::resetFilter() {
    totalSpentTime = 0;
    
    // we just need to set the inverse of the à posteriori
    // infoMatrix, since that's the only matrix we use in the start.
    // One can notice that, if we use Omega_posteriori(t=0) = 0, we have:
    //      Omega_posteriori(t=1) = Ct . Qinv . C
    // Therefore, we can initialize with that, instead of infinite.
    for(int i = 0; i < 4; i++)
        posterioriInfoMatrix_inv[i][i] = CQinvC_ii;
    infoVec.fill(0);
    mu.fill(0);
}

inline void InfoFilter::predict(const myVector4d &cmd) {
    // newOmega = (A . Omega_inv . At  +  R)^-1
    // newCsi = newOmega . (A . Omega_inv . csi  +  B.u)

    // update Omega
    // reminder: posterioriInfoMatrix_inv is already updated
    A_MTX_At_plus_R(posterioriInfoMatrix_inv, prioriInfoMatrix_inv, auxMatrix);
    invertBisymmetricMatrix(prioriInfoMatrix_inv, infoMatrix);

    // auxVec = (A . Omega_inv . csi   +  B.u)
    double csi1 = infoVec.at(0);
    double csi2 = infoVec.at(1);
    double csi3 = infoVec.at(2);
    double csi4 = infoVec.at(3);
    double auxVec_1, auxVec_2, auxVec_3, auxVec_4;
        // auxVec: first and fourth elements 
        double aOmegaInv1 = auxMatrix[0][0];
        double aOmegaInv2 = auxMatrix[0][1];
        double aOmegaInv3 = auxMatrix[0][2];
        double aOmegaInv4 = auxMatrix[0][3];
        auxVec_1 = aOmegaInv1 * csi1 + aOmegaInv2 * csi2 + aOmegaInv3 * csi3 + aOmegaInv4 * csi4 \
                   + Bii * cmd.at(0);
        auxVec_4 = aOmegaInv4 * csi1 + aOmegaInv3 * csi2 + aOmegaInv2 * csi3 + aOmegaInv1 * csi4 \
                   + Bii * cmd.at(3);

        // auxVec: second and third elements
        aOmegaInv1 = auxMatrix[1][0];
        aOmegaInv2 = auxMatrix[1][1];
        aOmegaInv3 = auxMatrix[1][2];
        aOmegaInv4 = auxMatrix[1][3];
        auxVec_2 = aOmegaInv1 * csi1 + aOmegaInv2 * csi2 + aOmegaInv3 * csi3 + aOmegaInv4 * csi4 \
                   + Bii * cmd.at(1);
        auxVec_3 = aOmegaInv4 * csi1 + aOmegaInv3 * csi2 + aOmegaInv2 * csi3 + aOmegaInv1 * csi4 \
                   + Bii * cmd.at(2);

    // update csi
    double omega1 = infoMatrix[0][0];
    double omega2 = infoMatrix[0][1];
    double omega3 = infoMatrix[0][2];
    double omega4 = infoMatrix[0][3];
    infoVec[0] = omega1 * auxVec_1 + omega2 * auxVec_2 + omega3 * auxVec_3 + omega4 * auxVec_4;
    infoVec[3] = omega4 * auxVec_1 + omega3 * auxVec_2 + omega2 * auxVec_3 + omega1 * auxVec_4;

    omega1 = omega2;
    omega4 = omega3;
    omega2 = infoMatrix[1][1];
    omega3 = infoMatrix[1][2];
    infoVec[1] = omega1 * auxVec_1 + omega2 * auxVec_2 + omega3 * auxVec_3 + omega4 * auxVec_4;
    infoVec[2] = omega4 * auxVec_1 + omega3 * auxVec_2 + omega2 * auxVec_3 + omega1 * auxVec_4;
}

inline void InfoFilter::filtrate(const myVector4d &obs) {
    // newOmega = Omega + Ct . Q_inv . C
    // newCsi = csi + Ct . Q_inv . z
    
    // updating Omega
    // only need to update diagonal
    double aux = infoMatrix[0][0] + CQinvC_ii;
    infoMatrix[0][0] = aux;
    infoMatrix[3][3] = aux;
    aux = infoMatrix[1][1] + CQinvC_ii;
    infoMatrix[1][1] = aux;
    infoMatrix[2][2] = aux;

    // update the inverse of infoMatrix à posteriori
    invertBisymmetricMatrix(infoMatrix, posterioriInfoMatrix_inv);

    // updating csi
    double infoVec1 = infoVec[0] + CQinv_ii * obs.at(0);
    double infoVec2 = infoVec[1] + CQinv_ii * obs.at(1);
    double infoVec3 = infoVec[2] + CQinv_ii * obs.at(2);
    double infoVec4 = infoVec[3] + CQinv_ii * obs.at(3); 
    infoVec[0] = infoVec1;
    infoVec[1] = infoVec2;
    infoVec[2] = infoVec3;
    infoVec[3] = infoVec4;

    // update mu
    // mu = Omega_inv * csi
        // first and fourth elements
        double omegaInv_1 = posterioriInfoMatrix_inv[0][0];
        double omegaInv_2 = posterioriInfoMatrix_inv[0][1];
        double omegaInv_3 = posterioriInfoMatrix_inv[0][2];
        double omegaInv_4 = posterioriInfoMatrix_inv[0][3];

        mu[0] = omegaInv_1 * infoVec1 * omegaInv_2 * infoVec2 \
                + omegaInv_3 * infoVec3 + omegaInv_4 * infoVec4;
        mu[3] = omegaInv_4 * infoVec1 * omegaInv_3 * infoVec2 \
                + omegaInv_2 * infoVec3 + omegaInv_1 * infoVec4;

        // second and third elements
        omegaInv_1 = omegaInv_2;
        omegaInv_4 = omegaInv_3;
        omegaInv_2 = posterioriInfoMatrix_inv[1][1];
        omegaInv_3 = posterioriInfoMatrix_inv[1][2];

        mu[1] = omegaInv_1 * infoVec1 * omegaInv_2 * infoVec2 \
                + omegaInv_3 * infoVec3 + omegaInv_4 * infoVec4;
        mu[2] = omegaInv_4 * infoVec1 * omegaInv_3 * infoVec2 \
                + omegaInv_2 * infoVec3 + omegaInv_1 * infoVec4;
}

#endif
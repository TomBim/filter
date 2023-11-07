#include "some_functions.h"

std::stringstream debugBuffer("");

void progressBar(float progress) {
    const int barWidth = 40;

    if(progress < 0)
        progress = 0;
    
    if(progress < 1) {
        int pos = progress * barWidth;
        std::cout << "[";
        for(int i = 0; i < pos; i++)
            std::cout << "=";
        for(int i = pos; i < barWidth; i++)
            std::cout << " ";
        std::cout << "] " << int(progress * 100) << " %\r";
        std::cout.flush();
    } else {
        std::cout << "Done!";
        int nChars2Delete = barWidth + 2 + 3 + 3 - 5; // bar + '[',']' + 3 numbers + ' ',' ','%' = 2+3+3
        for(int i = 0; i < nChars2Delete; i++)
            std::cout << " ";
        std::cout << std::endl;
    }
}

void multBisymmetricMatrixes_4x4(double **M1, double **M2, double **res, bool only2rows) {
    double a11 = M1[0][0];
    double a12 = M1[0][1];
    double a13 = M1[0][2];
    double a14 = M1[0][3];
    double a22 = M1[1][1];
    double a23 = M1[1][2];
    
    double b11 = M2[0][0];
    double b12 = M2[0][1];
    double b13 = M2[0][2];
    double b14 = M2[0][3];
    double b22 = M2[1][1];
    double b23 = M2[1][2];

    if(only2rows) {
        res[0][0] = a11*b11 + a12*b12 + a13*b13 + a14*b14;
        res[0][1] = a11*b12 + a12*b22 + a13*b23 + a14*b13;
        res[0][2] = a11*b13 + a12*b23 + a13*b22 + a14*b12;
        res[0][3] = a11*b14 + a12*b13 + a13*b12 + a14*b11;

        res[1][0] = a12*b11 + a22*b12 + a23*b13 + a13*b14;
        res[1][1] = a12*b12 + a22*b22 + a23*b23 + a13*b13;
        res[1][2] = a12*b13 + a22*b23 + a23*b22 + a13*b12;
        res[1][3] = a12*b14 + a22*b13 + a23*b12 + a13*b11;
    }
    else {
        double aux;
        
        aux = a11*b11 + a12*b12 + a13*b13 + a14*b14;
        res[0][0] = aux;
        res[3][3] = aux;
        aux = a11*b12 + a12*b22 + a13*b23 + a14*b13;
        res[0][1] = aux;
        res[3][2] = aux;
        aux = a11*b13 + a12*b23 + a13*b22 + a14*b12;
        res[0][2] = aux;
        res[3][1] = aux;
        aux = a11*b14 + a12*b13 + a13*b12 + a14*b11;
        res[0][3] = aux;
        res[3][0] = aux;

        aux = a12*b11 + a22*b12 + a23*b13 + a13*b14;
        res[1][0] = aux;
        res[2][3] = aux;
        aux = a12*b12 + a22*b22 + a23*b23 + a13*b13;
        res[1][1] = aux;
        res[2][2] = aux;
        aux = a12*b13 + a22*b23 + a23*b22 + a13*b12;
        res[1][2] = aux;
        res[2][1] = aux;
        aux = a12*b14 + a22*b13 + a23*b12 + a13*b11;
        res[1][3] = aux;
        res[2][0] = aux;
    }
}

void congruentMatrix_bisymmetricMatrixes_4x4(double **M1, double M2, double **res) {
    /// @attention not implemented
    std::cout << "congruentMatrix_bisymmetricMatrix_4x4 is not implemented. Don't use it.";
}

void multiply_A_BisymmetricMatrix(double **mtx, double **res, bool only2rows) {
    // we will consider as follow:
    //            [a   b   c   d]
    //      mtx = [b   e   f   c]
    //            [c   f   e   b]
    //            [d   c   b   a]

    std::cout << "multiply_A_BisymmetricMatrix in some_functions.cpp is not implemented.";

    // double a11 = M1[0][0];
    // double a12 = M1[0][1];
    // double a13 = M1[0][2];
    // double a14 = M1[0][3];
    // double a22 = M1[1][1];
    // double a23 = M1[1][2];
    
    // double b11 = M2[0][0];
    // double b12 = M2[0][1];
    // double b13 = M2[0][2];
    // double b14 = M2[0][3];
    // double b22 = M2[1][1];
    // double b23 = M2[1][2];

    // if(only2rows) {
    //     res[0][0] = a11*b11 + a12*b12 + a13*b13 + a14*b14;
    //     res[0][1] = a11*b12 + a12*b22 + a13*b23 + a14*b13;
    //     res[0][2] = a11*b13 + a12*b23 + a13*b22 + a14*b12;
    //     res[0][3] = a11*b14 + a12*b13 + a13*b12 + a14*b11;

    //     res[1][0] = a12*b11 + a22*b12 + a23*b13 + a13*b14;
    //     res[1][1] = a12*b12 + a22*b22 + a23*b23 + a13*b13;
    //     res[1][2] = a12*b13 + a22*b23 + a23*b22 + a13*b12;
    //     res[1][3] = a12*b14 + a22*b13 + a23*b12 + a13*b11;
    // }
    // else {
    //     double aux;
        
    //     aux = a11*b11 + a12*b12 + a13*b13 + a14*b14;
    //     res[0][0] = aux;
    //     res[3][3] = aux;
    //     aux = a11*b12 + a12*b22 + a13*b23 + a14*b13;
    //     res[0][1] = aux;
    //     res[3][2] = aux;
    //     aux = a11*b13 + a12*b23 + a13*b22 + a14*b12;
    //     res[0][2] = aux;
    //     res[3][1] = aux;
    //     aux = a11*b14 + a12*b13 + a13*b12 + a14*b11;
    //     res[0][3] = aux;
    //     res[3][0] = aux;

    //     aux = a12*b11 + a22*b12 + a23*b13 + a13*b14;
    //     res[1][0] = aux;
    //     res[2][3] = aux;
    //     aux = a12*b12 + a22*b22 + a23*b23 + a13*b13;
    //     res[1][1] = aux;
    //     res[2][2] = aux;
    //     aux = a12*b13 + a22*b23 + a23*b22 + a13*b12;
    //     res[1][2] = aux;
    //     res[2][1] = aux;
    //     aux = a12*b14 + a22*b13 + a23*b12 + a13*b11;
    //     res[1][3] = aux;
    //     res[2][0] = aux;
    // }
}

void startDebugLog() {
    debugBuffer.clear();
    std::ofstream file;
    file.open(LOG_MD_PATH, std::ios_base::out);
    if(file.is_open()) {
        file << "---" << std::endl
            << "marp: true" << std::endl
            << "theme: uncover " << std::endl
            << "headingDivider: 1" << std::endl
            << "style: |" << std::endl
            << "   section {" << std::endl
            << "   font-size: 10px;" << std::endl
            << "   background-color: #222;" << std::endl
            << "   color: #fff;" << std::endl
            << "   text-align: left;" << std::endl
            << "   }" << std::endl
            << std::endl
            << "---" << std::endl;
    }
}

void updateDebugLog() {
    std::ofstream file(LOG_MD_PATH, std::ios_base::app);
    std::string aux;
    if(file.is_open()) {
        file << debugBuffer.rdbuf();
        debugBuffer.clear();
    }
}

std::string formatMyVector(const myVector4d &vec) {
    std::stringstream ss;
    std::string str;
    ss << vec.at(0);
    for(int i = 1; i < 4; i++)
        ss << "\t" << vec.at(i);
    str = ss.str();
    return str;
}

std::string formatMyMatrixOneRow(const myMatrix4d &mtx, int row) {
    const myVector4d vec = mtx.at(row);
    return formatMyVector(vec);
}
#ifndef SOME_FUNCTIONS_H
#define SOME_FUNCTIONS_H

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <eigen3/Eigen/Core>

#include "../consts/some_consts.h"


extern std::stringstream debugBuffer;

void progressBar(float progress);

/// @note bisymmetric matrixes are symmetric about both of its diagonals.
/// Therefore, a bisymmetric is also a centrosymmetric matrix.
/// @note Property:
///     A, B bisymmetric -> A*B centrosymetric
/// @note a centrosymmetric matrix is a matrix with a symmetry in
/// its center:
///     a(i,j) = a(m,n),
///         for i + m = j + n = dimension + 1
/// @note centrosymmetric property:
///     A, B centrosymmetric -> A*B centrosymmetric
void multBisymmetricMatrixes_4x4(double **A, double **B, double **res);

void startDebugLog();

void updateDebugLog();

std::string formatMyVector(const myVector4d &vec);

std::string formatMyMatrixOneRow(const myMatrix4d &mtx, int row);


#endif
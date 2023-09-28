#ifndef SOME_FUNCTIONS_H
#define SOME_FUNCTIONS_H

#include <string>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Eigen>

void progressBar(float progress);

void logger(const std::string fileName, Eigen::VectorXd &data);

#endif
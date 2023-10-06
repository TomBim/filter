#ifndef MAIN_H
#define MAIN_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <eigen3/Eigen/Eigen>
#include <random>
#include <ctime>
#include <string>

#include "simulator/robot.h"
#include "simulator/some_functions.h"

static std::random_device rd;
static std::mt19937_64 gen(rd());
static std::normal_distribution<double> gaussian_distr(0, 1);

int main();

#endif
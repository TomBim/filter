#ifndef MAIN_H
#define MAIN_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <eigen3/Eigen/Eigen>
#include <random>
#include <ctime>
#include <string>

#include "consts/some_consts.h"
#include "simulator/some_functions.h"
#include "simulator/spd_types.h"
#include "simulator/cmd_signal.h"
#include "simulator/simulator.h"

std::random_device rd;
std::mt19937_64 gen(rd());
std::normal_distribution<double> gaussian_distr(0, 1);

int main();

#endif

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <eigen3/Eigen/Eigen>
#include <random>
#include <ctime>
#include <string>

#include "simulator/robot.h"

static std::random_device rd;
static std::mt19937_64 gen(rd());
static std::normal_distribution gaussian_distr;

int main(){

    for (int i = 0; i < 5; i++) {
        double n = gaussian_distr(gen);
        std::cout << "random number: " << n << std::endl;
    }

    return 0;
}
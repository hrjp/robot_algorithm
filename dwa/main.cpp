/**
* @file dwa/main.cpp
* @brief Dynamic Window Approach¶ algorithm sample
* @author Shunya Hara
* @date 2021.2.2
* @details Dynamic Window Approach¶ algorithm sample (c++)
* @ref https://pythonrobotics.readthedocs.io/en/latest/modules/path_planning/path_planning.html#dynamic-window-approach
**/

#include <iostream>
#include <cmath>
#include <string>

#include <Eigen/Eigen>
#include <matplotlib-cpp/matplotlibcpp.h>

namespace plt = matplotlibcpp;

class DWA{
private:

//robot param
double max_accel=1.0;
double max_vel=1.0;
double max_angular_accel=1.0;
double max_angular_vel=1.0;
int linear_predict_resolution=10;
int angular_predict_resolution=10;
double dt=0.01;

public:

};

int main() {

    return 0;
}

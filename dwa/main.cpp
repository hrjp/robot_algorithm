/**
* @file dwa/main.cpp
* @brief Dynamic Window Approach¶ algorithm sample
* @author Shunya Hara
* @date 2022.3.16
* @details Dynamic Window Approach¶ algorithm sample (c++)
* @ref https://pythonrobotics.readthedocs.io/en/latest/modules/path_planning/path_planning.html#dynamic-window-approach
**/

#include <iostream>
#include <cmath>
#include <string>

#include <Eigen/Eigen>
#include <matplotlib-cpp/matplotlibcpp.h>

#include "include/DWA.h"
#include "include/Position.h"

namespace plt = matplotlibcpp;

int main() {
    
    Position robot_pos(0.0, 0.0);
    Position goal_pos(1.0, 1.0);
    std::vector<Position> map;
    DWA dwa;

    dwa.calc(map,robot_pos,goal_pos);

    // show plots
    plt::show();
    return 0;
}

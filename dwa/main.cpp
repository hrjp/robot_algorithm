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

class Position{
    public:
    double x=0.0;
    double y=0.0;
};

class DWA{
private:

//robot param
double max_accel=1.0;
double max_vel=1.0;
double max_angular_accel=1.0;
double max_angular_vel=1.0;
//予測曲線の本数
int linear_predict_resolution=10;
int angular_predict_resolution=10;
//制御周期
double dt=0.01;
//何秒先まで予測するか
double predict_time=1.0;

//最大最小でクリップする
template <class T> T clip(const T& n, const T& lower, const T& upper){return std::max(lower, std::min(n, upper));}

public:
void calc(const std::vector<double>& map_x, const std::vector<double>& map_y, const Position& robot_pos, const Position& goal_pos);

};

void DWA::calc(const std::vector<double>& map_x, const std::vector<double>& map_y, const Position& robot_pos, const Position& goal_pos){

    double robot_vel;
    double robot_angular_vel;

    //取りうる速度と角速度の候補
    std::vector<double> linear_vels;
    std::vector<double> angular_vels;
    for(int i=0;i<linear_predict_resolution;i++){
        double linear_vel=robot_vel+(1-double(i)/double(linear_predict_resolution)*2.0)*max_accel*dt;
        linear_vel=clip(linear_vel,-max_vel,max_vel);
        linear_vels.push_back(linear_vel);
    }
    for(int i=0;i<angular_predict_resolution;i++){
        double angular_vel=robot_angular_vel+(1-double(i)/double(angular_predict_resolution)*2.0)*max_angular_accel*dt;
        angular_vel=clip(angular_vel,-max_angular_vel,max_angular_vel);
        angular_vels.push_back(angular_vel);
    }

    for(const auto& linear_vel : linear_vels){
        for(const auto& angular_vel : angular_vels){
            
        }
    }

}

int main() {
    


    // show plots
    plt::show();
    return 0;
}

/**
* @file DWA.h
* @brief Dynamic Window Approach¶ algorithm sample
* @author Shunya Hara
* @date 2022.3.17
* @details Dynamic Window Approach¶ algorithm sample (c++)
* @ref https://pythonrobotics.readthedocs.io/en/latest/modules/path_planning/path_planning.html#dynamic-window-approach
**/


#pragma once
#include <iostream>
#include <cmath>
#include <string>

#include <Eigen/Eigen>
#include <matplotlib-cpp/matplotlibcpp.h>

#include "Position.h"

namespace plt = matplotlibcpp;

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
void calc(const std::vector<Position>& map, const Position& robot_pos, const Position& goal_pos);
std::vector<Position> path_calc(const Position& robot_pos, double linear_vel, double angular_vel);
void path_plot(const std::vector<Position>& path);


};

std::vector<Position> DWA::path_calc(const Position& robot_pos, double linear_vel, double angular_vel){
    std::vector<Position> path;
    Position path_point;
    Eigen::MatrixXd rot(3,2);
    rot<<   cos(robot_pos.yaw), 0,
            sin(robot_pos.yaw), 0,
            0                 , 1;
    Eigen::Vector3d x(robot_pos.x,robot_pos.y,robot_pos.yaw);

    Eigen::Vector2d v(linear_vel,angular_vel);

    int step=predict_time/dt;
    for(int i=0; i<step;i++){
        x+=rot*v*dt;
        rot(0,0)=cos(rot(2,1));
        rot(1,0)=cos(rot(2,1));
        path.push_back(Position(x));
    }
    return path;
}

void DWA::calc(const std::vector<Position>& map, const Position& robot_pos, const Position& goal_pos){

    double robot_vel;
    double robot_angular_vel;

    //取りうる速度と角速度の候補
    std::vector<double> linear_vels;
    std::vector<double> angular_vels;
    for(int i=0;i<=linear_predict_resolution;i++){
        double linear_vel=robot_vel+(1-double(i)/double(linear_predict_resolution)*2.0)*max_accel*dt;
        linear_vel=clip(linear_vel,-max_vel,max_vel);
        linear_vels.push_back(linear_vel);
    }
    for(int i=0;i<=angular_predict_resolution;i++){
        double angular_vel=robot_angular_vel+(1-double(i)/double(angular_predict_resolution)*2.0)*max_angular_accel*dt;
        angular_vel=clip(angular_vel,-max_angular_vel,max_angular_vel);
        angular_vels.push_back(angular_vel);
    }

    for(const auto& linear_vel : linear_vels){
        for(const auto& angular_vel : angular_vels){
            std::cout<<linear_vel<<","<<angular_vel<<std::endl;
        }
    }

}


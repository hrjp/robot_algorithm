/**
* @file Position.h
* @brief 2D Position class
* @author Shunya Hara
* @date 2022.3.17
* @details 2D Position class
**/

#pragma once
#include <Eigen/Eigen>

class Position{
    public:
    Position(){}
    Position(double x_, double y_, double yaw_=0.0){
        x=x_;
        y=y_;
        yaw=yaw_;
    }
    Position(Eigen::Vector3d vec){
        x=vec(0);
        y=vec(1);
        yaw=vec(2);
    }
    double x=0.0;
    double y=0.0;
    double yaw=0.0;
};
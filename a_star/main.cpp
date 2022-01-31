#include <iostream>
#include <cmath>
 
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;


class Costmap{
    public:
    std::vector<std::vector<int> > cost_map;
    double width;
    double height;
    double resolution;
};

class Position{
    public:
    double x;
    double y;
};

class Astar{
    public:
    Astar();
    std::vector<Position> calc(const Costmap& costmap,const Position& start_pos,const Position& goal_pos);
    private:
    std::vector<int> search_x={1,1,0,-1,-1,-1,0,1};
    std::vector<int> search_y={0,1,1,1,0,-1,-1,-1};
    std::vector<double> search_cost={1,M_SQRT2,1,M_SQRT2,1,M_SQRT2,1,M_SQRT2};
};

Astar::Astar(){

}
 
std::vector<Position> Astar::calc(const Costmap& costmap,const Position& start_pos,const Position& goal_pos){
    int sx=start_pos.x/costmap.resolution;
    int sy=start_pos.y/costmap.resolution;
    int gx=goal_pos.x/costmap.resolution;
    int gy=goal_pos.y/costmap.resolution;
    

}

int main() {



    plt::plot({1, 2, 4, 8, 16});
    plt::show();
    return 0;
}

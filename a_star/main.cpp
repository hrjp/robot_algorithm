/**
* @file a_star/main.cpp
* @brief A* algorithm sample
* @author Shunya Hara
* @date 2021.2.2
* @details A* algorithm sample (c++)
* @ref https://pythonrobotics.readthedocs.io/en/latest/modules/path_planning/path_planning.html#a-algorithm
*/

#include <iostream>
#include <cmath>
#include <string>
 
#include <matplotlib-cpp/matplotlibcpp.h>

namespace plt = matplotlibcpp;

class Costmap{
    public:
    std::vector<std::vector<int> > cost_map;
    double width;
    double height;
    double resolution;
    int grid_x(){return int(width/resolution);}
    int grid_y(){return int(height/resolution);}
    Costmap(double width_,double height_,double resolution_);
};

Costmap::Costmap(double width_,double height_,double resolution_){
    width=width_;
    height=height_;
    resolution=resolution_;
    cost_map.assign(int(width/resolution), std::vector<int>(int(height/resolution),0));
}

class Position{
    public:
    double x;
    double y;
    Position(double x_,double y_){
        x=x_;
        y=y_;
    }
};

class Astar{
    public:
    Astar();
    std::vector<Position> calc(const Costmap& costmap,const Position& start_pos,const Position& goal_pos);
    private:
    std::vector<int> search_x={1,1,0,-1,-1,-1,0,1};
    std::vector<int> search_y={0,1,1,1,0,-1,-1,-1};
    std::vector<double> search_cost={1,M_SQRT2,1,M_SQRT2,1,M_SQRT2,1,M_SQRT2};
    class Grid{
        public:
        double from_start=0;
        double from_goal=0;
        double sum_cost(){return from_start+from_goal;}
        int status=0;
    };
    std::vector<std::vector<Grid> > cost;
    const int close_status=-1;
    const int open_status=1;
    const int free_status=0;
    const int wall_satatus=2;
    double distance(double x1,double y1,double x2,double y2);
};

Astar::Astar(){

}

double Astar::distance(double x1,double y1,double x2,double y2){
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}


 
std::vector<Position> Astar::calc(const Costmap& costmap,const Position& start_pos,const Position& goal_pos){
    int sx=start_pos.x/costmap.resolution;
    int sy=start_pos.y/costmap.resolution;
    int gx=goal_pos.x/costmap.resolution;
    int gy=goal_pos.y/costmap.resolution;
    int grid_size_x=int(costmap.width/costmap.resolution);
    int grid_size_y=int(costmap.height/costmap.resolution);
    
    std::map<std::string,std::string> key;
    std::vector<int> pltx={sx};
    std::vector<int> plty={sy};
    key["marker"]="x";
    plt::scatter(pltx,plty,100.0,key);
    pltx[0]=gx;
    plty[0]=gy;
    key["marker"]="*";
    plt::scatter(pltx,plty,100.0,key);

    //vector init
    cost.assign(grid_size_x, std::vector<Grid>(grid_size_y));
    int tar_x=sx;
    int tar_y=sy;
    std::vector<int> search_point_x;
    std::vector<int> search_point_y;
    //std::cout<<cost[gx].size()<<std::endl;

    for(int y=0;y<costmap.cost_map.size();y++){
        for (int x=0;x<costmap.cost_map.size();x++){
            if(costmap.cost_map[y][x]==100){
                cost[y][x].status=wall_satatus;
            }
                
        }
    }

    while(cost[gy][gx].status==free_status){
        
        //注目点の周囲のコストを計算
        cost[tar_y][tar_x].status=close_status;
        for(int i=0;i<8;i++){
            if(cost[tar_y+search_y[i]][tar_x+search_x[i]].status==free_status){
                cost[tar_y+search_y[i]][tar_x+search_x[i]].from_start=cost[tar_y][tar_x].from_start+search_cost[i]*(1+costmap.cost_map[tar_y][tar_x]);
                cost[tar_y+search_y[i]][tar_x+search_x[i]].from_goal=distance(tar_x+search_x[i],tar_y+search_y[i],gx,gy);
                cost[tar_y+search_y[i]][tar_x+search_x[i]].status=open_status;
            }
        }
        
        //コストが最小の点を探索して次の注目点を決定
        int mx=0,my=0;
        double min_cost=100.0*grid_size_x*grid_size_y;
        for(int y=1;y<grid_size_y-1;y++){
            for (int x=1;x<grid_size_x-1;x++){
                if(cost[y][x].status==open_status){
                    if(cost[y][x].sum_cost()<min_cost){
                        min_cost=cost[y][x].sum_cost();
                        mx=x;
                        my=y;
                    }
                }
            }
        }
        tar_x=mx;
        tar_y=my;
        search_point_x.push_back(mx);
        search_point_y.push_back(my);
        pltx[0]=tar_x;
        plty[0]=tar_y;
        //std::cout<<tar_x<<","<<tar_y<<","<<min_cost<<std::endl;
        key["c"]="g";
        plt::scatter(pltx,plty,2.0,key);
        plt::pause(0.00001);
        
    }

    //true path
    std::vector<int> path_x={gx};
    std::vector<int> path_y={gy};
    tar_x=gx;
    tar_y=gy;
    int mx=0,my=0;
    
    while(!(tar_x==sx && tar_y==sy)){
        double min_cost=100.0*grid_size_x*grid_size_y;
        for(int i=0;i<8;i++){
            if(cost[tar_y+search_y[i]][tar_x+search_x[i]].status==close_status || cost[tar_y+search_y[i]][tar_x+search_x[i]].status==open_status){
                if(cost[tar_y+search_y[i]][tar_x+search_x[i]].sum_cost()<min_cost){
                    min_cost=cost[tar_y+search_y[i]][tar_x+search_x[i]].sum_cost();
                    mx=tar_x+search_x[i];
                    my=tar_y+search_y[i];
                }
            }
        }
        tar_x=mx;
        tar_y=my;
        cost[tar_y][tar_x].status=free_status;
        path_x.push_back(mx);
        path_y.push_back(my);
        plt::plot(path_x,path_y);
        plt::pause(0.001);
    }

    plt::show();

    
}

int main() {

    //costmap init
    Costmap costmap(10.0,10.0, 0.5);
    std::vector<int> wall_x;
    std::vector<int> wall_y;

    //外周の壁
    for(int y=0;y<costmap.cost_map.size();y++){
        costmap.cost_map[y][0]=100;
        costmap.cost_map[y][costmap.cost_map[0].size()-1]=100;
    }
    for(int x=0;x<costmap.cost_map.size();x++){
        costmap.cost_map[0][x]=100;
        costmap.cost_map[costmap.cost_map.size()-1][x]=100;
    }

    //障害物の壁
    for(int y=0;y<costmap.cost_map.size()-int(costmap.grid_y()*0.25);y++){
        costmap.cost_map[y][costmap.grid_x()*0.25]=100;
        costmap.cost_map[costmap.cost_map.size()-1-y][costmap.grid_x()*0.5]=100;
        costmap.cost_map[y][costmap.grid_x()*0.75]=100;
    }

    for(int y=0;y<costmap.cost_map.size();y++){
        for (int x=0;x<costmap.cost_map.size();x++){
            if(costmap.cost_map[y][x]==100){
                wall_x.push_back(x);
                wall_y.push_back(y);
            }
                
        }
    }

    //壁の描画
    std::map<std::string,std::string> key;
    key["c"]="k";
    plt::scatter(wall_x,wall_y,100.0,key);


    Position start_pos(1.0,2.0);
    Position goal_pos(8.5,0.5);
    Astar astar;

    plt::xlim(-1, int(costmap.width/costmap.resolution)+1);
    plt::ylim(-1, int(costmap.height/costmap.resolution)+1);

    astar.calc(costmap,start_pos,goal_pos);
    
    plt::show();

    return 0;
}
